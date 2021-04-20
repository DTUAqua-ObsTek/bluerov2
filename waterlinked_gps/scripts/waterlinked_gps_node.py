#!/usr/bin/env python

# TODO:
# - Error handling when failing to connect to waterlinked GPS

import rospy
#
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Vector3
from requests.exceptions import ConnectionError
from requests_futures.sessions import FuturesSession
# Custom ROS messages
from waterlinked_gps_msgs.msg import About
from waterlinked_gps_msgs.msg import AboutStatus
from waterlinked_gps_msgs.msg import AboutTemperature
from waterlinked_gps_msgs.msg import ConfigGeneric
from waterlinked_gps_msgs.msg import ConfigReceivers
from waterlinked_gps_msgs.msg import ExternalOrientation
from waterlinked_gps_msgs.msg import PositionAcousticFiltered
from waterlinked_gps_msgs.msg import PositionAcousticRaw
from waterlinked_gps_msgs.msg import PositionGlobal
from waterlinked_gps_msgs.msg import PositionMaster
from waterlinked_gps_msgs.msg import Receiver
from tf2_ros import Buffer, TransformStamped, TransformBroadcaster
from geographic_msgs.msg import GeoPoint
from geodesy import utm
from scipy.spatial.transform import Rotation


class WaterlinkedGPS():
    def __init__(self):
        rospy.loginfo('Waterlinked GPS object created...\n')
        ip = rospy.get_param("~ip", "192.168.2.94")
        port = rospy.get_param("~port", "80")

        """
        Default TF Behaviour: If no datum point given, then waterlinked node sends utm->map transform referenced to the
        master GPS position and a utm->waterlinked transform referenced to the master GPS position and heading. Pose 
        data is referenced to waterlinked frame.
        Datum TF Behaviour: Datum [latitude, longitude] overrides default behaviour. utm->map transform referenced to the
        datum point given, ENU alignment. utm->waterlinked transform referenced to the master GPS position and heading. 
        Pose data is referenced to waterlinked frame.
        """

        self._map_frame_id = rospy.get_param("~map_frame_id", "map")
        self._waterlinked_frame_id = rospy.get_param("~waterlinked_frame_id", "waterlinked")
        self._send_tf = rospy.get_param("~send_tf", True)
        self._datum = rospy.get_param("~datum", None)  # if no datum specified

        self._tf_buffer = Buffer()
        self._tf_bcast = TransformBroadcaster()

        # The base URL is the one specified through: http://192.168.2.2:2770/waterlinked
        self._base_url = 'http://' + ip + ':' + port + '/api/v1'
        # self._base_url = 'http://192.168.2.94:80/api/v1'

        # The complete API can be found here: http://192.168.2.94/swagger/
        # Divide the messages into a slow and a fast group.
        self._api_endpoints_slow = ['/about',
                                    '/about/status',
                                    '/about/temperature',
                                    '/config/generic',
                                    '/config/receivers']

        self._api_endpoints_fast = ['/external/orientation',
                                    '/position/acoustic/filtered',
                                    '/position/acoustic/raw',
                                    '/position/global',
                                    '/position/master']

        # Create lists of the full APIs (base URL + endpoint)
        self._urls_slow = [self._base_url + api_endpoint for api_endpoint in self._api_endpoints_slow]
        self._urls_fast = [self._base_url + api_endpoint for api_endpoint in self._api_endpoints_fast]

        # Specify the frequencies for the two groups
        self._rate_slow_hz = 0.25
        self._rate_fast_hz = 4.0

        # Print the URLs and their specified frequencies
        self.print_urls()

        # HTTP request session
        self._session = FuturesSession(max_workers=10)

        # Configure the slow and fast timer callbacks
        rospy.Timer(rospy.Duration.from_sec(1.0 / self._rate_slow_hz), self.slow_callback)
        rospy.Timer(rospy.Duration.from_sec(1.0 / self._rate_fast_hz), self.fast_callback)

        # Time logging variables
        self._show_loop_timing = False
        self._is_first_slow_loop = True
        self._is_first_fast_loop = True

        # Slow publishers
        self._pub_about = rospy.Publisher('waterlinked/about', About, queue_size=5)
        self._pub_about_status = rospy.Publisher('waterlinked/about/status', AboutStatus, queue_size=5)
        self._pub_about_temperature = rospy.Publisher('waterlinked/about/temperature', AboutTemperature, queue_size=5)
        self._pub_config_generic = rospy.Publisher('waterlinked/config/generic', ConfigGeneric, queue_size=5)
        self._pub_config_receivers = rospy.Publisher('waterlinked/config/receivers', ConfigReceivers, queue_size=5)

        # Fast publishers
        self._pub_external_orientation = rospy.Publisher('waterlinked/external/orientation', ExternalOrientation,
                                                         queue_size=5)
        self._pub_position_acoustic_filtered = rospy.Publisher('waterlinked/position/acoustic/filtered',
                                                               PositionAcousticFiltered, queue_size=5)
        self._pub_position_acoustic_raw = rospy.Publisher('waterlinked/position/acoustic/raw', PositionAcousticRaw,
                                                          queue_size=5)
        self._pub_position_global = rospy.Publisher('waterlinked/position/global', PositionGlobal, queue_size=5)
        self._pub_position_master = rospy.Publisher('waterlinked/position/master', PositionMaster, queue_size=5)

        # Prepare sensor data for the robot_localization package
        self._pub_pos_with_covariance_stamped = rospy.Publisher('waterlinked/pose_with_cov_stamped',
                                                                PoseWithCovarianceStamped, queue_size=5)

        # Enter infinite spinning
        rospy.spin()

    def connection_error(self):
        rospy.logerr_throttle(10, "{} | Unable to connect to Waterlinked GPS on: {}".format(rospy.get_name(), self._base_url))

    def print_urls(self):
        message = 'Waterlinked APIs to be requested (see http://37.139.8.112:8000/swagger/):\n'
        message += 'Slow (f = ' + str(self._rate_slow_hz) + ' Hz)\n'
        for url_str in self._urls_slow:
            message += '- ' + url_str +'\n'
        message += 'Fast (f = ' + str(self._rate_fast_hz) + ' Hz)\n'
        for url_str in self._urls_fast:
            message += '- ' + url_str +'\n'
        rospy.loginfo('{} | {}'.format(rospy.get_name(), message))

    def slow_callback(self, event):
        """ Callback function that requests Waterlinked status and config
        settings at a low rate. """

        if self._show_loop_timing:
            tnow = rospy.Time.now().to_sec()
            if self._is_first_slow_loop:
                self._is_first_slow_loop = False
                self.f_cum_slow = 0
                self.n_slow = 0
            else:
                f = 1 / (tnow - self._slow_t0)
                self.f_cum_slow += f
                self.n_slow += 1
                f_avg = self.f_cum_slow / self.n_slow
                print("slow loop (n = %d): f_avg = %.3f Hz" % (self.n_slow, f_avg))
            self._slow_t0 = tnow

        # Initiate HTTP request to all URLs
        future_list = [self._session.get(url, timeout=0.5) for url in self._urls_slow]

        # Request current time and use it for all messages
        tnow = rospy.Time.now()

        try:
            # waterlinked/about
            res_about = future_list[0].result()

            if res_about.ok:
                data = res_about.json()
                msg_about = About()
                msg_about.header.stamp = tnow
                msg_about.chipid = data['chipid']
                msg_about.version = data['version']
                self._pub_about.publish(msg_about)

            # waterlinked/about/status
            res_about_status = future_list[1].result()
            if res_about_status.ok:
                data = res_about_status.json()
                msg_about_status = AboutStatus()
                msg_about_status.header.stamp = tnow
                msg_about_status.gps = data['gps']
                msg_about_status.imu = data['imu']
                self._pub_about_status.publish(msg_about_status)

            # waterlinked/about/temperature
            res_about_temperature = future_list[2].result()
            if res_about_temperature.ok:
                data = res_about_temperature.json()
                msg_about_temperature = AboutTemperature()
                msg_about_temperature.header.stamp = tnow
                msg_about_temperature.board = data['board']
                self._pub_about_temperature.publish(msg_about_temperature)

            # waterlinked/config/generic
            res_config_generic = future_list[3].result()
            if res_config_generic:
                data = res_config_generic.json()
                msg_config_generic = ConfigGeneric()
                msg_config_generic.header.stamp = tnow
                msg_config_generic.carrier_frequency = data['carrier_frequency']
                msg_config_generic.compass = data['compass'].encode('ascii', 'ignore')
                msg_config_generic.gps = data['gps'].encode('ascii', 'ignore')
                msg_config_generic.range_max_x = data['range_max_x']
                msg_config_generic.range_max_y = data['range_max_y']
                msg_config_generic.range_max_z = data['range_max_z']
                msg_config_generic.range_min_x = data['range_min_x']
                msg_config_generic.range_min_y = data['range_min_y']
                msg_config_generic.static_lat = data['static_lat']
                msg_config_generic.static_lon = data['static_lon']
                msg_config_generic.static_orientation = data['static_orientation']
                msg_config_generic.use_external_depth = data['use_external_depth']
                self._pub_config_generic.publish(msg_config_generic)

            # waterlinked/config/receivers
            res_config_receivers = future_list[4].result()
            # TODO check z axis is FLU oriented
            if res_config_receivers.ok:
                data = res_config_receivers.json()
                msg_config_receivers = ConfigReceivers()
                msg_config_receivers.header.stamp = tnow
                msg_config_receivers.receivers = []
                for i in range(len(data)):
                    rec = Receiver()
                    rec.id = data[i]['id']
                    rec.x = data[i]['x']
                    rec.y = data[i]['y']
                    rec.z = data[i]['z']
                    msg_config_receivers.receivers.append(rec)
                self._pub_config_receivers.publish(msg_config_receivers)
        except ConnectionError as e:
            self.connection_error()

    def fast_callback(self, event):
        """ Callback function that requests Waterlinked position and orientation
        information at a fast rate. """

        if self._show_loop_timing:
            tnow = rospy.Time.now().to_sec()
            if self._is_first_fast_loop:
                self._is_first_fast_loop = False
                self.f_cum_fast = 0
                self.n_fast = 0
            else:
                f = 1 / (tnow - self._fast_t0)
                self.f_cum_fast += f
                self.n_fast += 1
                f_avg = self.f_cum_fast / self.n_fast
                print("fast loop (n = %d): f_avg = %.3f Hz" % (self.n_fast, f_avg))
            self._fast_t0 = tnow

        # Initiate HTTP request to all URLs
        future_list = [self._session.get(url, timeout=0.5) for url in self._urls_fast]

        # Request current time and use it for all messages
        tnow = rospy.Time.now()

        try:
            # /waterlinked/external/orientation
            res_external_orientation = future_list[0].result()
            if res_external_orientation.ok:
                data = res_external_orientation.json()
                msg_external_orientation = ExternalOrientation()
                msg_external_orientation.header.stamp = tnow
                msg_external_orientation.orientation = data['orientation']
                self._pub_external_orientation.publish(msg_external_orientation)

            # /waterlinked/position/acoustic/filtered
            res_position_acoustic_filtered = future_list[1].result()

            if res_position_acoustic_filtered.ok:
                data = res_position_acoustic_filtered.json()
                msg_position_acoustic_filtered = PositionAcousticFiltered()
                msg_position_acoustic_filtered.header.stamp = tnow
                msg_position_acoustic_filtered.header.frame_id = self._waterlinked_frame_id
                msg_position_acoustic_filtered.std = data['std']
                msg_position_acoustic_filtered.temp = data['temp']
                msg_position_acoustic_filtered.x = data['x']
                msg_position_acoustic_filtered.y = data['y']
                msg_position_acoustic_filtered.z = data['z']
                self._pub_position_acoustic_filtered.publish(msg_position_acoustic_filtered)
                # Create message of the type geometry_msgs/PoseWithCovariance
                msg_pose_with_cov_stamped = PoseWithCovarianceStamped()
                var_xyz = pow(data['std'], 2)  # calculate variance from standard deviation
                msg_pose_with_cov_stamped.header.stamp = tnow
                msg_pose_with_cov_stamped.header.frame_id = self._waterlinked_frame_id
                msg_pose_with_cov_stamped.pose.pose.position.x = data['x']
                msg_pose_with_cov_stamped.pose.pose.position.y = data['y']
                # TODO check z axis is FLU oriented
                msg_pose_with_cov_stamped.pose.pose.position.z = data['z']
                msg_pose_with_cov_stamped.pose.pose.orientation = Quaternion(0, 0, 0, 1)
                msg_pose_with_cov_stamped.pose.covariance = [var_xyz, 0, 0, 0, 0, 0,
                                                             0, var_xyz, 0, 0, 0, 0,
                                                             0, 0, var_xyz, 0, 0, 0,
                                                             0, 0, 0, 0, 0, 0,
                                                             0, 0, 0, 0, 0, 0,
                                                             0, 0, 0, 0, 0, 0]
                self._pub_pos_with_covariance_stamped.publish(msg_pose_with_cov_stamped)

            # /waterlinked/position/acoustic/raw
            res_position_acoustic_raw = future_list[2].result()
            if res_position_acoustic_raw.ok:
                data = res_position_acoustic_raw.json()
                msg_position_acoustic_raw = PositionAcousticRaw()
                msg_position_acoustic_raw.header.stamp = tnow
                msg_position_acoustic_raw.header.frame_id = self._waterlinked_frame_id
                msg_position_acoustic_raw.std = data['std']
                msg_position_acoustic_raw.temp = data['temp']
                msg_position_acoustic_raw.x = data['x']
                msg_position_acoustic_raw.y = data['y']
                # TODO check z axis is FLU oriented
                msg_position_acoustic_raw.z = data['z']
                self._pub_position_acoustic_raw.publish(msg_position_acoustic_raw)

            # /waterlinked/position/global
            res_position_global = future_list[3].result()
            if res_position_global.ok:
                data = res_position_global.json()
                msg_position_global = PositionGlobal()
                msg_position_global.header.stamp = tnow
                msg_position_global.lat = data['lat']
                msg_position_global.lon = data['lon']
                self._pub_position_global.publish(msg_position_global)

            # /waterlinked/position/master
            res_position_master = future_list[4].result()
            if res_position_master.ok:
                data = res_position_master.json()
                msg_position_master = PositionMaster()
                msg_position_master.header.stamp = tnow
                msg_position_master.cog = data['cog']
                msg_position_master.hdop = data['hdop']
                msg_position_master.lat = data['lat']
                msg_position_master.lon = data['lon']
                msg_position_master.numsats = data['numsats']
                msg_position_master.orientation = data['orientation']
                msg_position_master.sog = data['sog']
                self._pub_position_master.publish(msg_position_master)

            # Todo Send utm->map, utm->waterlinked transforms here
            if self._send_tf:
                tf_loc = TransformStamped()
                tf_loc.header.stamp = tnow
                tf_loc.header.frame_id = "utm"
                tf_loc.child_frame_id = self._waterlinked_frame_id
                tf_map = TransformStamped()
                tf_map.header.stamp = tnow
                tf_map.header.frame_id = "utm"
                tf_map.child_frame_id = self._map_frame_id
                tf_map.transform.rotation = Quaternion(0, 0, 0, 1)
                geopoint = GeoPoint(msg_position_master.lat, msg_position_master.lon, 0.0)
                utmpoint = utm.fromMsg(geopoint)
                tf_loc.transform.translation = Vector3(utmpoint.easting, utmpoint.northing, 0.0)
                q = Rotation.from_euler('xyz', [0, 0, msg_position_master.orientation]).as_quat()
                tf_loc.transform.rotation = Quaternion(*q)
                if self._datum is None:
                    tf_map.transform.translation = Vector3(utmpoint.easting, utmpoint.northing, 0.0)
                else:
                    geopoint = GeoPoint(self._datum[0], self._datum[1], 0.0)
                    utmpoint = utm.fromMsg(geopoint)
                    tf_map.transform.translation = Vector3(utmpoint.easting, utmpoint.northing, 0.0)
                self._tf_bcast.sendTransform(tf_map)
                self._tf_bcast.sendTransform(tf_loc)
        except ConnectionError as e:
            self.connection_error()

# The main function
if __name__ == '__main__':
    rospy.init_node('waterlinked_gps_node')
    try:
        wl_gps = WaterlinkedGPS()
    except rospy.ROSInterruptException:
        pass
