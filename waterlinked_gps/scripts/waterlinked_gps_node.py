#!/usr/bin/env python

# TODO:
# - Error handling when failing to connect to waterlinked GPS

import rospy
#
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Vector3, TwistStamped
from requests.exceptions import ConnectionError
from requests_futures.sessions import FuturesSession
# Custom ROS messages
from waterlinked_gps_msgs.msg import *
from tf2_ros import Buffer, TransformStamped, TransformBroadcaster
from geographic_msgs.msg import GeoPoint
from geodesy import utm
from scipy.spatial.transform import Rotation
import numpy as np
from pyproj import Proj
from math import pi, atan, tan, sin, sqrt, atan2
from sensor_msgs.msg import NavSatFix


def is_south(band):
    """A way of finding hemisphere from band"""
    alpha = {c: num for num,c in enumerate(string.ascii_uppercase)}
    return alpha[band] < 12


def calc_grid_convergence(utmpoint):
    """A way of calculating grid convergence for a UTM point."""
    p = Proj(south=is_south(utmpoint.band), proj='utm', zone=utmpoint.zone, ellps='WGS84')
    lon_pm, lat_pm = p(500000, utmpoint.northing, inverse=True)
    return atan(tan(pi*lon_pm/180.0)*sin(pi*lat_pm/180.0))


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
        self._send_tf = rospy.get_param("~send_tf", False)
        self._datum = rospy.get_param("~datum", None)  # if no datum specified
        self._master_gps_ns = rospy.get_param("~master_gps_ns", None)  # None
        self._master_orientation_topic = rospy.get_param("~master_imu_topic", None)

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

        # If a master gps topic has been specified, then forward that to waterlinked for use
        if self._master_gps_ns is not None:
            self._gps_msg = {
                              "cog": 0,
                              "fix_quality": 1,
                              "hdop": 0,
                              "lat": 0.0,
                              "lon": 0.0,
                              "numsats": 11,
                              "orientation": 0.0,
                              "sog": 0.0
                            }
            self._gps_url = self._base_url + "/external/master"
            rospy.Subscriber(self._master_gps_ns+"/fix", NavSatFix, self._handle_master_gps)
            rospy.Subscriber(self._master_gps_ns+"/vel", TwistStamped, self._handle_master_vel)
            rospy.Timer(rospy.Duration.from_sec(1.0), self._forward_master_position)

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
        message = 'Waterlinked APIs to be requested (see https://demo.waterlinked.com/swagger/#/):\n'
        message += 'Slow (f = ' + str(self._rate_slow_hz) + ' Hz)\n'
        for url_str in self._urls_slow:
            message += '- ' + url_str +'\n'
        message += 'Fast (f = ' + str(self._rate_fast_hz) + ' Hz)\n'
        for url_str in self._urls_fast:
            message += '- ' + url_str +'\n'
        rospy.loginfo('{} | {}'.format(rospy.get_name(), message))

    def _forward_master_position(self, event):
        """ If an external gps topic is subscribed, forward the latitude and longitude over to waterlinked."""
        try:
            r = self._session.put(self._gps_url, json=self._gps_msg, timeout=2)
            r = r.result(10)
        except Exception as e:
            rospy.logerr_throttle(10.0, "{} | {}".format(rospy.get_name(), e.message))
            return
        if r.status_code != 200:
            rospy.logerr("Error setting master position: {} {}".format(r.status_code, r.text))

    def _handle_master_gps(self, msg):
        """Fill in GPS information for message"""
        self._gps_msg["lat"] = msg.latitude
        self._gps_msg["lon"] = msg.longitude
        self._gps_msg["hdop"] = msg.position_covariance[0]

    def _handle_master_vel(self, msg):
        """Fill in GPS cog/sog for message"""
        self._gps_msg["sog"] = sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
        val = -1*(atan2(msg.twist.linear.y, msg.twist.linear.x) * 180.0 / pi - 90)
        val = 360 + val if val < 0 else val
        self._gps_msg["cog"] = val

    def slow_callback(self, event):
        """ Callback function that requests Waterlinked status and config
        settings at a low rate. """
        # Request current time and use it for all messages
        tnow = rospy.Time.now()
        if self._is_first_slow_loop:
            self._is_first_slow_loop = False
            self.f_cum_slow = 0
            self.n_slow = 0
            self._slow_t0 = tnow.to_sec()
        else:
            f = 1 / (tnow.to_sec() - self._slow_t0)
            self.f_cum_slow += f
            self.n_slow += 1
            f_avg = self.f_cum_slow / self.n_slow
            rospy.logdebug("slow loop (n = %d): f_avg = %.3f Hz" % (self.n_slow, f_avg))

        # Initiate HTTP request to all URLs
        future_list = [self._session.get(url, timeout=2.0) for url in self._urls_slow]

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
                #msg_config_generic.carrier_frequency = data['carrier_frequency']
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
                #msg_config_generic.use_external_depth = data['use_external_depth']
                self._pub_config_generic.publish(msg_config_generic)

            # waterlinked/config/receivers
            res_config_receivers = future_list[4].result()
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
        # Request current time and use it for all messages
        tnow = rospy.Time.now()
        if self._is_first_fast_loop:
            self._is_first_fast_loop = False
            self.f_cum_fast = 0
            self.n_fast = 0
            self._fast_t0 = tnow.to_sec()
        else:
            f = 1 / (tnow.to_sec() - self._fast_t0)
            self.f_cum_fast += f
            self.n_fast += 1
            f_avg = self.f_cum_fast / self.n_fast
            rospy.logdebug("fast loop (n = %d): f_avg = %.3f Hz" % (self.n_fast, f_avg))

        # Initiate HTTP request to all URLs
        future_list = [self._session.get(url, timeout=2.0) for url in self._urls_fast]

        try:
            # WARN: ORIENTATION IS CLOCKWISE REFERENCED FROM MAGNETIC NORTH
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

            # WARN: WATERLINKED POSITION IS LEFT HANDED RFD -> X: RIGHT, y: FORWARDS, Z: DOWN
            # DO NOT USE ACOUSTIC_FILTERED FOR NAVIGATION!
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
                if self._pub_position_acoustic_filtered.get_num_connections() > 0:
                    rospy.logwarn_once("{} | waterlinked/acoustic_filtered is left-handed RFD, don't use for navigation, "
                                       "use waterlinked/pose_with_cov_stamped (FLU) instead.")
                self._pub_position_acoustic_filtered.publish(msg_position_acoustic_filtered)

                # Create message of the type geometry_msgs/PoseWithCovariance
                msg_pose_with_cov_stamped = PoseWithCovarianceStamped()
                var_xyz = pow(data['std'], 2)  # calculate variance from standard deviation
                msg_pose_with_cov_stamped.header.stamp = tnow
                msg_pose_with_cov_stamped.header.frame_id = self._waterlinked_frame_id
                msg_pose_with_cov_stamped.pose.pose.position.x = data['y']
                msg_pose_with_cov_stamped.pose.pose.position.y = -data['x']
                msg_pose_with_cov_stamped.pose.pose.position.z = -data['z']
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
                msg_position_acoustic_raw.z = data['z']
                if self._pub_position_acoustic_raw.get_num_connections() > 0:
                    rospy.logwarn_once("{} | waterlinked/acoustic_raw is left-handed RFD, don't use for navigation, "
                                       "use waterlinked/pose_with_cov_stamped (FLU) instead.")
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
            msg_position_master = None
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

            # CONVENTION: UTM -> WATERLINKED IS DEFINED BY UTM POSITION OF MASTER, ROTATED ACCORDING TO MASTER ORIENTATION
            # CONVENTION: UTM -> MAP IS DEFINED BY UTM POSITION OF MASTER, WITHOUT ANY ROTATION (ALIGNED WITH NORTH)
            # CONVENTION: UTM -> MAP CAN ALSO BE DEFINED BY AN EXTERNAL DATUM [LATITUDE, LONGITUDE]
            if self._send_tf and msg_position_master is not None:
                tf_map = TransformStamped()  # Map transformation
                tf_map.header.stamp = tnow
                tf_map.header.frame_id = self._map_frame_id
                tf_map.child_frame_id = self._waterlinked_frame_id
                # ORIENTATION IS PROVIDED AS NORTH REFERENCED CW
                # NEEDS TO BE CONVERTED TO EAST REFERENCED CCW
                q = Rotation.from_euler('xyz', [0, 0, 90-msg_position_master.orientation], degrees=True).as_quat()
                tf_map.transform.rotation = Quaternion(*q)
                self._tf_bcast.sendTransform(tf_map)
        except ConnectionError as e:
            self.connection_error()

# The main function
if __name__ == '__main__':
    rospy.init_node('waterlinked_gps_node')
    try:
        wl_gps = WaterlinkedGPS()
    except rospy.ROSInterruptException:
        pass
