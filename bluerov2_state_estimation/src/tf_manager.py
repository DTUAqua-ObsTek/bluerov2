#!/usr/bin/env python


import rospy
import tf2_geometry_msgs
import geodetic_helpers
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import Quaternion, Transform, Vector3, PoseWithCovarianceStamped, PointStamped, Vector3Stamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from scipy.spatial.transform import Rotation


class TFManager(object):
    """
    Listens to datum GPS or service and broadcasts:
        1. ECEF TO UTM
        2. UTM TO MAP
        3. MAP TO ODOM (fixed or updated by a pose with covariance stamped topic).
    """
    def __init__(self):
        self._datum = rospy.get_param("~datum", None)
        if self._datum is not None and len(self._datum) < 3:
            self._datum += [0.0]
        self._use_datum = self._datum is not None
        self._earth_frame_id = rospy.get_param("~earth_frame_id", "earth")
        self._utm_frame_id = rospy.get_param("~utm_frame_id", "utm")
        self._map_frame_id = rospy.get_param("~map_frame_id", "map")
        self._odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self._body_frame_id = rospy.get_param("~base_frame_id", "base_link")  # currently unused
        self._tf_broadcast_rate = rospy.get_param("~broadcast_rate", 10.0)
        self._serv = rospy.Service("~set_datum", SetDatum, self._set_datum)
        self._tf_buff = Buffer()
        TransformListener(self._tf_buff)
        self._tf_bcast = TransformBroadcaster()
        self._last_datum = None
        self._static_map_odom = rospy.get_param("~static_map_odom", False)
        self._odom_updated = False
        self._update_odom_serv = rospy.Service("~set_odom", Trigger, self._handle_set_odom)
        if not self._use_datum:
            rospy.Subscriber("gps_datum", NavSatFix, self._handle_datum)
        if not self._static_map_odom:
            rospy.Subscriber("waterlinked/pose_with_cov_stamped", PoseWithCovarianceStamped, self._handle_odom_tf)
        else:
            StaticTransformBroadcaster().sendTransform(TransformStamped(Header(0, rospy.Time.now(), self._map_frame_id),
                                                                        self._odom_frame_id,
                                                                        None,
                                                                        Quaternion(0,0,0,1)))
        self._map_odom_tf = None
        rospy.Timer(rospy.Duration.from_sec(1.0 / self._tf_broadcast_rate), self._send_tf)

    def _handle_set_odom(self, req):
        self._odom_updated = False
        res = TriggerResponse(True, "map -> odom tf set.")
        return res

    def _handle_odom_tf(self, msg):
        # Given the pose of the vehicle in waterlinked frame, output the map -> odom tf transformation
        if not self._odom_updated:
            point = PointStamped(Header(0, rospy.Time.from_sec(0.0), msg.header.frame_id),
                                 msg.pose.pose.position)
            try:
                point_map = self._tf_buff.transform(point, self._map_frame_id)
            except Exception as e:
                rospy.logerr_throttle(5, "{} | {}".format(rospy.get_name(), e.message))
                return
            # Odom is always at same depth as map
            self._map_odom_tf = TransformStamped(Header(0, rospy.Time.now(), self._map_frame_id),
                                                 self._odom_frame_id,
                                                 Transform(Vector3(point_map.point.x, point_map.point.y, 0),
                                                           Quaternion(0, 0, 0, 1)))
            self._odom_updated = True

    def _set_datum(self, req):
        self._datum = [req.geo_pose.position.latitude, req.geo_pose.position.longitude, req.geo_pose.position.altitude]
        return

    def _get_coords(self, latitude, longitude):
        # Get ECEF translation to UTM and Rotation to UTM from latitude and longitude (assuming 0.0 altitude)
        x, y, z = geodetic_helpers.utm_origin_ecef(*geodetic_helpers.lla_to_ecef(latitude, longitude))
        q = geodetic_helpers.latlong_ecef_enu_rotation(latitude, longitude)
        # Get UTM translation to ENU and rotation to ENU from latitude and longitude (assuming 0.0 altitude)
        utm_current = geodetic_helpers.lla_to_utm(latitude, longitude)
        Y = geodetic_helpers.grid_convergence(latitude, longitude, radians=False)
        Y = 0  # The grid convergence seems to be already accounted for?
        enu_rotation = Rotation.from_euler('xyz', [0.0, 0.0, -Y], degrees=True).as_quat().tolist()
        return (x, y, z) + tuple(q), utm_current[:3] + tuple(enu_rotation)

    def _get_tfs(self, data):
        if type(data) is NavSatFix:
            earth, utm = self._get_coords(data.latitude, data.longitude)
            header_earth = Header(data.header.seq, data.header.stamp, self._earth_frame_id)
            header_utm = Header(data.header.seq, data.header.stamp, self._utm_frame_id)
        else:
            earth, utm = self._get_coords(self._datum[0], self._datum[1])
            header_earth = Header(0, rospy.Time.now(), self._earth_frame_id)
            header_utm = Header(0, rospy.Time.now(), self._utm_frame_id)
        # Broadcast datum's latitude and longitude
        # Map from ECEF Frame
        earth_to_utm = TransformStamped(header_earth,
                                       self._utm_frame_id,
                                       Transform(Vector3(*earth[:3]),
                                                 Quaternion(*earth[3:])))
        # UTM from ECEF Frame
        utm_to_map = TransformStamped(header_utm,
                                       self._map_frame_id,
                                       Transform(Vector3(*utm[:3]),
                                                 Quaternion(*utm[3:])))
        return earth_to_utm, utm_to_map

    def _handle_datum(self, msg):
        self._last_datum = msg

    def _send_tf(self, event):
        if self._use_datum:
            tf_utm, tf_map = self._get_tfs(None)
        elif self._last_datum is not None:
            tf_utm, tf_map = self._get_tfs(self._last_datum)
        else:
            return
        self._tf_bcast.sendTransform(tf_utm)
        self._tf_bcast.sendTransform(tf_map)
        if self._map_odom_tf is not None:
            tf = self._map_odom_tf
            tf.header.stamp = rospy.Time.now()
            self._tf_bcast.sendTransform(tf)


if __name__=="__main__":
    rospy.init_node("tf_manager")
    try:
        manager = TFManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass