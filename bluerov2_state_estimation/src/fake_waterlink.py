#!/usr/bin/env python


import rospy
from tf2_ros import Buffer, TransformStamped, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import Quaternion, PointStamped, PoseWithCovarianceStamped, Transform, Point
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geodetic_helpers import lla_to_utm
import tf2_geometry_msgs


def handle_gps(msg, args):
    """
        Publish the GPS position in waterlinked frame.
    """
    pub, tf = args
    e, n, a, zone, band = lla_to_utm(msg.latitude, msg.longitude, 0.0, radians=False)
    rospy.logdebug("utm: {} {} {}".format(e, n, a))
    p_utm = PointStamped(Header(0, rospy.Time.from_sec(0.0), "utm"), Point(e, n, a))
    # tf = Buffer()
    try:
        p_wl = tf.transform(p_utm, "waterlinked")
        rospy.logdebug("waterlinked: {} {} {}".format(p_wl.point.x, p_wl.point.y, p_wl.point.z))
    except Exception as e:
        rospy.logerr_throttle(5.0, "{} | {}".format(rospy.get_name(), e.message))
        return
    msg_pose_with_cov_stamped = PoseWithCovarianceStamped()
    var_xyz = pow(1.5, 2)  # calculate variance from standard deviation
    msg_pose_with_cov_stamped.header.stamp = msg.header.stamp
    msg_pose_with_cov_stamped.header.frame_id = p_wl.header.frame_id
    msg_pose_with_cov_stamped.pose.pose.position.x = p_wl.point.x
    msg_pose_with_cov_stamped.pose.pose.position.y = p_wl.point.y
    msg_pose_with_cov_stamped.pose.pose.position.z = p_wl.point.z
    msg_pose_with_cov_stamped.pose.pose.orientation = Quaternion(0,0,0,1)
    msg_pose_with_cov_stamped.pose.covariance = [var_xyz, 0, 0, 0, 0, 0,
                                                 0, var_xyz, 0, 0, 0, 0,
                                                 0, 0, var_xyz, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0]
    pub.publish(msg_pose_with_cov_stamped)


if __name__=="__main__":
    rospy.init_node("fake_waterlinked", log_level=rospy.INFO)
    heading_offset = rospy.get_param("~heading")  # heading is given by waterlinked GPS in degrees CW from North
    master_datum = rospy.get_param("~datum")  # Master located (latitude, longitude)
    lat, lon, alt = master_datum + [0.0] if len(master_datum) < 3 else master_datum
    master_gps = rospy.Publisher('gps_datum', NavSatFix, queue_size=5)
    master_msg = NavSatFix()
    master_msg.altitude = alt
    master_msg.longitude = lon
    master_msg.latitude = lat
    master_msg.status = NavSatStatus()
    master_msg.status.status = master_msg.status.STATUS_FIX
    master_msg.status.service = master_msg.status.SERVICE_GPS
    master_msg.position_covariance_type = master_msg.COVARIANCE_TYPE_UNKNOWN
    master_msg.position_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
    pose_pub = rospy.Publisher('waterlinked/pose_with_cov_stamped', PoseWithCovarianceStamped, queue_size=5)
    buff = Buffer()
    TransformListener(buff)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, handle_gps, (pose_pub, buff))
    map_to_waterlink = TransformStamped(Header(0, rospy.Time.now(), 'map'), 'waterlinked',
                                        Transform(None,
                                                  Quaternion(*Rotation.from_euler('xyz', [0, 0, 90 - heading_offset],
                                                                                  degrees=True).as_quat().tolist())))
    StaticTransformBroadcaster().sendTransform(map_to_waterlink)
    rate = rospy.Rate(4.0)
    while not rospy.is_shutdown():
        master_msg.header.seq += 1
        master_msg.header.stamp = rospy.Time.now()
        master_gps.publish(master_msg)
        rate.sleep()
