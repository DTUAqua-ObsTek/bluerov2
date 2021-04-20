#!/usr/bin/env python


import rospy
from waterlinked_gps_msgs.msg import PositionAcousticFiltered
import geodesy.utm as utm
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformStamped, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import Vector3, Quaternion, PointStamped, PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation
from numpy import pi
import tf2_geometry_msgs


def handle_gps(msg, args):
    base, pose, buff = args
    base_pos = utm.fromMsg(GeoPoint(msg.latitude, msg.longitude, msg.altitude))  # utm position of base
    base_utm = PointStamped()
    base_utm.header.stamp = msg.header.stamp
    base_utm.header.frame_id = "utm"
    base_utm.point = base_pos.toPoint()
    base_waterlinked = buff.transform(base_utm, "waterlinked", rospy.Duration(1))
    out = PositionAcousticFiltered()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = "waterlinked"
    out.x = base_waterlinked.point.x
    out.y = base_waterlinked.point.y
    out.z = base_waterlinked.point.z
    out.temp = 69.0
    out.std = 1.5

    msg_pose_with_cov_stamped = PoseWithCovarianceStamped()
    var_xyz = pow(1.5, 2)  # calculate variance from standard deviation
    msg_pose_with_cov_stamped.header.stamp = msg.header.stamp
    msg_pose_with_cov_stamped.header.frame_id = 'waterlinked'
    msg_pose_with_cov_stamped.pose.pose.position.x = base_waterlinked.point.x
    msg_pose_with_cov_stamped.pose.pose.position.y = base_waterlinked.point.y
    msg_pose_with_cov_stamped.pose.pose.position.z = base_waterlinked.point.z
    msg_pose_with_cov_stamped.pose.covariance = [var_xyz, 0, 0, 0, 0, 0,
                                                 0, var_xyz, 0, 0, 0, 0,
                                                 0, 0, var_xyz, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0]
    pose.publish(msg_pose_with_cov_stamped)
    base.publish(out)


if __name__=="__main__":
    rospy.init_node("fake_waterlinked")
    buff = Buffer()
    TransformListener(buff)
    heading_offset = rospy.get_param("~heading", 0.0)  # assume that this is ENU referenced
    master_datum = rospy.get_param("~datum", [0.0,0.0,0.0])
    map_pos = utm.fromMsg(GeoPoint(*master_datum))  # utm position of master (origin of map frame)
    tfutm2master = TransformStamped()
    tfutm2master.header.stamp = rospy.Time.now()
    tfutm2master.header.frame_id = "utm"
    tfutm2master.child_frame_id = "waterlinked"
    tfutm2master.transform.translation = Vector3(map_pos.easting, map_pos.northing, map_pos.altitude)
    q = Rotation.from_euler('xyz', [0, 0, heading_offset]).as_quat()
    tfutm2master.transform.rotation = Quaternion(*q)
    tfutm2map = TransformStamped()
    tfutm2map.header.stamp = rospy.Time.now()
    tfutm2map.header.frame_id = "utm"
    tfutm2map.child_frame_id = "map"
    tfutm2map.transform.translation = Vector3(map_pos.easting, map_pos.northing, map_pos.altitude)
    tfutm2map.transform.rotation = Quaternion(0,0,0,1)
    StaticTransformBroadcaster().sendTransform([tfutm2master, tfutm2map])
    base = rospy.Publisher('waterlinked/position/acoustic/filtered', PositionAcousticFiltered, queue_size=5)
    pose = rospy.Publisher('waterlinked/pose_with_cov_stamped', PoseWithCovarianceStamped, queue_size=5)
    rospy.Subscriber("mavros/global_position/global", NavSatFix, handle_gps, (base, pose, buff))
    rospy.spin()