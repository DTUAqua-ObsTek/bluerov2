#!/usr/bin/env python3


import rospy
import tf2_ros, tf2_geometry_msgs
from geographic_msgs.msg import GeoPoint
from bluerov2_navigation.helpers.geoid import GeoidHeight
from bluerov2_navigation.helpers import geodetic


def broadcast_tf(broadcaster: tf2_ros.TransformBroadcaster, transform: tf2_ros.TransformStamped, datum: GeoPoint):
    transform.header.stamp = rospy.Time.now()
    geodetic.geodetic2enu(datum.latitude, datum.longitude, datum.altitude)




def main():
    rospy.init_node("map_origin_broadcaster")
    datum = rospy.get_param("~datum", None)
    earth_frame = rospy.get_param("~earth_frame_id", "earth")
    origin_frame = rospy.get_param("~origin_frame_id", "map_origin")
    tf_msg = tf2_ros.TransformStamped()
    tf_msg.header.frame_id = earth_frame
    tf_msg.child_frame_id = origin_frame



if __name__=="__main__":
    main()