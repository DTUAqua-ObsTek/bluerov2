#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import RCOut
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from bluerov2_navigation.helpers import math


def handle_RCOut_cb(msg: RCOut, args: tuple):
    camera_servo_pwm = msg.channels[6]
    try:
        camera_servo_angle = args[0](camera_servo_pwm)
    except ValueError:
        rospy.logwarn_throttle(10, "camera_tf_node | could not interpolate.")
        return
    # tf_msg = tf2_ros.TransformStamped()
    # tf_msg.header.stamp = msg.header.stamp
    # tf_msg.header.frame_id = args[2]
    # tf_msg.child_frame_id = args[3]
    # tf_msg.transform.rotation = Quaternion(*Rotation.from_euler("xyz", [camera_servo_angle, 0, 0], degrees=True).as_quat())
    # args[1].sendTransform(tf_msg)
    joint_msg = JointState()
    joint_msg.header = msg.header
    joint_msg.name = [f"{args[2]}_{args[3]}"]
    joint_msg.position = [math.deg2rad(camera_servo_angle)]
    joint_msg.velocity = [0]
    joint_msg.effort = [0]
    rospy.logdebug(f"{camera_servo_angle}\t{camera_servo_pwm}")
    args[1].publish(joint_msg)


if __name__ == "__main__":
    rospy.init_node("camera_tf_node")
    broadcaster = tf2_ros.TransformBroadcaster()
    publisher = rospy.Publisher("joint_states", JointState, queue_size=10)
    interp = interp1d(np.linspace(1100, 1900, 10), np.linspace(-45, 45, 10), bounds_error=False, fill_value=(-45, 45))
    rospy.Subscriber("mavros/rc/out", RCOut, handle_RCOut_cb, callback_args=(interp, publisher, rospy.get_param("base_frame_id", "base_link"), rospy.get_param("nose_camera_frame_id", "nose_cam")))
    rospy.spin()