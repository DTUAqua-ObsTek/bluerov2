#!/usr/bin/env python
import rospy
from bluerov2_control.controllers.cascade_pid import Cascade4DoF


if __name__ == "__main__":
    rospy.init_node("controller")#, log_level=rospy.DEBUG)
    try:
        controller = Cascade4DoF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
