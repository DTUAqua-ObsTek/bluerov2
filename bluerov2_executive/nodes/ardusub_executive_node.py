#!/usr/bin/env python3

import rospy
from bluerov2_executive.interfaces import ImcInterface


def main():
    rospy.init_node("executive")
    try:
        obj = ImcInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()