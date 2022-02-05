#!/usr/bin/env python3

import rospy
from bluerov2_hmi import HudOverlay


def main():
    rospy.init_node("hud_overlay_node")
    try:
        obj = HudOverlay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()