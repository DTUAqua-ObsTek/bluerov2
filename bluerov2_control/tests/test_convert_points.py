#!/usr/bin/env python
import rospy
from bluerov2_control.srv import ConvertGeoPoints, ConvertGeoPointsRequest
from geographic_msgs.msg import GeoPoint


if __name__=="__main__":
    rospy.init_node("test_convert_points")
    proxy = rospy.ServiceProxy("/convert_points", ConvertGeoPoints)
    proxy.wait_for_service()
    req = ConvertGeoPointsRequest()
    req.geopoints = [GeoPoint(1,2,3), GeoPoint(-1,-2,-3), GeoPoint(4, 5, 6)]
    res = proxy.call(req)
    rospy.loginfo(res)