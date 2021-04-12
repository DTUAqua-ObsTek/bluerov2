#!/usr/bin/env python

import rospy
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest, InitWaypointSetResponse
from uuv_control_msgs.msg import Waypoint
from bluerov2_control.srv import ConvertGeoPointsRequest, ConvertGeoPoints
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String, Time
from tf2_ros import Buffer, TransformListener
import sys
import os
import re



def parse_iver_format(string):
    values = re.split(', | ',string)
    return values

def get_request(mis):

    with open(mis, "r") as f:
        while not f.readline().startswith("START"):
            pass
        geopoints = []
        headings = []
        speeds = []
        line = f.readline().rstrip()
        while not line.startswith("END"):
            values = line.split(";")
            values = [value.strip() for value in values]
            params = parse_iver_format(values[5])
            geopoints.append(GeoPoint(float(values[1]),float(values[2]),float(params[0][1:])*0.3048))
            headings.append(float(values[4]))
            speeds.append(float(params[-1][1:])*0.5144)
            line = f.readline().rstrip()
    req = ConvertGeoPointsRequest()
    req.geopoints = geopoints
    p = rospy.ServiceProxy("convert_points", ConvertGeoPoints)
    p.wait_for_service()
    res = p(req)
    print req.geopoints
    buff = Buffer()
    TransformListener(buff)
    tf = buff.lookup_transform("world", "utm", rospy.Time.now(), rospy.Duration.from_sec(5.0))
    wps = []
    for point, heading, speed in zip(res.utmpoints, headings, speeds):
        wp = Waypoint()
        wp.point.x = point.x + tf.transform.translation.x
        wp.point.y = point.y + tf.transform.translation.y
        wp.point.z = -point.z
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "world"
        wp.radius_of_acceptance = 3.0
        wp.max_forward_speed = speed
        wp.use_fixed_heading = False
        wp.heading_offset = 0.0
        wps.append(wp)
    req = InitWaypointSetRequest()
    req.start_time = Time(rospy.Time.from_sec(0))
    req.start_now = True
    req.waypoints = wps
    req.max_forward_speed = max(speeds)
    req.interpolator = String("lipb")
    req.heading_offset = 0
    req.max_forward_speed = max(speeds)
    req.waypoints = wps
    return req

if __name__=="__main__":
    if len(sys.argv) < 2:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './import_iver_mis.py '
                     'iver_mission.mis')
        sys.exit(0)
    else:
        mission_filename = sys.argv[1] if sys.argv[1].endswith(".mis") else sys.argv[1]+".mis"
        if not os.path.exists(mission_filename):
            rospy.logerr("File not found: {}".format(mission_filename))
        else:
            rospy.loginfo("Parsing Iver Mission File: {}".format(mission_filename))

    rospy.init_node("set_iver_waypoints")
    req = get_request(mission_filename)
    proxy = rospy.ServiceProxy("start_waypoint_list", InitWaypointSet)
    proxy.wait_for_service()
    print req
    res = proxy(req)
    if res.success:
        rospy.loginfo("IVER Waypoints Set, Executing.")
    else:
        rospy.logerr("IVER Waypoints Error.")
