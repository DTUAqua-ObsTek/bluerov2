#!/usr/bin/env python


import rospy
from bluerov2_msgs.srv import ConvertGeoPoints, ConvertGeoPointsRequest, SetControllerState, SetControllerStateRequest
from bluerov2_msgs.msg import FollowWaypointsGoal, FollowWaypointsResult, FollowWaypointsAction, FollowWaypointsFeedback
from geometry_msgs.msg import Pose, Point
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Empty, Time, String, Header
from imc_ros_bridge.msg import PlanDB, EstimatedState, PlanControlState, PlanControl, PlanDBInformation, PlanDBState, PlanManeuver
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from mavros_msgs.msg import Waypoint, WaypointList
import numpy as np
import imc_enums
import hashlib
from io import StringIO
import time
import actionlib
from actionlib_msgs.msg import GoalStatus


class MainInterface:
    """
    This object exposes services to operate an actionlib client for the guidance, navigation, control structure.
    """
    def __init__(self):
        pass


class ImcInterface(object):
    def __init__(self):
        self._estimated_state_pub = rospy.Publisher("imc/estimated_state", EstimatedState, queue_size=5)
        self._estimated_state_msg = EstimatedState()
        self._geo_converter = rospy.ServiceProxy("convert_points", ConvertGeoPoints)
        self._plan_db = dict()
        self._current_plan = PlanDB()
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer)
        rospy.Subscriber("gps", NavSatFix, self._handle_gps)
        rospy.Subscriber("odometry", Odometry, self._handle_pose)
        rospy.Subscriber("imc/goto_waypoint", Pose, self._handle_goto)
        rospy.Subscriber("imc/plan_control", PlanControl, self._handle_plan_control)
        rospy.Subscriber("imc/abort", Empty, self._handle_abort)
        rospy.Subscriber("imc/imc_heartbeat", Empty, self._handle_imc_heartbeat)
        rospy.Subscriber("imc/plan_db", PlanDB, self._handle_plan_db)
        rospy.Timer(rospy.Duration(1,0), self._send_estimated_state)
        self._waypoint_serv = rospy.Service("load_waypoints", InitWaypointSet, self._send_goal)
        self._action_client = actionlib.SimpleActionClient("follow_waypoints", FollowWaypointsAction)
        self._mode_client = rospy.ServiceProxy("controller/set_control_mode", SetControlMode)
        self._plan_db_pub = rospy.Publisher("imc/plan_db", PlanDB, queue_size=10)
        self._marker_pub = rospy.Publisher("current_waypoints", WaypointSet, queue_size=1)

    def _send_goal(self, req):
        res = InitWaypointSetResponse()
        res.success = False
        try:
            self._mode_client.wait_for_service(rospy.Duration(5))
            mode_req = SetControlModeRequest()
            mode_req.mode.mode = mode_req.mode.LOSGUIDANCE
            self._mode_client(mode_req)
            self._action_client.wait_for_server(rospy.Duration(5))
            goal = FollowWaypointsGoal()
            goal.waypoints.header = Header(0, rospy.Time.now(), "utm")
            goal.waypoints.start_time = Time(goal.waypoints.header.stamp)
            goal.waypoints.waypoints = req.waypoints
            self._marker_pub.publish(goal.waypoints)
            self._action_client.send_goal(goal, self._handle_done, self._handle_active, self._handle_feedback)
            res.success = True
            return res
        except Exception as e:
            rospy.logerr("{} | {}".format(rospy.get_name(),e.message))
            return res

    def _handle_done(self, status, result):
        rospy.loginfo("{} | Action completed, status: {}\n Waypoints completed: {}".format(rospy.get_name(), status, result))

    def _handle_active(self):
        rospy.loginfo("{} | Waypoints set, Front Seat Driver active.".format(rospy.get_name()))

    def _handle_feedback(self, feedback):
        rospy.loginfo_throttle(10, "{} | {}".format(rospy.get_name(), feedback))

    def _handle_goto(self, msg):
        """Parse if a goto message is received"""
        # TODO Not implemented in imc_ros_bridge yet
        rospy.loginfo(msg)
        pass

    def _parse_plan(self):
        # TODO this parser only accepts a set of goto commands
        """A plan has been requested by neptus to start. Parse the plan here and send to controller."""
        geopoints = []
        speeds = []
        is_depths = []
        for maneuver in self._current_plan.plan_spec.maneuvers:
            geopoints.append( GeoPoint(maneuver.maneuver.lat*180.0/np.pi,maneuver.maneuver.lon*180.0/np.pi,maneuver.maneuver.z))
            speeds.append(maneuver.maneuver.speed)
            is_depths.append(maneuver.maneuver.z_units == 1)
        req = ConvertGeoPointsRequest()
        req.geopoints = geopoints
        points = self._geo_converter.call(req).utmpoints
        wps = []
        # If datum is available, then reference frame is ENU World
        for point, speed, is_depth in zip(points, speeds, is_depths):
            wp = Waypoint()
            wp.point.x = point.x
            wp.point.y = point.y
            wp.point.z = -abs(point.z) if is_depth else abs(point.z)
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = "utm"
            wp.radius_of_acceptance = 1.0
            wp.max_forward_speed = speed
            wp.use_fixed_heading = False
            wp.master_heading_offset = 0.0
            wps.append(wp)
        req = InitWaypointSetRequest()
        req.start_time = Time(rospy.Time.from_sec(0))
        req.start_now = True
        req.waypoints = wps
        req.max_forward_speed = max(speeds)
        req.interpolator.data = "lipb"
        req.master_heading_offset = 0
        self._send_goal(req)

    def _handle_plan_control(self, msg):
        """Parse requests to start or stop a plan."""
        typee = msg.type
        op = msg.op
        # request to start a mission!
        if typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_SET:
            self._current_plan = self._plan_db[msg.plan_id]
            self._parse_plan()
            req = SetControlModeRequest()
            req.mode.mode = req.mode.LOSGUIDANCE
            self._mode_client(req)
        # request to stop mission
        elif typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_DEL:
            self._current_plan = PlanDB()
            req = SetControlModeRequest()
            req.mode.mode = req.mode.HOLDPOSITION
            self._mode_client(req)

    def _handle_plan_db(self, msg):
        """Parse requests for Plan Database messages"""
        typee = msg.type
        op = msg.op
        # request for plan info
        if typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_INFO:
            # we need to respond to this with some info... but what?
            rospy.loginfo_throttle_identical(30, "Got REQUEST GET_INFO planDB msg from Neptus for plan: {}".format(str(msg.plan_id)))
            response = PlanDB()
            response.plan_id = self._plan_db[msg.plan_id].plan_id
            response.type = imc_enums.PLANDB_TYPE_SUCCESS
            response.op = imc_enums.PLANDB_OP_GET_INFO
            response.plandb_information = self._plan_db[msg.plan_id].plandb_information
            self._plan_db_pub.publish(response)
            rospy.loginfo_throttle_identical(30, "Answered GET_INFO for plan:" + str(response.plan_id))
        # request for plan state
        elif typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(30, "Got REQUEST GET_STATE planDB msg from Neptus")
            response = PlanDB()
            response.plan_id = self._current_plan.plan_id
            response.type = imc_enums.PLANDB_TYPE_SUCCESS
            response.op = imc_enums.PLANDB_OP_GET_STATE
            response.plandb_state = PlanDBState()
            response.plandb_state.plan_count = len(self._plan_db)
            response.plandb_state.plans_info = []
            totalSize = 0
            latest = None
            # TODO Neptus needs an md5 calculation on the plans_info message to be stored in plandb_state.md5 in order
            #  to sync, but we cannot seem to get that right for now.
            #  https://github.com/LSTS/imcjava/blob/d95fddeab4c439e603cf5e30a32979ad7ace5fbc/src/java/pt/lsts/imc/adapter/PlanDbManager.java#L160
            #  See above for an example
            #  It seems like we need to keep a planDB ourselves on this side, collect all the plans we
            #  received and answer this get_state with data from them all to get a properly synchronized plan.
            buffer = StringIO()  # This buffer method for calculating md5 sum of a ros message is not ideal.
            md = hashlib.md5()
            for p in self._plan_db.values():
                pdi = p.plandb_information
                totalSize += pdi.plan_size
                response.plandb_state.plans_info.append(pdi)
                latest = latest if latest is None else max(latest, pdi.change_time)
                pdi.serialize(buffer)
                [md.update(i) for i in buffer.buflist]
            response.plandb_state.plan_count = len(response.plandb_state.plans_info)
            response.plandb_state.plan_size = totalSize
            response.plandb_state.md5 = md.digest()
            self._plan_db_pub.publish(response)
            rospy.loginfo_throttle_identical(30, "Answered GET_STATE for plans.")
        # ack for plan set success
        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_SET:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb set")
        # ack for plan get info
        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_INFO:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get info")
        # ack for plan get state
        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get state")
        # request to set plan
        elif op == imc_enums.PLANDB_OP_SET:
            # update the plan_info and plan_state attributes
            msg.plandb_information.plan_id = msg.plan_id
            msg.plandb_information.plan_size = len(msg.plan_spec.maneuvers)
            msg.plandb_information.change_time = time.time()
            msg.plandb_information.md5 = msg.plan_spec_md5
            self._plan_db.update({msg.plan_spec.plan_id: msg}) # place the plan in a dictionary of plans
            # send a success response
            response = PlanDB()
            response.type = imc_enums.PLANDB_TYPE_SUCCESS
            response.op = imc_enums.PLANDB_OP_SET
            response.plan_id = msg.plan_spec.plan_id
            self._plan_db_pub.publish(response)
        # handle other requests
        else:
            rospy.loginfo_throttle_identical(5, "Received some unhandled planDB message:\n" + str(msg))

    def _handle_abort(self, msg):
        """Listen for the abort message, do something if it comes :)"""
        self._action_client.wait_for_server()
        if self._action_client.get_state() == GoalStatus.ACTIVE:
            self._action_client.cancel_goal()
        self._mode_client.wait_for_service()
        req = SetControlModeRequest()
        req.mode.mode = req.mode.ABORT
        self._mode_client(req)

    def _handle_imc_heartbeat(self, msg):
        """Here in case you want to do something on the heartbeat of the IMC"""
        pass

    def _handle_gps(self, msg):
        # neptus expects latitude and longitude to be in radians
        self._estimated_state_msg.lat = msg.latitude*np.pi/180.0
        self._estimated_state_msg.lon = msg.longitude*np.pi/180.0
        self._estimated_state_msg.alt = msg.altitude

    def _handle_pose(self, msg):
        if self._tf_buffer.can_transform("bluerov2/base_link", "map_ned", rospy.Time.now(), rospy.Duration.from_sec(0.5)):
            tf = self._tf_buffer.lookup_transform("bluerov2/base_link", "map_ned", rospy.Time.now(), rospy.Duration.from_sec(0.5))
        else:
            return
        # TODO neptus gets confused if you try to give all available estimates, there is probably a transformation problem
        # self._estimated_state_msg.x = tf.transform.translation.x
        # self._estimated_state_msg.y = tf.transform.translation.y
        # self._estimated_state_msg.z = tf.transform.translation.z
        # self._estimated_state_msg.depth = tf.transform.translation.z
        # self._estimated_state_msg.height = msg.pose.pose.position.z
        R = Rotation([tf.transform.rotation.x,
                      tf.transform.rotation.y,
                      tf.transform.rotation.z,
                      tf.transform.rotation.w])
        self._estimated_state_msg.phi, self._estimated_state_msg.theta, self._estimated_state_msg.psi = R.as_euler(
            "xyz", False).squeeze()
        # self._estimated_state_msg.u = msg.twist.twist.linear.x
        # self._estimated_state_msg.v = -msg.twist.twist.linear.y
        # self._estimated_state_msg.w = -msg.twist.twist.linear.z
        # self._estimated_state_msg.p = msg.twist.twist.angular.x
        # self._estimated_state_msg.q = -msg.twist.twist.angular.y
        # self._estimated_state_msg.r = -msg.twist.twist.angular.z
        # u = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z])
        # self._estimated_state_msg.vx, self._estimated_state_msg.vy, self._estimated_state_msg.vz = R.apply(u, True)

    def _send_estimated_state(self, event):
        self._estimated_state_pub.publish(self._estimated_state_msg)


if __name__=="__main__":
    rospy.init_node("executive")
    handler = ImcInterface()
    rospy.spin()
