#!/usr/bin/env python


import rospy
from bluerov2_msgs.srv import ConvertGeoPoints, ConvertGeoPointsRequest, SetControllerState, SetControllerStateRequest
from bluerov2_msgs.msg import FollowWaypointsGoal, FollowWaypointsResult, FollowWaypointsAction, FollowWaypointsFeedback
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Empty, Header
from imc_ros_bridge.msg import PlanDB, EstimatedState, PlanControlState, PlanControl, PlanDBInformation, PlanDBState, PlanManeuver
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode
from mavros_msgs.srv import *
import numpy as np
from bluerov2_executive import imc
import hashlib
from io import StringIO
import time
import actionlib
from actionlib_msgs.msg import GoalStatus
import math


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
        self._map_frame_id = rospy.get_param("map_frame_id", "map")
        self._map_ned_frame_id = rospy.get_param("map_ned_frame_id", self._map_frame_id + "_ned")
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer)
        self._name = rospy.get_name()

        self._waypoint_serv = rospy.Service("~waypoints/load", WaypointPush, self._send_goal)
        self._action_client = actionlib.SimpleActionClient("actions/follow_waypoints", FollowWaypointsAction)
        self._controller_state_client = rospy.ServiceProxy("controller/set_controller_state", SetControllerState)
        self._plan_db_pub = rospy.Publisher("imc/plan_db", PlanDB, queue_size=10)
        self._marker_pub = rospy.Publisher("waypoints/current", WaypointList, queue_size=1)

        rospy.Subscriber("gps", NavSatFix, self._handle_gps)
        rospy.Subscriber("odometry", Odometry, self._handle_odom)
        rospy.Subscriber("imc/goto_waypoint", Pose, self._handle_goto)
        rospy.Subscriber("imc/plan_control", PlanControl, self._handle_plan_control)
        rospy.Subscriber("imc/abort", Empty, self._handle_abort)
        rospy.Subscriber("imc/imc_heartbeat", Empty, self._handle_imc_heartbeat)
        rospy.Subscriber("imc/plan_db", PlanDB, self._handle_plan_db)
        rospy.Timer(rospy.Duration(1, 0), self._send_estimated_state)

    def _send_goal(self, req: WaypointPushRequest):
        res = WaypointPushResponse()
        res.success = False
        try:
            self._controller_state_client.wait_for_service(rospy.Duration(5))
            state_req = SetControllerStateRequest()
            state_req.state.state = state_req.state.LOSGUIDANCE
            self._controller_state_client(state_req)

            self._action_client.wait_for_server(rospy.Duration(5))
            goal = FollowWaypointsGoal()

            goal.header = Header(0, rospy.Time.now(), "")
            goal.waypoints = req.waypoints

            self._marker_pub.publish(goal.waypoints)
            self._action_client.send_goal(goal, self._handle_done, self._handle_active, self._handle_feedback)
            res.success = True
            res.wp_transfered = len(goal.waypoints.waypoints)
            return res
        except Exception as e:
            rospy.logerr(f"{self._name} | {e}")
            return res

    def _handle_done(self, status, result):
        rospy.loginfo(f"{self._name} | Action completed, status: {status}\n Waypoints completed: {result}")

    def _handle_active(self):
        rospy.loginfo(f"{self._name} | Waypoints set, Front Seat Driver active.")

    def _handle_feedback(self, feedback):
        rospy.loginfo_throttle(10, f"{self._name} | {feedback}")

    def _handle_goto(self, msg):
        """Parse if a goto message is received"""
        # TODO Not implemented in imc_ros_bridge yet
        rospy.logerr(NotImplementedError("No GoTo Support yet."))

    def _parse_plan(self):
        # TODO this parser only accepts a set of goto commands in a plan db
        """A plan has been requested by neptus to start. Parse the plan here and send to controller."""
        wps = WaypointList()
        for i, maneuver in enumerate(self._current_plan.plan_spec.maneuvers):
        # for geopoint, z_unit in zip(geopoints z_units):
            wp = Waypoint()
            wp.x_lat = maneuver.maneuver.lat * 180.0 / math.pi
            wp.y_long = maneuver.maneuver.lon * 180.0 / math.pi
            wp.z_alt = maneuver.maneuver.z
            if maneuver.maneuver.z_units == imc.ZUnits.Z_NONE or maneuver.maneuver.z_units == imc.ZUnits.Z_HEIGHT:
                wp.frame = wp.FRAME_GLOBAL
            elif maneuver.maneuver.z_units == imc.ZUnits.Z_DEPTH:
                wp.frame = wp.FRAME_GLOBAL_REL_ALT
            elif maneuver.maneuver.z_units == imc.ZUnits.Z_ALTITUDE:
                wp.frame = wp.FRAME_GLOBAL_TERRAIN_ALT
            wp.autocontinue = True
            wp.command = CommandCode.NAV_WAYPOINT
            wp.param1 = 2.0
            wp.param2 = maneuver.maneuver.speed
            wp.is_current = i == 0
            wps.waypoints.append(wp)
        wps.current_seq = 0
        req = WaypointPushRequest()
        req.waypoints = wps
        req.start_index = 0
        self._send_goal(req)

    def _handle_plan_control(self, msg):
        """Parse requests to start or stop a plan."""
        typee = msg.type
        op = msg.op
        # request to start a mission!
        if typee == imc.PlanDBType.PLANDB_TYPE_REQUEST and op == imc.PlanDBOp.PLANDB_OP_SET:
            self._current_plan = self._plan_db[msg.plan_id]
            self._parse_plan()
            req = SetControllerStateRequest()
            req.state.state = req.state.LOSGUIDANCE
            self._controller_state_client(req)
        # request to stop mission
        elif typee == imc.PlanDBType.PLANDB_TYPE_REQUEST and op == imc.PlanDBOp.PLANDB_OP_DEL:
            self._current_plan = PlanDB()
            req = SetControllerStateRequest()
            req.state.state = req.state.IDLE
            self._controller_state_client(req)

    def _handle_plan_db(self, msg):
        """Parse requests for Plan Database messages"""
        typee = msg.type
        op = msg.op
        # request for plan info
        if typee == imc.PlanDBType.PLANDB_TYPE_REQUEST and op == imc.PlanDBOp.PLANDB_OP_GET_INFO:
            # we need to respond to this with some info... but what?
            rospy.loginfo_throttle_identical(30, "Got REQUEST GET_INFO planDB msg from Neptus for plan: {}".format(str(msg.plan_id)))
            response = PlanDB()
            response.plan_id = self._plan_db[msg.plan_id].plan_id
            response.type = imc.PlanDBType.PLANDB_TYPE_SUCCESS.value
            response.op = imc.PlanDBOp.PLANDB_OP_GET_INFO.value
            response.plandb_information = self._plan_db[msg.plan_id].plandb_information
            self._plan_db_pub.publish(response)
            rospy.loginfo_throttle_identical(30, "Answered GET_INFO for plan:" + str(response.plan_id))
        # request for plan state
        elif typee == imc.PlanDBType.PLANDB_TYPE_REQUEST and op == imc.PlanDBOp.PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(30, "Got REQUEST GET_STATE planDB msg from Neptus")
            response = PlanDB()
            response.plan_id = self._current_plan.plan_id
            response.type = imc.PlanDBType.PLANDB_TYPE_SUCCESS.value
            response.op = imc.PlanDBOp.PLANDB_OP_GET_STATE.value
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
                #pdi.serialize(buffer)
                #[md.update(i) for i in buffer]
            response.plandb_state.plan_count = len(response.plandb_state.plans_info)
            #response.plandb_state.plan_size = totalSize
            #response.plandb_state.md5 = md.digest()
            self._plan_db_pub.publish(response)
            rospy.loginfo_throttle_identical(30, "Answered GET_STATE for plans.")
        # ack for plan set success
        elif typee == imc.PlanDBType.PLANDB_TYPE_SUCCESS and op == imc.PlanDBOp.PLANDB_OP_SET:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb set")
        # ack for plan get info
        elif typee == imc.PlanDBType.PLANDB_TYPE_SUCCESS and op == imc.PlanDBOp.PLANDB_OP_GET_INFO:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get info")
        # ack for plan get state
        elif typee == imc.PlanDBType.PLANDB_TYPE_SUCCESS and op == imc.PlanDBOp.PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get state")
        # request to set plan
        elif op == imc.PlanDBOp.PLANDB_OP_SET:
            # update the plan_info and plan_state attributes
            msg.plandb_information.plan_id = msg.plan_id
            msg.plandb_information.plan_size = len(msg.plan_spec.maneuvers)
            msg.plandb_information.change_time = time.time()
            msg.plandb_information.md5 = msg.plan_spec_md5
            self._plan_db.update({msg.plan_spec.plan_id: msg}) # place the plan in a dictionary of plans
            # send a success response
            response = PlanDB()
            response.type = imc.PlanDBType.PLANDB_TYPE_SUCCESS.value
            response.op = imc.PlanDBOp.PLANDB_OP_SET.value
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
        self._controller_state_client.wait_for_service()
        req = SetControllerStateRequest()
        req.state.state = req.state.ABORT
        self._controller_state_client(req)

    def _handle_imc_heartbeat(self, msg):
        """Here in case you want to do something on the heartbeat of the IMC"""
        pass

    def _handle_gps(self, msg: NavSatFix):
        # neptus expects latitude and longitude to be in radians
        self._estimated_state_msg.lat = msg.latitude*np.pi/180.0
        self._estimated_state_msg.lon = msg.longitude*np.pi/180.0
        self._estimated_state_msg.alt = msg.altitude

    def _handle_odom(self, msg: Odometry):
        p = PoseStamped(msg.header, msg.pose.pose.position, msg.pose.pose.orientation)
        if self._map_ned_frame_id != msg.header.frame_id:
            if self._tf_buffer.can_transform(msg.header, self._map_ned_frame_id, rospy.Time.from_sec(0)):
                p = self._tf_buffer.transform(p, self._map_ned_frame_id)
            else:
                rospy.logerr(f"{self._name} | No TF between {msg.header} and {self._map_ned_frame_id}")
                return
        # TODO neptus gets confused if you try to give all available estimates, there is probably a transformation problem
        # self._estimated_state_msg.x = tf.transform.translation.x
        # self._estimated_state_msg.y = tf.transform.translation.y
        # self._estimated_state_msg.z = tf.transform.translation.z
        # self._estimated_state_msg.depth = tf.transform.translation.z
        # self._estimated_state_msg.height = msg.pose.pose.position.z
        R = Rotation([p.pose.orientation.x,
                      p.pose.orientation.y,
                      p.pose.orientation.z,
                      p.pose.orientation.w])
        self._estimated_state_msg.phi, self._estimated_state_msg.theta, self._estimated_state_msg.psi = R.as_euler("xyz", False).squeeze()
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

