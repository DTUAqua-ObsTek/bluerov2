import rospy
from bluerov2_msgs.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsResult, FollowWaypointsFeedback, ControllerState, Autopilot
from bluerov2_msgs.srv import SetControllerState, SetControllerStateRequest, SetControllerStateResponse
import actionlib
import numpy as np
from sensor_msgs.msg import NavSatFix, Range
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import QuaternionStamped, Quaternion, Point, PoseStamped, TwistStamped, Pose
from scipy.spatial.transform import Rotation
from bluerov2_navigation.helpers import geodetic, geoid, math
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached
import tf2_ros, tf2_geometry_msgs


def euler_to_quaternion(x: float, y: float, z: float, degrees=True):
    """

    Args:
        x: rotation around x
        y: rotation around y
        z: rotation around z
        degrees: flag to indicate degrees

    Returns:
        geometry_msgs.msg/Quaternion

    """
    return Quaternion(* Rotation.from_euler("xyz", [x, y, z], degrees=degrees).as_quat())


def quaternion_to_euler(q: Quaternion, degrees=True):
    """

    Args:
        q: geometry_msgs.msg/Quaternion Message
        degrees: indicates if output euler should be in degrees

    Returns: Euler tuple in 'xyz' order.

    """
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=degrees)


def tf2_quaternion_rotate(buffer: tf2_ros.Buffer, q: QuaternionStamped, target_frame: str):
    if buffer.can_transform(target_frame, q.header.frame_id, rospy.Time.from_sec(0)):
        tf = buffer.lookup_transform(target_frame, q.header.frame_id, rospy.Time.from_sec(0))
        R1 = Rotation.from_quat([q.quaternion.x, q.quaternion.y, q.quaternion.z, q.quaternion.w]).as_matrix()
        R2 = Rotation.from_quat([tf.transform.rotation.x, tf.transform.rotation.y,
                                 tf.transform.rotation.z, tf.transform.rotation.w]).as_matrix()
        R3 = R2 @ R1
        q_out = Rotation.from_matrix(R3).as_quat()
        return QuaternionStamped(Header(0, q.header.stamp, target_frame), Quaternion(*q_out))


class LineOfSight:
    """ Provide LOS guidance for autopilot controller. """

    def __init__(self):
        # ---------- Waypoint Following Config ----------
        self._los_server = actionlib.SimpleActionServer('actions/follow_waypoints', FollowWaypointsAction, self._los_action_cb,
                                                        False)
        self._los_server.start()
        self._latest_wp = None
        self._controller_state = ControllerState(ControllerState.OFF)
        self._current_gps = None
        self._msl = None
        self._bottom_height = None
        self._ned_heading = None
        self._enu_heading = None
        self._map_frame_id = rospy.get_param("map_frame_id", "map")
        self._map_ned_frame_id = rospy.get_param("map_ned_frame_id", self._map_frame_id + "_ned")
        self._autopilot_msg = Autopilot()
        self._speed_gain = rospy.get_param("~speed_gain", 1.0)
        self._geoid = geoid.GeoidHeight(rospy.get_param("geoid_path", "/usr/share/GeographicLib/geoids/egm96-5.pgm"))

        # ---------- Publishers ----------
        self._set_autopilot_pub = rospy.Publisher("guidance/autopilot", Autopilot, queue_size=5)  # Publish the autopilot settings
        self._msl_pub = rospy.Publisher("guidance/msl", Float64, queue_size=10)  # Convert GPS altitude to MSL
        self._sog_pub = rospy.Publisher("guidance/sog", Float64, queue_size=10)  # Extract SoG from gps velocity.
        self._cog_pub = rospy.Publisher("guidance/cog", Float64, queue_size=10)  # Extract CoG from gps velocity.

        # -------- TF Buffer ----------
        self._tf_buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buff)

        # -------- Services ----------
        self._set_controller_state = rospy.ServiceProxy("controller/set_controller_state", SetControllerState)

        # ---------- Subscribers ----------
        rospy.Subscriber("controller/state", ControllerState,
                         self._controller_state_cb)  # Subscribe to the controller's state
        rospy.Subscriber("mavros/global_position/global", NavSatFix,
                         self._global_pos_cb)  # Subscribe to the vehicle's GPS estimate
        rospy.Subscriber("mavros/global_position/compass_hdg", Float64,
                         self._heading_cb)  # Subscribe to the vehicle's Heading estimate
        rospy.Subscriber('mavros/global_position/raw/gps_vel', TwistStamped,
                         self._global_vel_cb)  # Subscribe to the vehicle's easting/northing speed
        rospy.Subscriber('mavros/distance_sensor/range_pub', Range,
                         self._altitude_cb)  # Subscribe to the altitude estimation

    def _global_vel_cb(self, msg: TwistStamped):
        sog = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2)
        cog = 90 - math.rad2deg(math.atan2(msg.twist.linear.y, msg.twist.linear.x))
        cog = 360 + cog if cog < 0 else cog
        self._sog_pub.publish(Float64(sog))
        self._cog_pub.publish(Float64(cog))

    def _altitude_cb(self, msg: Range):
        self._bottom_height = Range.range

    def _controller_state_cb(self, msg: ControllerState):
        self._controller_state = msg

    def _global_pos_cb(self, msg: NavSatFix):
        geoid_height_above_ellipsoid = self._geoid.get(msg.latitude, msg.longitude)
        self._msl = msg.altitude - geoid_height_above_ellipsoid
        self._msl_pub.publish(Float64(msg.altitude - geoid_height_above_ellipsoid))
        self._current_gps = msg

    def _heading_cb(self, msg: Float64):
        self._ned_heading = QuaternionStamped(Header(0, rospy.Time.now(), self._map_ned_frame_id),
                                              euler_to_quaternion(0, 0, msg.data))
        self._enu_heading = tf2_quaternion_rotate(self._tf_buff, self._ned_heading, self._map_frame_id)
        #self._enu_heading = self._tf_buff.transform(self._ned_heading, self._map_frame_id, 0)

    # -------- ACTION CALLBACKS --------
    def _los_action_cb(self, goal: FollowWaypointsGoal):
        """ Given a goal containg waypoints, set the vehicle up for tracking them. """
        r = rospy.Rate(1.0)
        results = FollowWaypointsResult()
        feedback = FollowWaypointsFeedback()
        autopilot_msg = Autopilot()
        autopilot_msg.header.frame_id = self._map_ned_frame_id

        # Wait for mode to be set to LOSGUIDANCE
        while self._controller_state.state != ControllerState.LOSGUIDANCE:
            # rospy.logwarn_throttle(10.0, "{} | Waypoints pending. Call controller/set_controller_state service with LOSGUIDANCE state to execute.".format(
            #     rospy.get_name()))
            res = SetControllerStateResponse()
            while not res.success:
                self._set_controller_state.wait_for_service()
                req = SetControllerStateRequest()
                req.state = ControllerState.LOSGUIDANCE
                res = self._set_controller_state.call(req)
            if self._los_server.is_preempt_requested():
                results.waypoints_completed = 0
                rospy.loginfo(f"{rospy.get_name()} | Follow waypoints action halted.")
                self._los_server.set_preempted(results)
                return
            r.sleep()
        rospy.loginfo(f"{rospy.get_name()} | Adding {len(goal.waypoints.waypoints)} waypoint commands to Autopilot.")
        # Iterate through each waypoint, and send Autopilot commands to controller
        r = rospy.Rate(10.0)
        for i, wp in enumerate(goal.waypoints.waypoints):
            # Set the latest waypoint
            self._latest_wp = wp
            # While the distance to the waypoint is not within the spherical radius of acceptance
            self._calculate_los()
            while not self._distance_ref < wp.param1:
                self._calculate_los()
                self._autopilot_msg.header.stamp = rospy.Time.now()
                self._set_autopilot_pub.publish(self._autopilot_msg)
                feedback.percentage_complete = float(i) / float(len(goal.waypoints.waypoints)) * 100.0
                self._los_server.publish_feedback(feedback)
                if self._los_server.is_preempt_requested():
                    rospy.loginfo(f"{rospy.get_name()} | Follow waypoints action halted.")
                    self._los_server.set_preempted(results)
                    return
                if self._controller_state.state != ControllerState.LOSGUIDANCE:
                    rospy.loginfo(f"{rospy.get_name()} | Follow waypoints action aborted.")
                    self._los_server.set_aborted(results)
                    return
                r.sleep()
            results.waypoints_completed = i + 1
        rospy.loginfo("{} | Waypoint tasks completed.".format(rospy.get_name()))
        self._set_controller_state.wait_for_service()
        req = SetControllerStateRequest()
        req.state = ControllerState.IDLE
        self._set_controller_state.call(req)
        self._los_server.set_succeeded(results)

    def _is_gps_valid(self):
        if self._current_gps is None:
            rospy.logwarn_throttle(10., f"{rospy.get_name()} | No GPS received.")
            return False
        if (rospy.Time.now() - self._current_gps.header.stamp).to_sec() > 5.0:
            rospy.logwarn_throttle(10., f"{rospy.get_name()} | No recent GPS received.")
            return False
        return True

    def _is_wp_valid(self):
        if self._latest_wp is None:
            rospy.logwarn_throttle(10.0, f"{rospy.get_name()} | No waypoint provided.")
            return False
        return True

    def _is_heading_valid(self):
        if self._ned_heading is None:
            rospy.logwarn_throttle(10.0, f"{rospy.get_name()} | No heading estimate provided.")
            return False
        if (rospy.Time.now() - self._ned_heading.header.stamp).to_sec() > 5.0:
            rospy.logwarn_throttle(10., f"{rospy.get_name()} | No recent heading received.")
            return False
        return True

    def _is_range_valid(self):
        if self._bottom_height is None:
            rospy.logwarn_throttle(10.0, f"{rospy.get_name()} | No ranging provided.")
            return False
        if (rospy.Time.now() - self._bottom_height.header.stamp).to_sec() > 5.0:
            rospy.logwarn_throttle(10., f"{rospy.get_name()} | No recent ranging received.")
            return False
        return True

    def _calculate_depth(self):
        geoid_height = self._geoid.get(self._current_gps.latitude, self._current_gps.longitude)
        return self._current_gps.altitude - geoid_height

    def _calculate_los(self):
        if self._is_gps_valid() and self._is_wp_valid() and self._is_heading_valid():
            # Get the x and y coordinates
            pose_enu = self._global_enu_reference()
            # Convert to NED
            los_reference_ned = self._tf_buff.transform(pose_enu, self._map_ned_frame_id)
            # Calculate the required heading
            self._heading_ref = quaternion_to_euler(los_reference_ned.pose.orientation)[-1]
            self._heading_ref = 360 + self._heading_ref if self._heading_ref < 0 else self._heading_ref
            # If the waypoint is constant depth
            if self._latest_wp.frame == Waypoint.FRAME_GLOBAL_REL_ALT:
                self._autopilot_msg.height_reference = self._autopilot_msg.DPT
                # Calculate the 3d distance
                self._distance_ref = math.sqrt(
                    los_reference_ned.pose.position.x ** 2 + los_reference_ned.pose.position.y ** 2 + (self._msl - self._latest_wp.z_alt) ** 2)
            # If the waypoint is Global Position MSL
            elif self._latest_wp.frame == Waypoint.FRAME_GLOBAL:
                self._autopilot_msg.height_reference = self._autopilot_msg.MSL
                # Calculate the 3d distance
                self._distance_ref = math.sqrt(los_reference_ned.pose.position.x ** 2 + los_reference_ned.pose.position.y ** 2 + los_reference_ned.pose.position.z ** 2)
            elif self._latest_wp.frame == Waypoint.FRAME_GLOBAL_TERRAIN_ALT:
                if self._is_range_valid():
                    self._autopilot_msg.height_reference = self._autopilot_msg.BTM
                    self._distance_ref = math.sqrt(
                        los_reference_ned.pose.position.x ** 2 + los_reference_ned.pose.position.y ** 2 + (self._bottom_height - self._latest_wp.z_alt) ** 2)
            self._autopilot_msg.heading = self._heading_ref
            self._autopilot_msg.Z = self._latest_wp.z_alt
            rospy.loginfo(self._distance_ref)
            self._autopilot_msg.U = min(self._latest_wp.param2, self._speed_gain * self._distance_ref)
        else:
            rospy.logerr_throttle(5.0, "{rospy.get_name(} | GPS, waypoint, hading not all valid")

    def _global_enu_reference(self):
        """ Calculate local ENU transformation. """
        # Get the waypoint location referenced to the vehicle's position
        if self._is_gps_valid() and self._is_wp_valid() and self._is_heading_valid():
            # Get the x, y, z offsets
            x, y, z = geodetic.geodetic2enu(self._latest_wp.x_lat, self._latest_wp.y_long, self._latest_wp.z_alt,
                                            self._current_gps.latitude, self._current_gps.longitude, self._current_gps.altitude)
            # Get the rotation of vector between position and destination, point the vehicle this way
            los_reference_enu = PoseStamped(Header(0, rospy.Time.now(), self._map_frame_id),
                                            Pose(Point(x, y, z),
                                                 euler_to_quaternion(0, 0, math.atan2(y, x), degrees=False))
                                           )
            return los_reference_enu
