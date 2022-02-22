from bluerov2_msgs.msg import ControllerState
import tf2_ros
import tf2_geometry_msgs
import rospy
from bluerov2_control.controllers.pid import MIMOPID
from bluerov2_msgs.srv import SetControllerState, SetControllerStateResponse, SetControllerStateRequest, SetAutopilotRequest, SetAutopilot, SetAutopilotResponse
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelStamped, AccelWithCovarianceStamped, PoseStamped, Twist, TwistStamped, Vector3Stamped, WrenchStamped, QuaternionStamped
import numpy as np
from bluerov2_msgs.msg import Autopilot
from sensor_msgs.msg import Range
from dynamic_reconfigure.server import Server
from bluerov2_msgs.cfg import pid4dofConfig
from std_msgs.msg import Header, Float64
from scipy.spatial.transform import Rotation
from bluerov2_guidance import euler_to_quaternion, quaternion_to_euler, tf2_quaternion_rotate


class Cascade4DoF:
    """

    A 4 DoF (Surge, Sway, Heave, Yaw)
    Cascade PID Controller that Can be Configured for Different Modes of Control
    Modes:
    AccelTeleop -> Acceleration Setpoint Given by AccelStamped Message      [X]
    VelTeleop -> Velocity Setpoint Given by Vector3Stamped Message          [X]
    Autopilot -> Velocity Setpoint Set to Requested Vector3Stamped Message  [X]
    HoldPosition -> Position Setpoint Set to Latest Odometry Message        [X]
    Abort -> Tells Vehicle to Surface                                       [X]
    Outputs WrenchStamped Message with Linear Force X, Y, Z, and Torque Z Populated

    """

    def __init__(self):
        """
        Initialize tf2 buffer, subscribers, publishers, PID parameters, dynamic reconfigure server
        """
        self._ready = False
        self._first = True
        self._controller_state = ControllerState()
        self._controller_state.state = self._controller_state.OFF
        self._name = rospy.get_name()

        # -------- Params -----------
        self._map_frame_id = rospy.get_param("map_frame_id", default="map")
        self._map_ned_frame_id = rospy.get_param("map_ned_frame_id", default = self._map_frame_id + "_ned")
        self._use_accel_fb = rospy.get_param("~use_accel_fb", default=False)
        self._base_frame_id = rospy.get_param("base_frame_id", default="base_link")

        # -------- TF Buffer ----------
        self._tf_buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buff)

        # -------- Service Advertisements ---------
        self._set_controller_state = rospy.Service("controller/set_controller_state", SetControllerState, self._handle_controller_state)
        self._configure_mavros = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # -------- State Config -------------------
        self._controller_state_pub = rospy.Publisher("controller/state", ControllerState, queue_size=5)
        self._latest_odom_fb = None
        rospy.Subscriber("odometry/filtered", Odometry, self._odom_feedback)

        # -------- Output Wrench Config ------------------
        self._wrench = WrenchStamped()
        self._wrench_pub = rospy.Publisher("wrench/target", WrenchStamped, queue_size=5)

        # -------- Acceleration Config ------------
        self._latest_accel_sp = None
        rospy.Subscriber("accel/setpoint", AccelStamped, self._accel_sp)
        self._latest_accel_fb = None
        rospy.Subscriber("accel/filtered", AccelWithCovarianceStamped, self._accel_feedback)
        # If using acceleration feedback, then setup the PID Gains and subscribe to the acceleration feedback
        if self._use_accel_fb:
            # Can only use ax, ay, az because ar is not provided by robot_localization
            self._accel_pids = MIMOPID([0, 0, 0, 0],
                                       [0, 0, 0, 0],
                                       [0, 0, 0, 0],
                                       [0, 0, 0, 0])
        # Otherwise, mass and moments of inertia must be given
        else:
            self.mass = rospy.get_param("~pid/mass")
            self.inertial = rospy.get_param("~pid/inertial")
            # update mass, moments of inertia
            self.inertial_tensor = np.array(
                [[self.inertial['ixx'], self.inertial['ixy'], self.inertial['ixz']],
                 [self.inertial['ixy'], self.inertial['iyy'], self.inertial['iyz']],
                 [self.inertial['ixz'], self.inertial['iyz'], self.inertial['izz']]])
            self.mass_inertial_matrix = np.vstack((
                np.hstack((self.mass * np.identity(3), np.zeros((3, 3)))),
                np.hstack((np.zeros((3, 3)), self.inertial_tensor))))

        # --------- Velocity Config ----------
        self._vel_pids = MIMOPID([0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0])
        rospy.Subscriber("vel/setpoint", TwistStamped, self._vel_sp)
        self._vel_limits = np.zeros((4, 1), dtype=float)
        self._latest_vel_sp = None
        self._vel_deadband = rospy.get_param("~pid/deadband", 0.01)

        # ---------- Position Config ---------
        self._pos_pids = MIMOPID([0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0])
        rospy.Subscriber("pos/setpoint", PoseStamped, self._pos_sp)
        self._latest_pos_sp = None
        rospy.Subscriber("depth/filtered", Float64, self._depth_feedback)
        self._latest_depth = None

        # ---------- Autopilot Config ----------
        rospy.Subscriber("autopilot/setpoint", Autopilot, self._ap_sp)
        rospy.Service("autopilot/set", SetAutopilot, self._autopilot_service)
        self._latest_ap_sp = None
        rospy.Subscriber("range", Range, self._range_feedback)
        self._latest_range_fb = None
        self._control_depth = True
        self._control_sway = True
        self._control_yaw = True

        # -------- Dynamic Reconfigure Server -------
        self._reconfig_serv = Server(pid4dofConfig, self._reconfig)

        # ---------- Timer Loops -----------
        rospy.Timer(rospy.Duration.from_sec(1.0 / 10.0), self._main_loop)  # TODO expose rate parameter
        self._ready = True

    def _reconfig(self, config, level):
        if self._use_accel_fb:
            self._accel_pids.set_p([config["accel_x_kp"],
                                    config["accel_y_kp"],
                                    config["accel_z_kp"],
                                    config["accel_r_kp"]])
            self._accel_pids.set_i([config["accel_x_ki"],
                                    config["accel_y_ki"],
                                    config["accel_z_ki"],
                                    config["accel_r_ki"]])
            self._accel_pids.set_d([config["accel_x_kd"],
                                    config["accel_y_kd"],
                                    config["accel_z_kd"],
                                    config["accel_r_kd"]])
            self._accel_pids.set_sat([config["accel_x_sat"],
                                      config["accel_y_sat"],
                                      config["accel_z_sat"],
                                      config["accel_r_sat"]])
        self._vel_pids.set_p([config["vel_x_kp"],
                              config["vel_y_kp"],
                              config["vel_z_kp"],
                              config["vel_r_kp"]])
        self._vel_pids.set_i([config["vel_x_ki"],
                              config["vel_y_ki"],
                              config["vel_z_ki"],
                              config["vel_r_ki"]])
        self._vel_pids.set_d([config["vel_x_kd"],
                              config["vel_y_kd"],
                              config["vel_z_kd"],
                              config["vel_r_kd"]])
        self._vel_pids.set_sat([config["vel_x_sat"],
                                config["vel_y_sat"],
                                config["vel_z_sat"],
                                config["vel_r_sat"]])
        self._vel_limits = np.array([[config["vel_x_lim"]],
                                     [config["vel_y_lim"]],
                                     [config["vel_z_lim"]],
                                     [config["vel_r_lim"]]], dtype=float)
        self._pos_pids.set_p([config["pos_x_kp"],
                              config["pos_y_kp"],
                              config["pos_z_kp"],
                              config["pos_r_kp"]])
        self._pos_pids.set_i([config["pos_x_ki"],
                              config["pos_y_ki"],
                              config["pos_z_ki"],
                              config["pos_r_ki"]])
        self._pos_pids.set_d([config["pos_x_kd"],
                              config["pos_y_kd"],
                              config["pos_z_kd"],
                              config["pos_r_kd"]])
        self._pos_pids.set_sat([config["pos_x_sat"],
                                config["pos_y_sat"],
                                config["pos_z_sat"],
                                config["pos_r_sat"]])
        return config

    # ------- CONTROL MODE SERVICE --------
    def _handle_controller_state(self, state: SetControllerStateRequest):
        """
            Handles the "set_contoller_state" service.
            When called, places the FCU into manual flight mode and clears the last accel, vel, and pos setpoints.
        """
        res = SetControllerStateResponse()
        res.success = False
        if not self._ready:
            return res
        self._controller_state = state.state
        # Set the vehicle to stabilize flight mode
        req = SetModeRequest()
        #req.base_mode = req.MAV_MODE_STABILIZE_ARMED
        req.custom_mode = '0'  # STABILIZE FLIGHT MODE
        req.custom_mode = "19"  # MANUAL FLIGHT MODE
        # req.custom_mode = "0"  # STABILIZE FLIGHT MODE
        # req.custom_mode = "2"    # ALT_HOLD FLIGHT MODE
        self._configure_mavros.wait_for_service()
        res.success = self._configure_mavros(req).mode_sent
        self._control_depth = True
        self._control_sway = True
        self._control_yaw = True
        self._latest_accel_sp = None
        self._latest_vel_sp = None
        self._latest_pos_sp = None
        self._latest_ap_sp = None
        return res

    # ------- EXTERNAL SETPOINT SERVICE ---------
    def _autopilot_service(self, req: SetAutopilotRequest):
        res = SetAutopilotResponse()
        if not self._ready:
            rospy.logwarn_throttle(10, f"{self._name} | Controller not ready yet.")
            return
        self._latest_ap_sp = req.settings
        if self._controller_state.state != ControllerState.AUTOPILOT:
            self._controller_state.state = ControllerState.AUTOPILOT
        else:
            rospy.logwarn_throttle(10, f"{self._name} | Controller already in autpilot state, make sure you are not publishing on autopilot/setpoint topic.")
        res.success = True
        return res

    # ------- EXTERNAL SETPOINT HANDLERS --------
    def _vel_sp(self, msg: TwistStamped):
        """ Set the velocity """
        if not self._ready:
            return
        if self._controller_state.state == self._controller_state.VELTELEOP:
            self._latest_vel_sp = msg

    def _accel_sp(self, msg: AccelStamped):
        """ Set the velocity """
        if not self._ready:
            return
        if self._controller_state.state == self._controller_state.ACCELTELEOP:
            self._latest_accel_sp = msg

    def _pos_sp(self, msg: PoseStamped):
        """ Set the position """
        if not self._ready:
            return
        self._latest_pos_sp = msg

    def _ap_sp(self, msg: Autopilot):
        """ Set the autopilot """
        if not self._ready:
            return
        self._latest_ap_sp = msg

    # ------- FEEDBACK HANDLERS --------
    def _odom_feedback(self, msg: Odometry):
        """ Handle Odometry """
        if not self._ready:
            return
        self._latest_odom_fb = msg

    def _accel_feedback(self, msg: AccelWithCovarianceStamped):
        """ Handle Accleration """
        if not self._ready:
            return
        self._latest_accel_fb = msg

    def _range_feedback(self, msg: Range):
        """ Handle Ranged """
        if not self._ready:
            return
        self._latest_range_fb = msg

    def _depth_feedback(self, msg: Float64):
        """ Handle Depth """
        if not self._ready:
            return
        self._latest_depth = msg.data

    # -------- CONTROL CALCULATIONS -------
    def _calc_surface(self):
        """
            Forces the vehicle to surface
        """
        cmd_accel = AccelStamped(Header(0, rospy.Time.now(), ""), None)
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | SURFACING! No odom feedback.".format(self._name))
            cmd_accel.accel.linear.z = 10
        else:
            cmd_accel.accel.linear.z = 10 if self._latest_odom_fb.pose.pose.position.z < -0.5 else 0
        self._latest_accel_sp = cmd_accel
        return True

    def _hold_pos(self):
        """
            Configures vehicle to hold latest odom position, uses autopilot controller.
        """
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(self._name))
            return False
        cmd_ap = Autopilot()
        cmd_ap.U = 0
        cmd_ap.Z = self._latest_odom_fb.pose.pose.position.z
        euler = Rotation.from_quat([self._latest_odom_fb.pose.pose.orientation.x,
                                    self._latest_odom_fb.pose.pose.orientation.y,
                                    self._latest_odom_fb.pose.pose.orientation.z,
                                    self._latest_odom_fb.pose.pose.orientation.w]).as_euler("xyz")
        cmd_ap.heading = euler[-1]
        cmd_ap.height_reference = cmd_ap.MSL
        self._control_yaw = False
        self._latest_ap_sp = cmd_ap
        return True

    def _calc_autopilot_vel(self):
        """
        Calculate Velocity Setpoints from U (m/s), z (m) and psi (rad)
        Z may be depth, height referenced, or if None will be ignored. Default is depth.
        """
        if self._latest_ap_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No autopilot setpoint.".format(self._name))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(self._name))
            return False
        cmd_vel = TwistStamped(Header(0, rospy.Time.now(), self._latest_odom_fb.child_frame_id),
                               Twist(None, None))
        cmd_vel.twist.linear.x = self._latest_ap_sp.U

        heading_set_ned = self._latest_ap_sp.heading
        orientation_setpoint_ned = QuaternionStamped(Header(0, rospy.Time.now(), self._map_ned_frame_id),
                                                     euler_to_quaternion(0, 0, heading_set_ned))
        #orientation_setpoint_enu = self._tf_buff.transform(orientation_setpoint_ned, self._map_frame_id, 0)
        orientation_setpoint_enu = tf2_quaternion_rotate(self._tf_buff, orientation_setpoint_ned, self._map_frame_id)
        if orientation_setpoint_enu is None:
            return False
        heading_set_enu = quaternion_to_euler(orientation_setpoint_enu.quaternion, False)[-1]
        if self._latest_odom_fb.header.frame_id != self._map_frame_id:
            orientation_enu = tf2_quaternion_rotate(self._tf_buff, QuaternionStamped(Header(0, rospy.Time.now(), self._latest_odom_fb.header.frame_id),
                                                      self._latest_odom_fb.pose.pose.orientation), self._map_frame_id)
            if orientation_enu is None:
                return False
        else:
            orientation_enu = QuaternionStamped(self._latest_odom_fb.header, self._latest_odom_fb.pose.pose.orientation)
        heading_enu = quaternion_to_euler(orientation_enu.quaternion, False)[-1]
        heading_error = np.array([heading_set_enu - heading_enu])
        heading_error = np.where(np.bitwise_and(np.abs(heading_error) > np.pi, heading_error < 0),
                                 heading_error + 2 * np.pi, heading_error)
        heading_error = np.where(np.bitwise_and(np.abs(heading_error) > np.pi,
                                                heading_error > 0), heading_error - 2 * np.pi, heading_error)
        # TODO this gets set to None if load_waypoints gets called too quickly
        if self._latest_ap_sp is None:
            return False
        if self._latest_depth is None:
            return False

        if self._latest_ap_sp.height_reference.lower() == self._latest_ap_sp.MSL:
            z_error = self._latest_ap_sp.Z - self._latest_depth
            self._control_depth = True
        elif self._latest_ap_sp.height_reference.lower() == self._latest_ap_sp.DPT:
            z_error = -self._latest_ap_sp.Z - self._latest_depth
            self._control_depth = True
        elif self._latest_ap_sp.height_reference.lower() == self._latest_ap_sp.BTM:
            if self._latest_range_fb is None:
                rospy.logerr_throttle(5, "{} | {}".format(self._name,
                                                          "No ranging info available for altitude control."))
                return False
            z_error = self._latest_ap_sp.Z - self._latest_range_fb.range
            self._control_depth = True
        else:
            z_error = 0
            self._control_depth = False
        pos_err = np.array(
            [[0],
             [0],
             [z_error],
             [heading_error]
             ],
            dtype=float
        )
        _, _, cmd_vel.twist.linear.z, cmd_vel.twist.angular.z = self._pos_pids(pos_err, rospy.Time.now().to_sec())
        self._control_sway = False
        self._latest_vel_sp = cmd_vel
        return True

    # REGULATE POSITION WITH VELOCITY
    def _calc_vel(self):
        """
        Calculate Velocity Setpoint from Pose Setpoint PID
        """
        if self._latest_pos_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No pos setpoint.".format(self._name))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No pos feedback.".format(self._name))
            return False
        # Placing the setpoint into body-fixed coordinates (similar to Vessel Parallel Transform)
        cmd_pos = self._latest_pos_sp
        cmd_pos.header.stamp = rospy.Time.from_sec(0.0)  # Get the latest available transform
        body_pose = self._tf_buff.transform(cmd_pos, self._latest_odom_fb.child_frame_id)
        euler_error = Rotation.from_quat([body_pose.pose.orientation.x,
                                          body_pose.pose.orientation.y,
                                          body_pose.pose.orientation.z,
                                          body_pose.pose.orientation.w]).as_euler("xyz")
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error < 0), euler_error + 2 * np.pi,
                               euler_error)
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error > 0), euler_error - 2 * np.pi,
                               euler_error)
        pos_err = np.array(
            [[body_pose.pose.position.x],
             [body_pose.pose.position.y],
             [body_pose.pose.position.z],
             [euler_error[-1]]
             ],
            dtype=float
        )

        vel_sp_msg = TwistStamped()
        vel_sp_msg.header.frame_id = self._latest_odom_fb.child_frame_id
        vel_sp_msg.twist.linear.x, vel_sp_msg.twist.linear.y, vel_sp_msg.twist.linear.z, vel_sp_msg.twist.angular.z = np.clip(
            self._vel_pids(pos_err, self._latest_odom_fb.header.stamp.to_sec()).reshape(4, 1), -self._vel_limits,
            self._vel_limits)
        self._latest_vel_sp = vel_sp_msg
        return True

    def _enforce_deadband(self, vector):
        vector.x = 0 if np.abs(vector.x) < self._vel_deadband else vector.x
        vector.y = 0 if np.abs(vector.y) < self._vel_deadband else vector.y
        vector.z = 0 if np.abs(vector.z) < self._vel_deadband else vector.z
        return vector

    # REGULATE VELOCITY WITH ACCELERATION
    def _calc_accel(self):
        """
        Calculate Acceleration Setpoint from Twist Setpoint PID
        """
        if self._latest_vel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No vel setpoint.".format(self._name))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No vel feedback.".format(self._name))
            return False
        # Convert setpoint velocity into the correct frame
        if self._latest_vel_sp.header.frame_id != self._latest_odom_fb.child_frame_id and len(
                self._latest_vel_sp.header.frame_id) > 0:
            if self._tf_buff.can_transform(self._latest_odom_fb.child_frame_id, self._latest_vel_sp.header.frame_id,
                                           rospy.Time.from_sec(0.0)):
                cmd_vel = TwistStamped()
                header = Header(self._latest_vel_sp.header.seq, rospy.Time.from_sec(0.0),
                                self._latest_vel_sp.header.frame_id)
                cmd_vel.twist.linear = self._tf_buff.transform(Vector3Stamped(header,
                                                                              self._latest_vel_sp.twist.linear),
                                                               self._latest_odom_fb.child_frame_id,
                                                               rospy.Duration(5)).vector
                cmd_vel.twist.angular = self._tf_buff.transform(Vector3Stamped(header,
                                                                               self._latest_vel_sp.twist.angular),
                                                                self._latest_odom_fb.child_frame_id,
                                                                rospy.Duration(5)).vector
                cmd_vel.header.stamp = rospy.Time.now()
            else:
                rospy.logwarn_throttle(5, "{} | Cannot TF Velocity SP frame_id: {}".format(self._name,
                                                                                           self._latest_vel_sp.header.frame_id))
                return False
        else:
            cmd_vel = self._latest_vel_sp
        # clean_linear_vel = self._enforce_deadband(self._latest_odom_fb.twist.twist.linear)
        # clean_angular_vel = self._enforce_deadband(self._latest_odom_fb.twist.twist.angular)
        clean_linear_vel = self._latest_odom_fb.twist.twist.linear
        clean_angular_vel = self._latest_odom_fb.twist.twist.angular
        vel_err = np.array(
            [[cmd_vel.twist.linear.x - clean_linear_vel.x],
             [cmd_vel.twist.linear.y - clean_linear_vel.y],
             [cmd_vel.twist.linear.z - clean_linear_vel.z],
             [cmd_vel.twist.angular.z - self._latest_odom_fb.twist.twist.angular.z]],
            dtype=float)
        accel_sp_msg = AccelStamped()
        accel_sp_msg.header = cmd_vel.header
        accel_sp_msg.accel.linear.x, accel_sp_msg.accel.linear.y, accel_sp_msg.accel.linear.z, accel_sp_msg.accel.angular.z = self._vel_pids(
            vel_err, self._latest_odom_fb.header.stamp.to_sec())
        accel_sp_msg.accel.linear.y = 0 if not self._control_sway else accel_sp_msg.accel.linear.y
        accel_sp_msg.accel.linear.z = 0 if not self._control_depth else accel_sp_msg.accel.linear.z
        accel_sp_msg.accel.angular.z = 0 if not self._control_yaw else accel_sp_msg.accel.angular.z
        self._latest_accel_sp = accel_sp_msg
        return True

    def _calc_wrench(self):
        """
        Calculate Wrench Setpoint from Accel Message
        Uses the odometry child_frame_id as the body-fixed frame
        Will transform the requested setpoint into the body-fixed frame.
        """
        if self._latest_accel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No accel setpoint.".format(self._name))
            return
        if self._use_accel_fb and self._latest_accel_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No accel feedback.".format(self._name))
            return
        # Convert setpoint acceleration into the same frame
        if self._use_accel_fb and self._latest_accel_sp.header.frame_id != self._latest_accel_fb.header.frame_id and len(
                self._latest_accel_sp.header.frame_id) > 0:
            if self._tf_buff.can_transform(self._latest_accel_fb.header.frame_id,
                                           self._latest_accel_sp.header.frame_id, rospy.Time.from_sec(0.0),
                                           rospy.Duration(5)):
                header = Header(self._latest_accel_sp.header.seq, rospy.Time.from_sec(0.0),
                                self._latest_accel_sp.header.frame_id)
                cmd_accel = AccelStamped()
                cmd_accel.accel.linear = self._tf_buff.transform(Vector3Stamped(header,
                                                                                self._latest_accel_sp.accel.linear),
                                                                 self._latest_accel_fb.header.frame_id,
                                                                 rospy.Duration(5)).vector
                cmd_accel.accel.angular = self._tf_buff.transform(Vector3Stamped(header,
                                                                                 self._latest_accel_sp.accel.angular),
                                                                  self._latest_accel_fb.header.frame_id,
                                                                  rospy.Duration(5)).vector
            else:
                rospy.logwarn_throttle(5, "{} | Cannot TF Acceleration SP frame_id: {}".format(self._name,
                                                                                               self._latest_accel_sp.header.frame_id))
                return False
        else:
            cmd_accel = self._latest_accel_sp  # If it's already in the correct frame then never mind.
        if self._use_accel_fb:
            accel_err = np.array(
                [[cmd_accel.accel.linear.x - self._latest_accel_fb.accel.accel.linear.x],
                 [cmd_accel.accel.linear.y - self._latest_accel_fb.accel.accel.linear.y],
                 [cmd_accel.accel.linear.z - self._latest_accel_fb.accel.accel.linear.z],
                 [cmd_accel.accel.angular.z - self._latest_accel_fb.accel.accel.angular.z]],
                dtype=float)
            tau = self._accel_pids(accel_err, self._latest_accel_fb.header.stamp.to_sec())
        else:
            accel = np.array([[cmd_accel.accel.linear.x],
                              [cmd_accel.accel.linear.y],
                              [cmd_accel.accel.linear.z],
                              [cmd_accel.accel.angular.x],
                              [cmd_accel.accel.angular.y],
                              [cmd_accel.accel.angular.z]])
            tau = self.mass_inertial_matrix.dot(accel)[[0, 1, 2, -1], :]
        self._wrench.header.stamp = rospy.Time.now()
        self._wrench.header.frame_id = cmd_accel.header.frame_id
        self._wrench.wrench.force.x, self._wrench.wrench.force.y, self._wrench.wrench.force.z, self._wrench.wrench.torque.z = tau.squeeze()
        self._wrench_pub.publish(self._wrench)

    def _main_loop(self, event):
        if not self._ready:
            return
        self._controller_state_pub.publish(self._controller_state)  # send the controller's state
        if self._controller_state.state <= self._controller_state.IDLE:
            self._latest_accel_sp = AccelStamped(Header(0, rospy.Time.now(), self._base_frame_id), None)
            self._calc_wrench()
        elif self._controller_state.state == self._controller_state.ACCELTELEOP:
            self._calc_wrench()
        elif self._controller_state.state == self._controller_state.VELTELEOP:
            if self._calc_accel():
                self._calc_wrench()
        elif self._controller_state.state == self._controller_state.HOLDPOSITION:
            if self._hold_pos():
                if self._calc_autopilot_vel():
                    if self._calc_accel():
                        self._calc_wrench()
        elif self._controller_state.state == self._controller_state.AUTOPILOT:
            # Given heading and forward velocity, match it!
            if self._calc_autopilot_vel():
                if self._calc_accel():
                    self._calc_wrench()
        elif self._controller_state.state == self._controller_state.LOSGUIDANCE:
            if self._calc_autopilot_vel():
                if self._calc_accel():
                    self._calc_wrench()
        elif self._controller_state.state == self._controller_state.ABORT:
            if self._calc_surface():
                self._calc_wrench()
        else:
            rospy.logerr_throttle(5.0,
                                  f"{self._name} | Control mode {self._controller_state.state} not implemented.")