#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry  # We listen to this message for state estimation
from geometry_msgs.msg import AccelWithCovarianceStamped, WrenchStamped, AccelStamped, TwistStamped, PoseStamped
from PID import PIDRegulator
from mavros_msgs.srv import SetMode, SetModeRequest
import numpy as np
from bluerov2_control.srv import SetControlMode, SetControlModeResponse, SetControlModeRequest
from bluerov2_control.msg import ControlMode
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation
from dynamic_reconfigure.server import Server
from bluerov2_control.cfg import pid_reconfigConfig


class MIMOPID(object):
    def __init__(self, *args):
        self._regulators = [PIDRegulator(*arg) for arg in args]

    def set_p(self, gains):
        for r, g in zip(self._regulators, gains):
            r.p = g

    def get_p(self):
        return [r.p for r in self._regulators]

    def set_i(self, gains):
        for r, g in zip(self._regulators, gains):
            r.i = g
            r.integral = 0

    def get_i(self):
        return [r.i for r in self._regulators]

    def set_d(self, gains):
        for r, g in zip(self._regulators, gains):
            r.d = g

    def get_d(self):
        return [r.d for r in self._regulators]

    def set_sat(self, sats):
        for r, s in zip(self._regulators, sats):
            r.sat = s
            r.integral = 0

    def get_sat(self):
        return [r.sat for r in self._regulators]

    def __call__(self, errors, t):
        return np.array([[r.regulate(e, t)] for r, e in zip(self._regulators, errors)], dtype=float)[:,None]


class SimpleCascade(object):
    """
    Nothing fancy here, just a 4 DoF (Surge, Sway, Heave, Yaw)
    Cascade Controller Can be Configured for Different Modes of Control
    Modes
    AccelTeleop -> Acceleration Setpoint Given by AccelStamped Message      [ ]
    VelTeleop -> Velocity Setpoint Given by Vector3Stamped Message          [ ]
    HoldPosition -> Position Setpoint Set to Latest Odometry Message        [ ]
    LOSPosition -> Position Setpoint Set to Requested PointStamped Message  [ ]
    Autopilot -> Velocity Setpoint Set to Requested Vector3Stamped Message  [ ]

    Outputs WrenchStamped Message

    Requirements

    """

    def __init__(self):
        self._ready = False
        self._first = True
        self._mode = ControlMode()
        self._mode.mode = self._mode.OFF
        self._use_accel_fb = rospy.get_param("~use_accel_fb", default=False)

        #-------- TF Buffer ----------
        self._tf_buff = Buffer()
        TransformListener(self._tf_buff)

        #-------- Service Advertisements ---------
        self._set_control_mode = rospy.Service("controller/set_control_mode", SetControlMode, self._handle_control_mode)
        self._configure_mavros = rospy.ServiceProxy("mavros/set_mode", SetMode)

        #-------- State Config -------------------
        self._control_mode_pub = rospy.Publisher("controller/mode", ControlMode, queue_size=5)
        self._latest_odom_fb = None
        rospy.Subscriber("odometry/feedback", Odometry, self._odom_feedback)

        #-------- Wrench Config ------------------
        self._wrench = WrenchStamped()
        self._wrench_pub = rospy.Publisher("wrench/target", WrenchStamped, queue_size=5)

        #-------- Acceleration Config ------------
        self._latest_accel_sp = None
        rospy.Subscriber("accel/setpoint", AccelStamped, self._accel_sp)
        # If using acceleration feedback, then setup the PID Gains and subscribe to the acceleration feedback
        if self._use_accel_fb:
            self._accel_pids = MIMOPID([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0,0,0,0],
                                        [0,0,0,0]])
            # accel_gains = rospy.get_param("~accel")
            # self._accel_pids = MIMOPID( [accel_gains["x"]["KP"], accel_gains["x"]["KI"], accel_gains["x"]["KD"], accel_gains["x"]["sat"]],
            #                             [accel_gains["y"]["KP"], accel_gains["y"]["KI"], accel_gains["y"]["KD"], accel_gains["y"]["sat"]],
            #                             [accel_gains["z"]["KP"], accel_gains["z"]["KI"], accel_gains["z"]["KD"], accel_gains["z"]["sat"]],
            #                             [accel_gains["r"]["KP"], accel_gains["r"]["KI"], accel_gains["r"]["KD"], accel_gains["r"]["sat"]])
            self._latest_accel_fb = None
            rospy.Subscriber("accel/feedback", AccelWithCovarianceStamped, self._accel_feedback)
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

        #--------- Velocity Config ----------
        self._vel_pids = MIMOPID([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0,0,0,0],
                                        [0,0,0,0]])
        # vel_gains = rospy.get_param("~vel")
        # self._vel_pids = MIMOPID(
        #     [vel_gains["x"]["KP"], vel_gains["x"]["KI"], vel_gains["x"]["KD"], vel_gains["x"]["sat"]],
        #     [vel_gains["y"]["KP"], vel_gains["y"]["KI"], vel_gains["y"]["KD"], vel_gains["y"]["sat"]],
        #     [vel_gains["z"]["KP"], vel_gains["z"]["KI"], vel_gains["z"]["KD"], vel_gains["z"]["sat"]],
        #     [vel_gains["r"]["KP"], vel_gains["r"]["KI"], vel_gains["r"]["KD"], vel_gains["r"]["sat"]])
        rospy.Subscriber("vel/setpoint", TwistStamped, self._vel_sp)
        self._vel_limits = np.zeros((4,1), dtype=float)
        # self._vel_limits = np.array([[vel_gains["x"]["lim"]],[vel_gains["y"]["lim"]],[vel_gains["z"]["lim"]],[vel_gains["r"]["lim"]]], dtype=float)
        self._latest_vel_sp = None

        #---------- Position Config ---------
        self._pos_pids = MIMOPID([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0,0,0,0],
                                        [0,0,0,0]])
        # pos_gains = rospy.get_param("~pos")
        # self._pos_pids = MIMOPID(
        #     [pos_gains["x"]["KP"], pos_gains["x"]["KI"], pos_gains["x"]["KD"], pos_gains["x"]["sat"]],
        #     [pos_gains["y"]["KP"], pos_gains["y"]["KI"], pos_gains["y"]["KD"], pos_gains["y"]["sat"]],
        #     [pos_gains["z"]["KP"], pos_gains["z"]["KI"], pos_gains["z"]["KD"], pos_gains["z"]["sat"]],
        #     [pos_gains["r"]["KP"], pos_gains["r"]["KI"], pos_gains["r"]["KD"], pos_gains["r"]["sat"]])
        rospy.Subscriber("pos/setpoint", PoseStamped, self._pos_sp)
        self._latest_pos_sp = None

        # -------- Dynamic Reconfigure Server -------
        self._reconfig_serv = Server(pid_reconfigConfig, self._reconfig)

        #---------- Timer Loops -----------
        rospy.Timer(rospy.Duration.from_sec(1.0 / 10.0), self._main_loop)
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

    def _handle_control_mode(self, mode):
        res = SetControlModeResponse()
        res.success = False
        if not self._ready:
            return res
        self._mode = mode.mode
        self._latest_pos_sp = PoseStamped()
        self._latest_pos_sp.header = self._latest_odom_fb.header
        self._latest_pos_sp.pose = self._latest_odom_fb.pose.pose
        req = SetModeRequest()
        req.base_mode = 0
        req.custom_mode = "19"
        self._configure_mavros.wait_for_service()
        res.success = self._configure_mavros(req).mode_sent
        return res

    # ------- EXTERNAL SETPOINT HANDLERS --------
    def _vel_sp(self, msg):
        if not self._ready:
            return
        if self._mode.mode == self._mode.VELTELEOP:
            self._latest_vel_sp = msg

    def _accel_sp(self, msg):
        if not self._ready:
            return
        if self._mode.mode == self._mode.ACCELTELEOP:
            self._latest_accel_sp = msg

    def _pos_sp(self, msg):
        if not self._ready:
            return
        self._latest_pos_sp = msg

    # ------- FEEDBACK HANDLERS --------

    def _odom_feedback(self, msg):
        if not self._ready:
            return
        self._latest_odom_fb = msg

    def _accel_feedback(self, msg):
        if not self._ready:
            return
        self._latest_accel_fb = msg

    # -------- CONTROL CALCULATIONS -------
    def _calc_vel(self):
        """
        Calculate Velocity Setpoint from Pose Setpoint PID
        """
        if self._latest_pos_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No pos setpoint.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No pos feedback.".format(rospy.get_name()))
            return False
        # The pose_sp frame in "base_link" frame coordinates gives body fixed feedback error
        msg = self._latest_pos_sp
        msg.header.stamp = rospy.Time.now()  # work around to stop setpoint from going out of date
        body_pose = self._tf_buff.transform(msg, self._latest_odom_fb.child_frame_id, rospy.Duration(5))
        euler_error = Rotation.from_quat([body_pose.pose.orientation.x,
                                            body_pose.pose.orientation.y,
                                            body_pose.pose.orientation.z,
                                            body_pose.pose.orientation.w]).as_euler("xyz")
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error < 0), euler_error + 2*np.pi, euler_error)
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error > 0), euler_error - 2*np.pi, euler_error)
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
        vel_sp_msg.twist.linear.x, vel_sp_msg.twist.linear.y, vel_sp_msg.twist.linear.z, vel_sp_msg.twist.angular.z = np.clip(self._vel_pids(pos_err, self._latest_odom_fb.header.stamp.to_sec()).reshape(4, 1), -self._vel_limits, self._vel_limits)
        self._latest_vel_sp = vel_sp_msg
        return True

    def _calc_accel(self):
        """
        Calculate Acceleration Setpoint from Twist Setpoint PID
        """
        if self._latest_vel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No vel setpoint.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No vel feedback.".format(rospy.get_name()))
            return False
        vel_err = np.array(
            [[self._latest_vel_sp.twist.linear.x - self._latest_odom_fb.twist.twist.linear.x],
             [self._latest_vel_sp.twist.linear.y - self._latest_odom_fb.twist.twist.linear.y],
             [self._latest_vel_sp.twist.linear.z - self._latest_odom_fb.twist.twist.linear.z],
             [self._latest_vel_sp.twist.angular.z - self._latest_odom_fb.twist.twist.angular.z]],
            dtype=float)
        accel_sp_msg = AccelStamped()
        accel_sp_msg.header.stamp = self._latest_odom_fb.header.stamp.to_sec()
        # TODO the frame should be relative to the Twist Message frame_id
        accel_sp_msg.header.frame_id = self._latest_odom_fb.child_frame_id
        accel_sp_msg.accel.linear.x, accel_sp_msg.accel.linear.y, accel_sp_msg.accel.linear.z, accel_sp_msg.accel.angular.z = self._vel_pids(vel_err, self._latest_odom_fb.header.stamp.to_sec())
        self._latest_accel_sp = accel_sp_msg
        return True

    def _calc_wrench(self):
        """
        Calculate Wrench Setpoint from External Accel Message
        """
        if self._latest_accel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No accel setpoint.".format(rospy.get_name()))
            return
        if self._use_accel_fb:
            if self._latest_accel_fb is None:
                rospy.logwarn_throttle(10.0, "{} | No accel feedback.".format(rospy.get_name()))
            # We assume that commanded acceleration is in the correct base_link FLU frame
            accel_err = np.array(
                [[self._latest_accel_sp.accel.linear.x - self._latest_accel_fb.accel.accel.linear.x],
                 [self._latest_accel_sp.accel.linear.y - self._latest_accel_fb.accel.accel.linear.y],
                 [self._latest_accel_sp.accel.linear.z - self._latest_accel_fb.accel.accel.linear.z],
                 [self._latest_accel_sp.accel.angular.z - self._latest_accel_fb.accel.accel.angular.z]],
                dtype=float)
            tau = self._accel_pids(accel_err, self._latest_accel_fb.header.stamp.to_sec())
        else:
            accel = np.array([[self._latest_accel_sp.accel.linear.x],
                              [self._latest_accel_sp.accel.linear.y],
                              [self._latest_accel_sp.accel.linear.z],
                              [self._latest_accel_sp.accel.angular.x],
                              [self._latest_accel_sp.accel.angular.y],
                              [self._latest_accel_sp.accel.angular.z]])
            tau = self.mass_inertial_matrix.dot(accel)[[0, 1, 2, -1], :]
        self._wrench.header.stamp = rospy.Time.now()
        self._wrench.header.frame_id = self._latest_odom_fb.child_frame_id
        self._wrench.wrench.force.x, self._wrench.wrench.force.y, self._wrench.wrench.force.z, self._wrench.wrench.torque.z = tau
        self._wrench_pub.publish(self._wrench)

    def _main_loop(self, event):
        if not self._ready:
            return
        self._control_mode_pub.publish(self._mode)  # send the controller's state
        if self._mode.mode == self._mode.OFF:
            return
        elif self._mode.mode == self._mode.IDLE:
            return
        elif self._mode.mode == self._mode.ACCELTELEOP:
            self._calc_wrench()
        elif self._mode.mode == self._mode.VELTELEOP:
            if self._calc_accel():
                self._calc_wrench()
        elif self._mode.mode == self._mode.HOLDPOSITION:
            self._calc_vel()
            self._calc_accel()
            self._calc_wrench()
        elif self._mode.mode == self._mode.AUTOPILOT:
            # Given
            self._calc_autopilot_vel()
            self._calc_accel()
            self._calc_wrench()
        elif self._mode.mode == self._mode.LOSPOSITION:
            self._calc_vel()
            self._calc_accel()
            self._calc_wrench()
        else:
            raise NotImplementedError


if __name__=="__main__":
    rospy.init_node("controller")
    try:
        controller = SimpleCascade()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
