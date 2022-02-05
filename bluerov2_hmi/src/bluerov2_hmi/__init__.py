import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range, BatteryState, CameraInfo, Temperature, NavSatFix, Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Float64, Header
from mavros_msgs.msg import State
import numpy as np
from image_geometry import PinholeCameraModel
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from message_filters import TimeSynchronizer, Subscriber
from bluerov2_navigation.helpers import math


class MsgMonitor:
    """ Checks if a message has been received within a timeout. """
    def __init__(self, timeout: float = 5.0):
        self._last_time = rospy.Time.now()
        self._timeout = timeout

    def is_valid(self):
        return (rospy.Time.now() - self._last_time).to_sec() < self._timeout

    def __call__(self, msg: rospy.AnyMsg):
            self._last_time = rospy.Time.now()

class AnnotationFormat:
    def __init__(self, color: tuple = (255, 255, 255), thickness: int = 2, font: int = cv2.FONT_HERSHEY_SIMPLEX):
        self.color = color
        self.thickness = thickness
        self.font = font

class HudOverlay:
    def __init__(self):
        self._c = 0
        self._cvbridge = CvBridge()
        # --------- Params ---------
        self._fontsize = rospy.get_param("fontsize", 1.5)
        self._skip_frames = int(rospy.get_param("skip_frames", 5))
        # --------- Topic Defs -------
        self._info_in_topic = "image_in/camera_info"
        self._image_in_topic = "image_in/image_raw"
        self._image_out_topic = "image_out/image_raw"
        self._heading_topic = "mavros/global_position/compass_hdg"
        self._alt_topic = "mavros/distance_sensor/rangefinder_pub"
        self._state_topic = "mavros/state"
        self._batt_topic = "mavros/battery"
        self._temp_topic = "mavros/imu/temperature_baro"
        self._pose_topic = "waterlinked/pose_with_cov_stamped"
        self._latlon_topic = "mavros/global_position/global"
        self._sog_topic = "guidance/sog"
        self._cog_topic = "guidance/cog"
        self._depth_topic = "mavros/global_position/rel_alt"
        # ------- MSG Defs ---------
        self._heading = Float64()
        self._sog = Float64()
        self._cog = Float64
        self._alt = Range()
        self._depth = Float64()
        self._battery = BatteryState()
        self._state = State()
        self._temperature = Temperature()
        self._pose = PoseWithCovarianceStamped()
        self._latlon = NavSatFix()
        # ------- MSG Monitors --------
        self._heading_mon = MsgMonitor()
        self._sog_mon = MsgMonitor()
        self._cog_mon = MsgMonitor()
        self._alt_mon = MsgMonitor()
        self._depth_mon = MsgMonitor()
        self._battery_mon = MsgMonitor()
        self._state_mon = MsgMonitor()
        self._temperature_mon = MsgMonitor()
        self._pose_mon = MsgMonitor()
        self._latlon_mon = MsgMonitor()
        # ------- Image Geometry ----------
        self._cam_model = PinholeCameraModel()
        self._home = None  # Frame projected onto camera
        # ------- TF
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer)
        # ---------- Annotation -------
        self._valid_annotation = AnnotationFormat()
        self._invalid_annotation = AnnotationFormat(color=(0, 0, 255))
        # ------- Subscribers -------
        self._image_in_sub = Subscriber(self._image_in_topic, Image)
        self._info_in_sub = Subscriber(self._info_in_topic, CameraInfo)
        ts = TimeSynchronizer([self._image_in_sub, self._info_in_sub], 10)
        ts.registerCallback(self._annotate_img)
        rospy.Subscriber(self._heading_topic, Float64, self._update_heading)
        rospy.Subscriber(self._depth_topic, Float64, self._update_depth)
        rospy.Subscriber(self._alt_topic, Range, self._update_alt)
        rospy.Subscriber(self._state_topic, State, self._update_state)
        rospy.Subscriber(self._batt_topic, BatteryState, self._update_bat)
        rospy.Subscriber(self._temp_topic, Temperature, self._update_temperature)
        rospy.Subscriber(self._pose_topic, PoseWithCovarianceStamped, self._update_pose)
        rospy.Subscriber(self._latlon_topic, NavSatFix, self._update_latlon)
        rospy.Subscriber(self._cog_topic, Float64, self._update_cog)
        rospy.Subscriber(self._sog_topic, Float64, self._update_sog)
        # ---------- Publisher --------
        self._pub = rospy.Publisher(self._image_out_topic, Image, queue_size=10)


    def _update_cog(self, msg: Float64):
        self._cog_mon(msg)
        self._cog = msg

    def _update_sog(self, msg: Float64):
        self._sog_mon(msg)
        self._sog = msg

    def _update_heading(self, msg: Float64):
        self._heading_mon(msg)
        self._heading = msg

    def _update_state(self, msg: State):
        self._state_mon(msg)
        self._state = msg

    def _update_depth(self, msg: Float64):
        self._depth_mon(msg)
        self._depth = msg

    def _update_alt(self, msg: Range):
        self._alt_mon(msg)
        self._alt = msg

    def _update_bat(self, msg: BatteryState):
        self._battery_mon(msg)
        self._battery = msg

    def _update_temperature(self, msg: Temperature):
        self._temperature_mon(msg)
        self._temperature = msg

    def _update_pose(self, msg: PoseWithCovarianceStamped):
        self._pose_mon(msg)
        self._pose = msg

    def _update_latlon(self, msg: NavSatFix):
        self._latlon_mon(msg)
        self._latlon = msg

    def _degToCompass(self, num):
        val = int((num / 22.5) + .5)
        arr = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        return arr[(val % 16)]

    def _gen_annotation(self, fmt: str="{}", value: tuple = (np.inf,), anno: AnnotationFormat = AnnotationFormat()):
        st = fmt.format(*value) if value is not None else fmt.format(None)
        box, _ = cv2.getTextSize(st, anno.font, self._fontsize, anno.thickness)
        w, h = box
        b, g, r = anno.color
        return st, w, h, b, g, r

    def _annotate_img(self, image_msg: Image, info_msg: CameraInfo):
        if self._c % self._skip_frames == 0:
            # Convert image to opencv format
            img = self._cvbridge.imgmsg_to_cv2(image_msg)
            # Construct pinhole camera model
            self._cam_model.fromCameraInfo(info_msg)
            # Get the frame origin projected into image coordiantes
            self._home = None
            try:
                if self._tf_buffer.can_transform(self._cam_model.tf_frame, "waterlinked", rospy.Time.from_sec(0)):
                    point = self._tf_buffer.transform(PointStamped(Header(0, "waterlinked", rospy.Time.now()), None),
                                                      self._cam_model.tf_frame)
                    self._home = self._cam_model.project3dToPixel(point.point.x, point.point.y, point.point.z)
            except Exception as e:
                rospy.logerr_throttle(10.0, f"{rospy.get_name()} | {e}")

            # TOP CENTRAL BOX: IMPORTANT DATA (Altitude, Heading, Depth)
            st, w, h, b, g, r = self._gen_annotation("Alt: {:02.1f} m", (self._alt.range,), self._valid_annotation) if self._alt_mon.is_valid() else self._gen_annotation("Alt: {:02.1f} m", (self._alt.range,), self._invalid_annotation)
            pos = (int(img.shape[1] - w/2), 10)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b,g,r), self._valid_annotation.thickness, cv2.LINE_AA)

            st, w1, h1, b, g, r = self._gen_annotation("Hdg: {:03d} {}", (int(self._heading.data), self._degToCompass(self._heading.data)),
                                                     self._valid_annotation) if self._heading_mon.is_valid() else self._gen_annotation(
                "Hdg: {:03d} {}", (self._heading.data, self._degToCompass(self._heading.data)), self._invalid_annotation)
            pos = (int(img.shape[1] - w1/2) + w + 5, 10)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b,g,r), self._valid_annotation.thickness, cv2.LINE_AA)

            st, w2, h2, b, g, r = self._gen_annotation("Dpt: {:02.1f} m",
                                                       (self._depth.data,),
                                                       self._valid_annotation) if self._depth_mon.is_valid() else self._gen_annotation(
                "Dpt: {:02.1f} m", (self._depth.data), self._invalid_annotation)
            pos = (int(img.shape[1] - w2 / 2) + w1 + w2 + 10, 10)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)

            #BOTTOM LEFT BOX: SYSTEM HEALTH (State, Battery voltage, Temperature)
            # State ROV
            st, w, h, b, g, r = self._gen_annotation("Bat: {:02.1f} V", (self._battery.voltage,),
                                                     self._valid_annotation) if self._alt_mon.is_valid() else self._gen_annotation(
                "Bat: {:02.1f} V", (self._battery.voltage,), self._invalid_annotation)
            pos = (0, img.shape[0] - 3*h)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)
            fmt = "St: Armed" if self._state.armed else "St: Disarmed"
            st, w1, h1, b, g, r = self._gen_annotation(fmt, None,
                                                     self._valid_annotation) if self._alt_mon.is_valid() else self._gen_annotation(
                fmt, None, self._invalid_annotation)
            pos = (0, img.shape[0] - 2*h + 5 )
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)
            # Temperature
            st, w2, h2, b, g, r = self._gen_annotation("Temp: {:02.1f} degC", (self._temperature.temperature,),
                                                     self._valid_annotation) if self._alt_mon.is_valid() else self._gen_annotation(
                "Temp: {:02.1f} degC", (self._temperature.temperature,), self._invalid_annotation)
            pos = (0, img.shape[0] - 2*h + h1 + 10 )
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)

            # TOP LEFT BOX: POSITION RELATIVE TO BOAT (LOS distance, LOS angle)
            distance = np.sqrt(self._pose.pose.pose.position.x ** 2 + self._pose.pose.pose.position.y ** 2)


            st, w, h, b, g, r = self._gen_annotation("Ship Distance: {:02.1f} m", (distance,),
                                                     self._valid_annotation) if self._pose_mon.is_valid() else self._gen_annotation(
                "Ship distance: {:02.1f} m", (distance,), self._invalid_annotation)
            pos = (0, 150)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)

            bearing = math.rad2deg(math.atan2(self._pose.pose.pose.position.y,
                                              self._pose.pose.pose.position.x))  # This is ENU, convert to NED
            bearing = 90 - bearing
            bearing = 360 + bearing if bearing < 0 else bearing
            relative_bearing = bearing - self._heading.data
            relative_bearing = relative_bearing + 360 if abs(relative_bearing) > 180 and relative_bearing < 0 else relative_bearing
            relative_bearing = relative_bearing - 360 if abs(relative_bearing) > 180 and relative_bearing > 0 else relative_bearing
            st, w1, h1, b, g, r = self._gen_annotation("Ship Bearing: {:02.1f} deg", (relative_bearing,),
                                                     self._valid_annotation) if self._pose_mon.is_valid() else self._gen_annotation(
                "Ship Bearing: {:02.1f} deg", (relative_bearing,), self._invalid_annotation)
            pos = (0, 150 + h + 20)
            img = cv2.putText(img, st, pos, self._valid_annotation.font, self._fontsize, (b, g, r),
                              self._valid_annotation.thickness, cv2.LINE_AA)

            # TOP RIGHT BOX: POSITION GLOBAL (Latitude, Longitude, SoG, CoG)
            # # Latitude
            # pos = (img.shape[1] - w/3 - 100, 150)
            # st = "Lat: {:02.2f}deg".format(self._latlon.latitude)
            # img = cv2.putText(img, st, (pos[0] + 2, pos[1]), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # # Longitude
            # pos = (img.shape[1] - w/3 - 100, 150 + h + 20)
            # st = "Lon: {:02.2f}deg".format(self._latlon.longitude)
            # img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # # SoG
            # pos = (img.shape[1] - w/3 - 100, 150 + 2*(h + 20))
            # st = "SoG: {:02.2f}km/h".format(self._sogcog.sog)
            # img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # # CoG
            # pos = (img.shape[1] - w/3 - 100, 150 + 3*(h + 20))
            # st = "CoG: {:02.2f}deg".format(self._sogcog.cog)
            # img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)

            if self._home is not None:
                self._home = [int(i) for i in self._home]
                img = cv2.circle(img, self._home, 5, (0, 255, 0), -1)
                box, baseline = cv2.getTextSize("SHIP", cv2.FONT_HERSHEY_PLAIN, self._fontsize, 2)
                pos = (int(self._home[0] - box[0] / 2), self._home[1] - 5 - baseline - 3)
                img = cv2.putText(img, "SHIP", pos, cv2.FONT_HERSHEY_PLAIN, self._fontsize, (0, 255, 0), 2)

            out = self._cvbridge.cv2_to_imgmsg(img, encoding="bgr8")
            self._pub.publish(out)
            self._c = 0
        else :
            self._c = self._c + 1
