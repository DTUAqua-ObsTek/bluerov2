#!/usr/bin/env python


import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range, CompressedImage, BatteryState, CameraInfo, Temperature, NavSatFix, Image
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from waterlinked_gps_msgs.msg import PositionMaster
import numpy as np
from image_geometry import PinholeCameraModel
from tf2_ros import TransformListener, Buffer

class SimpleHud:
    def __init__(self):
        self._c = 0
        self._info_in = "image_in/camera_info"
        #self._image_in = "image_in/compressed" #use image_raw instead
        self._image_in = "image_in/image_raw" #use image_raw instead
        self._image_out = "image_out/compressed"
        self._cvbridge = CvBridge()
        self._heading_topic = "mavros/global_position/compass_hdg"
        self._depth_topic = "mavros/global_position/rel_alt"
        self._alt_topic = "mavros/distance_sensor/rangefinder_pub"
        self._state_topic = "mavros/state"
        self._batt_topic = "mavros/battery"
        self._temp_topic = "mavros/imu/temperature_baro"
        self._pose_topic = "waterlinked/pose_with_cov_stamped"
        self._latlon_topic = "mavros/global_position/global"
        self._sogcog_topic = "waterlinked/position/master"
        self._fontsize = rospy.get_param("fontsize", 1.5)
        self._heading = Float64()
        self._alt = Range()
        self._depth = Float64()
        self._battery = BatteryState()
        self._state = State()
        self._temperature = Temperature()
        self._pose = PoseWithCovarianceStamped()
        self._latlon = NavSatFix()
        self._sogcog = PositionMaster()
        self._cam_model = PinholeCameraModel()
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer)
        self._home = None
        #self._pub = rospy.Publisher(self._image_out, CompressedImage, queue_size=10)
        self._pub = rospy.Publisher(self._image_out, Image, queue_size=10)
        #rospy.Subscriber(self._image_in, CompressedImage, self._annotate_img)
        rospy.Subscriber(self._image_in, Image, self._annotate_img)
        rospy.Subscriber(self._info_in, CameraInfo, self._update_info)
        rospy.Subscriber(self._heading_topic, Float64, self._update_heading)
        rospy.Subscriber(self._depth_topic, Float64, self._update_depth)
        rospy.Subscriber(self._alt_topic, Range, self._update_alt)
        rospy.Subscriber(self._state_topic, State, self._update_state)
        rospy.Subscriber(self._batt_topic, BatteryState, self._update_bat)
        rospy.Subscriber(self._temp_topic, Temperature, self._update_temperature)
        rospy.Subscriber(self._pose_topic, PoseWithCovarianceStamped, self._update_pose)
        rospy.Subscriber(self._latlon_topic, NavSatFix, self._update_latlon)
        rospy.Subscriber(self._sogcog_topic,PositionMaster, self._update_sogcog)

    def _update_heading(self, msg):
        self._heading = msg

    def _update_state(self, msg):
        self._state = msg

    def _update_depth(self, msg):
        self._depth = msg

    def _update_alt(self, msg):
        self._alt = msg

    def _update_bat(self, msg):
        self._battery = msg

    def _update_temperature(self, msg):
        self._temperature = msg

    def _update_pose(self, msg):
        self._pose = msg

    def _update_latlon(self, msg):
        self._latlon = msg

    def _update_sogcog(self, msg):
        self._sogcog = msg

    def _degToCompass(self, num):
        val = int((num / 22.5) + .5)
        arr = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        return arr[(val % 16)]

    def _update_info(self, msg):
        self._cam_model.fromCameraInfo(msg)
        try:
            tf = self._tf_buffer.lookup_transform(self._cam_model.tf_frame, "waterlinked", rospy.Time.from_sec(0), rospy.Duration.from_sec(1.0/30.0))
        except Exception as e:
            rospy.logerr_throttle(10.0, "{} | {}".format(rospy.get_name(), e.message))
            self._home = None
            return
        self._home = self._cam_model.project3dToPixel((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z))

    def _annotate_img(self, msg):
        if self._c % 5 == 0 :

            #img = self._cvbridge.compressed_imgmsg_to_cv2(msg)
            img = self._cvbridge.imgmsg_to_cv2(msg)
            color = (255, 20, 147)
            thickness = 2
            font = cv2.FONT_HERSHEY_SIMPLEX

            # TOP CENTRAL BOX: IMPORTANT DATA (Altitude, Heading, Depth)
            st = "Alt: {:02.1f}m   Hdg: {:03d} {}   Dpt: {:02.1f}m".format(self._alt.range, int(self._heading.data), self._degToCompass(self._heading.data), self._depth.data)
            (w, h) = cv2.getTextSize(st, font, self._fontsize, thickness)[0]
            pos = ((img.shape[1] - w)/2 - 2, 10)#img.shape[0] - h/2)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + h + 5), font, self._fontsize, color, thickness, cv2.LINE_AA)

            #BOTTOM LEFT BOX: SYSTEM HEALTH (State, Battery voltage, Temperature) 
            # State ROV
            pos = (0, img.shape[0] - 3*h - 20)
            st = "St: Armed" if self._state.armed else "St: Disarmed"
            img = cv2.putText(img, st, (pos[0] + 2, pos[1]), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # Battery voltage & current
            pos = (0, img.shape[0] - 2*h - 10)
            st = "Bat: {:02.1f}V".format(self._battery.voltage)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 10), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # pos = (0.1, 0.9)
            # st = "Bat: {:02.1f}A".format(self._battery.current)
            # img = cv2.putText(img, st, tuple((np.array(pos) * img.shape[1::-1]).astype(int)), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # Temperature
            pos = (0, img.shape[0] - h)
            st = "Temp: {:02.1f}degC".format(self._temperature.temperature)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 20), font, self._fontsize, color, thickness, cv2.LINE_AA)

            # TOP LEFT BOX: POSITION RELATIVE TO BOAT (LOS distance, LOS angle)
            # LOS distance
            pos = (0, 150)
            st = "LOS d: {:02.1f}m".format(np.sqrt(self._pose.pose.pose.position.x ** 2 + self._pose.pose.pose.position.y ** 2))
            img = cv2.putText(img, st, (pos[0] + 2, pos[1]), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # LOS angle
            pos = (0, 150 + h + 20)
            theta = np.arctan2(self._pose.pose.pose.position.x, self._pose.pose.pose.position.y) * 180. / np.pi
            alpha = theta - self._heading.data
            st = "LOS alpha: {:02.1f}deg".format(alpha)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1]), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # # Pose
            # pos = (0.1, 0.2)
            # st = "Pose: {:02.1f}, {:02.1f}, {:02.1f}".format(self._pose.pose.pose.position.x, self._pose.pose.pose.position.y, self._pose.pose.pose.position.z)
            # img = cv2.putText(img, st, tuple((np.array(pos) * img.shape[1::-1]).astype(int)), font, self._fontsize, color, thickness, cv2.LINE_AA)

            # TOP RIGHT BOX: POSITION GLOBAL (Latitude, Longitude, SoG, CoG)
            # Latitude
            pos = (img.shape[1] - w/3 - 100, 150)
            st = "Lat: {:02.2f}deg".format(self._latlon.latitude)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1]), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # Longitude
            pos = (img.shape[1] - w/3 - 100, 150 + h + 20)
            st = "Lon: {:02.2f}deg".format(self._latlon.longitude)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # SoG
            pos = (img.shape[1] - w/3 - 100, 150 + 2*(h + 20))
            st = "SoG: {:02.2f}km/h".format(self._sogcog.sog)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)
            # CoG
            pos = (img.shape[1] - w/3 - 100, 150 + 3*(h + 20))
            st = "CoG: {:02.2f}deg".format(self._sogcog.cog)
            img = cv2.putText(img, st, (pos[0] + 2, pos[1] + 2), font, self._fontsize, color, thickness, cv2.LINE_AA)

            if self._home is not None:
                img = cv2.circle(img, self._home, 5, (0, 255, 0), -1)
                box, baseline = cv2.getTextSize("HOME", cv2.FONT_HERSHEY_PLAIN, self._fontsize, 2)
                pos = (self._home[0] - box[0] / 2, self._home[1] - 5 - baseline - 3)
                img = cv2.putText(img, "WL", pos, cv2.FONT_HERSHEY_PLAIN, self._fontsize, (0, 255, 0), 2)

            img = img[:,:,[2,1,0]]
            #out = self._cvbridge.cv2_to_compressed_imgmsg(img)
            out = self._cvbridge.cv2_to_imgmsg(img)
            self._pub.publish(out)

            self._c = 0
        else :
            self._c = self._c + 1


def main():
    rospy.init_node("drone_hud_node")
    try:
        obj = SimpleHud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()