#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class WhiteLineFollower:
    def __init__(self):
        rospy.init_node('white_line_follower_node', anonymous=True)

        # HSV thresholds for WHITE
        self.hsv_lower = np.array(rospy.get_param('~hsv_lower', [0, 0, 200]))
        self.hsv_upper = np.array(rospy.get_param('~hsv_upper', [180, 30, 255]))
        self.crop_height = rospy.get_param('~crop_height', 100)

        # Drive parameters
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.k_p = rospy.get_param('~k_p', 0.005)
        # Desired fraction across the image for the line centroid (e.g. 0.8 → 80% to the right)
        self.desired_frac = rospy.get_param('~desired_frac', 0.8)

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher(
            '/motoduck/coordinator_node/car_cmd',
            Twist, queue_size=1
        )
        rospy.Subscriber(
            '/motoduck/camera_node/image/compressed',
            CompressedImage, self.image_cb,
            queue_size=1, buff_size=2**24
        )

        rospy.loginfo("✅ White‐Line Follower initialized (keep line on right).")
        rospy.spin()

    def image_cb(self, msg):
        # decode
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        h, w, _ = frame.shape

        # crop bottom band
        roi = frame[h-self.crop_height:h, :]

        # threshold white in HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # clean up
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # find largest contour
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.cmd_pub.publish(Twist())  # stop
            rospy.logwarn_throttle(2, "⚠️ No white line detected")
            return

        # centroid of the biggest blob
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00']==0:
            return
        cx = M['m10']/M['m00']

        # error relative to desired position
        desired_x = self.desired_frac * w
        error = desired_x - cx

        # publish command
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = self.k_p * error
        self.cmd_pub.publish(twist)

        rospy.loginfo_throttle(1, f"Line at {cx:.1f}, err={error:.1f}")

if __name__=='__main__':
    try:
        WhiteLineFollower()
    except rospy.ROSInterruptException:
        pass

