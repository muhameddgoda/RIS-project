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

        # HSV thresholds for white
        self.hsv_lower = np.array(rospy.get_param('~hsv_lower', [0, 0, 200]))
        self.hsv_upper = np.array(rospy.get_param('~hsv_upper', [180, 30, 255]))
        self.crop_height = rospy.get_param('~crop_height', 100)

        # Control gains
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.k_p = rospy.get_param('~k_p', 0.005)
        self.desired_frac = rospy.get_param('~desired_frac', 0.8)

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher(
            '/motoduck/coordinator_node/car_cmd', Twist, queue_size=1
        )
        rospy.Subscriber(
            '/motoduck/camera_node/image/compressed',
            CompressedImage, self.image_cb,
            queue_size=1, buff_size=2**24
        )

        rospy.loginfo("✅ White‐Line Follower initialized.")
        rospy.spin()

    def image_cb(self, msg):
        # Decode
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        h, w, _ = frame.shape

        # Crop bottom
        roi = frame[h-self.crop_height:h, :]

        # HSV threshold
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        # Find largest contour
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.cmd_pub.publish(Twist())
            rospy.logwarn_throttle(2, "⚠️ No white line detected")
            return

        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            return
        cx = M['m10']/M['m00']

        # Compute error & publish
        desired_x = self.desired_frac * w
        error = desired_x - cx
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = self.k_p * error
        self.cmd_pub.publish(twist)

        rospy.loginfo_throttle(1, f"Line at {cx:.1f}px, err={error:.1f}")

if __name__ == '__main__':
    try:
        WhiteLineFollower()
    except rospy.ROSInterruptException:
        pass
