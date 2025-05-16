#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class RTAPViewer:
    def __init__(self):
        rospy.init_node('rtap_viewer_node', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber(
            '/motoduck/camera_node/image/compressed',
            CompressedImage,
            self.callback,
            queue_size=1, buff_size=2**24
        )
        rospy.loginfo("👀 RTAP Viewer started")
        rospy.spin()

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow("🚀 Camera Feed", frame)
            cv2.waitKey(1)
        else:
            rospy.logwarn("Empty frame")

if __name__ == '__main__':
    try:
        RTAPViewer()
    except rospy.ROSInterruptException:
        pass

