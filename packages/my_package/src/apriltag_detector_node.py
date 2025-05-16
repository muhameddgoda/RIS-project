#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector_node', anonymous=True)

        opts = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(opts)
        self.bridge = CvBridge()

        self.pub = rospy.Publisher(
            '/apriltag_detections',
            AprilTagDetectionArray,
            queue_size=1
        )
        rospy.Subscriber(
            '/motoduck/camera_node/image/compressed',
            CompressedImage, self.callback,
            queue_size=1, buff_size=2**24
        )

        rospy.loginfo("✅ AprilTag Detector initialized.")
        rospy.spin()

    def callback(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        color = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)
        arr_msg = AprilTagDetectionArray(header=msg.header)

        for d in detections:
            det = AprilTagDetection()
            det.id = [d.tag_id]
            # corner_points uses geometry_msgs/Point
            det.corners = [
                self._make_point(p) for p in d.corners
            ]
            det.center = self._make_point(d.center)
            arr_msg.detections.append(det)

        self.pub.publish(arr_msg)
        rospy.loginfo_throttle(1, f"🔖 Detected {len(detections)} tag(s)")

    @staticmethod
    def _make_point(pt):
        from geometry_msgs.msg import Point
        return Point(x=float(pt[0]), y=float(pt[1]), z=0.0)

if __name__ == '__main__':
    try:
        AprilTagDetector()
    except rospy.ROSInterruptException:
        pass
