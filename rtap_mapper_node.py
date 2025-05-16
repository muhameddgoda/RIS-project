#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import csv
from datetime import datetime

class RTAPMapper:
    def __init__(self):
        rospy.init_node('rtap_mapper_node', anonymous=True)

        # Parameters
        self.nth_frame = rospy.get_param('~nth_frame', 30)
        base_dir = rospy.get_param('~base_dir', '/data/maps')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(base_dir, f'session_{timestamp}')
        os.makedirs(self.session_dir, exist_ok=True)

        # CSV setup
        self.csv_path = os.path.join(self.session_dir, 'mapping_log.csv')
        self.csv_file = open(self.csv_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'frame','filename','ros_time',
            'imu_ax','imu_ay','imu_az','imu_gx','imu_gy','imu_gz',
            'vel_lin_x','vel_lin_y','vel_lin_z','vel_ang_x','vel_ang_y','vel_ang_z'
        ])

        # State
        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_imu = None
        self.last_twist = None

        # Subscribers
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmdvel_callback)

        rospy.loginfo("✅ RTAP Mapper initialized; saving to %s", self.session_dir)
        rospy.spin()

    def imu_callback(self, msg):
        self.last_imu = msg

    def cmdvel_callback(self, msg):
        self.last_twist = msg

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.nth_frame != 0:
            return

        t = msg.header.stamp.to_sec()
        filename = f"frame_{self.frame_count:05d}.png"
        filepath = os.path.join(self.session_dir, filename)

        # Convert and save
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(filepath, cv_img)
        except Exception as e:
            rospy.logerr("Failed to save image: %s", e)
            return

        # Extract IMU & Twist
        imu = self.last_imu
        twist = self.last_twist
        imu_vals = [imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z] if imu else [None]*6
        vel_vals = [twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z] if twist else [None]*6

        # Log to CSV
        row = [self.frame_count, filename, t] + imu_vals + vel_vals
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        rospy.loginfo("📸 Saved %s (frame %d)", filename, self.frame_count)

if __name__ == '__main__':
    try:
        RTAPMapper()
    except rospy.ROSInterruptException:
        pass

