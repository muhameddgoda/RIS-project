#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class TagActionNode:
    def __init__(self):
        rospy.init_node('tag_action_node', anonymous=True)

        self.tag_actions = rospy.get_param('~tag_actions', {
            0: 'stop',  1: 'go',  2: 'left',  3: 'right'
        })
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.turn_speed    = rospy.get_param('~turn_speed', 0.5)

        self.pub = rospy.Publisher(
            '/motoduck/coordinator_node/car_cmd',
            Twist, queue_size=1
        )
        rospy.Subscriber(
            '/apriltag_detections',
            AprilTagDetectionArray,
            self.callback,
            queue_size=1
        )

        rospy.loginfo("✅ Tag‐Action node ready.")
        rospy.spin()

    def callback(self, msg):
        if not msg.detections:
            return

        tag_id = msg.detections[0].id[0]
        action = self.tag_actions.get(tag_id)
        twist = Twist()

        if action == 'stop':
            rospy.loginfo(f"🛑 Tag {tag_id}: STOP")
        elif action == 'go':
            twist.linear.x = self.forward_speed
            rospy.loginfo(f"▶️ Tag {tag_id}: GO")
        elif action == 'left':
            twist.linear.x = self.forward_speed
            twist.angular.z =  self.turn_speed
            rospy.loginfo(f"⬅️ Tag {tag_id}: LEFT")
        elif action == 'right':
            twist.linear.x = self.forward_speed
            twist.angular.z = -self.turn_speed
            rospy.loginfo(f"➡️ Tag {tag_id}: RIGHT")
        else:
            rospy.logwarn(f"❓ Tag {tag_id}: no action")

        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        TagActionNode()
    except rospy.ROSInterruptException:
        pass
