#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
import math

class ChangeLinkPose:

    def __init__(self) -> None:
        rospy.init_node('change_link_pose')
        self.pub = rospy.Publisher('link/pose', Pose, queue_size=10)
        self.angle = 0

        rospy.Subscriber('/change', Bool, self.callback)

    def callback(self, data):
        q = quaternion_from_euler(0, 0, self.angle)
        new_pose = Pose()
        new_pose.position.x = 0
        new_pose.position.y = 0
        new_pose.position.z = 0.5
        new_pose.orientation.x = q[0]
        new_pose.orientation.y = q[1]
        new_pose.orientation.z = q[2]
        new_pose.orientation.w = q[3]
        self.pub.publish(new_pose)
        print(new_pose)
        self.angle += 0.0031
        if self.angle > 3.1:
            rospy.loginfo("Finished Rotating link!!")
            rospy.signal_shutdown("Finished Rotating link!!")

if __name__ == '__main__':
    change_link_pose = ChangeLinkPose()
    rospy.spin()