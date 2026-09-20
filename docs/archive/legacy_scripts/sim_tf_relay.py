#!/usr/bin/env python3
"""Republish Gazebo odometry TF using simulation-time stamps."""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomTfRelay:
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/my_car/odom', Odometry, self.on_odom, queue_size=50)

    def on_odom(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'chassis'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('sim_tf_relay')
    OdomTfRelay()
    rospy.spin()
