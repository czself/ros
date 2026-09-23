#!/usr/bin/env python3
"""Simulation truth odometry at the drive axle; twist is in body coordinates."""
import math
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_multiply, quaternion_inverse)


class ModelStateOdom:
    def __init__(self):
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=2)
        self.legacy = rospy.Publisher('/my_car/odom', Odometry, queue_size=2)
        self.tf = tf2_ros.TransformBroadcaster()
        self.last = None
        # The embedded competition-world model is scaled: both drive wheels
        # are centred 5.25 cm ahead of the chassis origin.  This must match
        # the world SDF, otherwise base_footprint, sensors and the costmaps
        # describe three different robots.
        self.axle = 0.0525
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback, queue_size=1)

    def callback(self, msg):
        stamp = rospy.Time.now()
        if stamp.to_sec() == 0:
            return
        if self.last is not None and stamp >= self.last and (stamp-self.last).to_sec() < 0.02:
            return
        self.last = stamp  # Accept clock rollback; never fabricate future stamps.
        try:
            i = msg.name.index('my_car')
        except ValueError:
            return
        p, v = msg.pose[i], msg.twist[i]
        q = p.orientation
        yaw = euler_from_quaternion((q.x,q.y,q.z,q.w))[2]
        c, s = math.cos(yaw), math.sin(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = p.position.x + self.axle*c
        odom.pose.pose.position.y = p.position.y + self.axle*s
        flat = quaternion_from_euler(0,0,yaw)
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = flat
        odom.twist.twist.linear.x = c*v.linear.x + s*v.linear.y
        odom.twist.twist.linear.y = -s*v.linear.x + c*v.linear.y + self.axle*v.angular.z
        odom.twist.twist.angular.z = v.angular.z
        self.pub.publish(odom)
        legacy = Odometry()
        legacy.header = odom.header
        legacy.child_frame_id = 'chassis'
        legacy.pose.pose = p
        legacy.twist.twist.linear.x = c*v.linear.x+s*v.linear.y
        legacy.twist.twist.linear.y = -s*v.linear.x+c*v.linear.y
        legacy.twist.twist.angular.z = v.angular.z
        self.legacy.publish(legacy)
        axle = TransformStamped()
        axle.header = odom.header
        axle.child_frame_id = 'base_footprint'
        axle.transform.translation.x = odom.pose.pose.position.x
        axle.transform.translation.y = odom.pose.pose.position.y
        axle.transform.rotation = odom.pose.pose.orientation
        chassis = TransformStamped()
        chassis.header.stamp = stamp
        chassis.header.frame_id = 'base_footprint'
        chassis.child_frame_id = 'chassis'
        chassis.transform.translation.x = -self.axle
        chassis.transform.translation.z = p.position.z
        rel = quaternion_multiply(quaternion_inverse(flat),(q.x,q.y,q.z,q.w))
        (chassis.transform.rotation.x,chassis.transform.rotation.y,
         chassis.transform.rotation.z,chassis.transform.rotation.w)=rel
        self.tf.sendTransform([axle,chassis])


if __name__ == '__main__':
    rospy.init_node('model_state_odom')
    ModelStateOdom()
    rospy.spin()
