#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf.transformations import quaternion_inverse, quaternion_multiply

class Relay:
    def __init__(self):
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.cb, queue_size=5)
    def cb(self, msg):
        try:
            base = self.buffer.lookup_transform('odom', 'chassis', rospy.Time(0), rospy.Duration(0.3))
        except Exception:
            return
        q = base.transform.rotation
        inv = quaternion_inverse((q.x, q.y, q.z, q.w))
        mq = msg.pose.pose.orientation
        rot = quaternion_multiply((mq.x, mq.y, mq.z, mq.w), inv)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now(); t.header.frame_id = 'map'; t.child_frame_id = 'odom'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rot
        self.pub.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('amcl_tf_relay'); Relay(); rospy.spin()
