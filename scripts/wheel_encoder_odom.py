#!/usr/bin/env python3
"""Expose Gazebo's wheel encoder integration to the ROS navigation stack.

The differential-drive plugin must use odometrySource=encoder,
publishTf=true (enables its odometry topic), and publishOdomTF=false.
Only this node owns odom -> base_footprint.  It never reads model/link states.

gazebo_ros_diff_drive 2.9.3 computes encoder linear speed with sqrt(dx²+dy²),
which loses the sign while reversing.  Derive the signed body velocity from
successive encoder-integrated poses before giving the message to DWA.
"""
import copy
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def planar_yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z
                             + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def signed_velocity(previous, current, dt):
    """Return forward and yaw velocity from two (x, y, yaw) encoder poses."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('encoder interval must be positive and finite')
    if not all(math.isfinite(value) for value in previous + current):
        raise ValueError('encoder pose must be finite')
    turn = math.atan2(math.sin(current[2] - previous[2]),
                      math.cos(current[2] - previous[2]))
    heading = previous[2] + 0.5 * turn
    forward = ((current[0] - previous[0]) * math.cos(heading)
               + (current[1] - previous[1]) * math.sin(heading)) / dt
    return forward, turn / dt


class WheelEncoderOdom:
    def __init__(self):
        self.last = None
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=5)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Geometry of the loaded competition world: axle 5.25 cm ahead of
        # chassis; the ground deck is 7 cm thick and centred on world z=0.
        # These fixed dimensions do not use simulator pose observations.
        chassis = TransformStamped()
        chassis.header.stamp = rospy.Time.now()
        chassis.header.frame_id = 'base_footprint'
        chassis.child_frame_id = 'chassis'
        chassis.transform.translation.x = -float(
            rospy.get_param('~drive_axle_offset', 0.0525))
        chassis.transform.translation.z = float(
            rospy.get_param('~chassis_height', 0.035))
        chassis.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(chassis)
        self.subscriber = rospy.Subscriber(
            rospy.get_param('~input_topic', '/my_car/wheel_odom'),
            Odometry, self.callback, queue_size=5, tcp_nodelay=True)
        rospy.loginfo('navigation odometry uses wheel encoders; '
                      'AMCL must publish map -> odom')

    def callback(self, message):
        if (message.header.frame_id.lstrip('/') != 'odom'
                or message.child_frame_id.lstrip('/') != 'base_footprint'):
            rospy.logerr_throttle(5.0, 'unexpected wheel odometry frames: %s -> %s',
                                  message.header.frame_id, message.child_frame_id)
            return
        stamp = message.header.stamp
        if stamp.to_sec() <= 0.0:
            return
        pose = message.pose.pose
        sample = (pose.position.x, pose.position.y, planar_yaw(pose.orientation))
        if not all(math.isfinite(value) for value in sample):
            rospy.logerr_throttle(5.0, 'discarding non-finite wheel odometry')
            self.last = None
            return
        previous = self.last
        self.last = (stamp, sample)
        # Do not invent an initial velocity; wait for the next wheel sample.
        if previous is None:
            return
        dt = (stamp - previous[0]).to_sec()
        if dt <= 0.0:
            rospy.logwarn_throttle(5.0, 'wheel odometry time reset/duplicate; '
                                   'waiting for the next sample')
            return
        forward, yaw_rate = signed_velocity(previous[1], sample, dt)
        odometry = copy.deepcopy(message)
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_footprint'
        odometry.twist.twist.linear.x = forward
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = yaw_rate
        self.publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('wheel_encoder_odom')
    WheelEncoderOdom()
    rospy.spin()
