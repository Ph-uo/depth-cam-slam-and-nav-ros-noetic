#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import math
from tf.transformations import quaternion_from_euler


class MecanumTfPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("mecanum_tf_publisher", anonymous=True)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Robot state (pose and orientation)
        self.x = 0.0  # X position in odom
        self.y = 0.0  # Y position in odom
        self.theta = 0.0  # Orientation (yaw) in odom

        # Velocities
        self.vx = 0.0  # Linear velocity in x (m/s)
        self.vy = 0.0  # Linear velocity in y (m/s)
        self.omega = 0.0  # Angular velocity around z (rad/s)

        # Previous time for integration
        self.last_time = rospy.Time.now()

        # Subscribing to velocity or odometry topics
        rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self.cmd_vel_callback)

        # Update rate
        self.rate = rospy.Rate(50)  # 50 Hz

    def cmd_vel_callback(self, msg):
        """
        Callback to update velocities from a Twist message.
        :param msg: Twist message containing vx, vy, omega
        """
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

    def publish_tf(self):
        """
        Publishes the transform from 'odom' to 'baselink' based on the robot's odometry.
        """
        while not rospy.is_shutdown():

            # Current time for integration
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # Update pose based on velocities
            self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            self.theta += self.omega * dt

            # Normalize theta to keep it within [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Create and broadcast the transform
            odom_to_baselink = geometry_msgs.msg.TransformStamped()
            odom_to_baselink.header.stamp = current_time
            odom_to_baselink.header.frame_id = "odom"
            odom_to_baselink.child_frame_id = "base_link"

            # Translation
            odom_to_baselink.transform.translation.x = self.x
            odom_to_baselink.transform.translation.y = self.y
            odom_to_baselink.transform.translation.z = 0.0  # Flat ground

            # Rotation (yaw -> quaternion)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta) 
            odom_to_baselink.transform.rotation.x = qx
            odom_to_baselink.transform.rotation.y = qy
            odom_to_baselink.transform.rotation.z = qz
            odom_to_baselink.transform.rotation.w = qw

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(odom_to_baselink)

            # Sleep to maintain the update rate
            self.rate.sleep()


if __name__ == "__main__":
    try:
        mecanum_tf_publisher = MecanumTfPublisher()
        mecanum_tf_publisher.publish_tf()
    except rospy.ROSInterruptException:
        pass
