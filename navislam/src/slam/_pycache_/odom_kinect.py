#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(msg):
    # Lấy tọa độ (position)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # Lấy góc quay (quaternion)
    orientation = msg.pose.pose.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]

    # Chuyển quaternion thành góc Euler (yaw, pitch, roll)
    euler = tf.transformations.euler_from_quaternion(quat)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # In ra vị trí và góc quay
    rospy.loginfo(f"Position: x={x}, y={y}, z={z}")
    rospy.loginfo(f"Orientation: roll={roll}, pitch={pitch}, yaw={yaw}")

def listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
