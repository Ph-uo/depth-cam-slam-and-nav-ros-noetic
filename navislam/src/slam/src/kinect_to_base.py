#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class KinectToBaseLink:
    def __init__(self):
        rospy.init_node('kinect_to_baselink_broadcaster')

        # TF Broadcaster
        self.br = tf.TransformBroadcaster()

        # Subscriber để lấy dữ liệu từ /rtabmap/odom
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        # Vị trí và hướng của base_link (ban đầu gán là gốc tọa độ)
        self.base_position = (0.0, 0.0, 0.0)
        self.base_orientation = (0.0, 0.0, 0.0, 1.0)  # Quaternion mặc định

        # Offset cố định từ base_link tới kinect
        self.offset_kinect = (0.17, 0.0, 0.33)

    def odom_callback(self, msg):
        # Lấy vị trí từ /rtabmap/odom (gắn cho base_link)
        self.base_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # Lấy hướng từ /rtabmap/odom
        self.base_orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def broadcast_transform(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Truyền transform từ base_link tới kinect
            self.br.sendTransform(
                self.offset_kinect,  # Tọa độ cố định từ base_link tới kinect
                (0.0, 0.0, 0.0, 1.0),  # Góc quay cố định (không thay đổi)
                rospy.Time.now(),
                "kinect",       # Khung con
                "base_link"     # Khung cha
            )

            # Truyền transform từ /rtabmap/odom (nếu cần)
            self.br.sendTransform(
                self.base_position,      # Vị trí của base_link trong không gian toàn cục
                self.base_orientation,   # Hướng của base_link trong không gian toàn cục
                rospy.Time.now(),
                "base_link",     # Khung con
                "odom"           # Khung cha (toàn cục)
            )

            rate.sleep()

if __name__ == '__main__':
    try:
        node = KinectToBaseLink()
        node.broadcast_transform()
    except rospy.ROSInterruptException:
        pass
