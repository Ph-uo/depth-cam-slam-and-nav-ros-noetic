#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class RGBOdometryToKinectTF:
    def __init__(self):
        rospy.init_node('rgb_odometry_to_kinect_tf', anonymous=True)

        # Khởi tạo broadcaster TF
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Lắng nghe dữ liệu từ topic /rtabmap/rgb_odometry
        rospy.Subscriber('/rtabmap/rgb_odometry', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Lấy vị trí từ dữ liệu Odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Chuyển vị trí và hướng thành định dạng của TF
        translation = (position.x, position.y, position.z)
        rotation = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Phát TF từ baselink tới kinect
        self.tf_broadcaster.sendTransform(
            translation,        # Vị trí
            rotation,           # Hướng
            rospy.Time.now(),   # Thời gian hiện tại
            "kinect",           # Khung đích
            "baselink"          # Khung gốc
        )

if __name__ == "__main__":
    try:
        RGBOdometryToKinectTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
