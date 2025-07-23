#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path  # Import Path tại đây
from tf.transformations import euler_from_quaternion
import serial

class GoalAndOdomLogger:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('arduino_communication', anonymous=True)

        # Subscriber cho /move_base_simple/goal và /move_base/odom
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)   #odometry của xe
        # self.pose_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.pose_callback)  # Đọc dữ liệu từ local plan
        self.pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        rospy.Rate(10)
        
        # Biến lưu goal hiện tại
        self.goal_position = None
        self.goal_theta = None  # Góc yaw của goal
        self.robot_position = None
        self.robot_theta = None

        rospy.loginfo("Node initialized, waiting for messages...")

    def goal_callback(self, msg):
        """Callback cho topic /move_base_simple/goal"""
        # Lưu tọa độ goal
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        # Chuyển đổi quaternion sang Euler để lấy góc yaw (theta)
        orientation = msg.pose.orientation
        _, _, self.goal_theta = euler_from_quaternion([
            orientation.x, 
            orientation.y, 
            orientation.z, 
            orientation.w
        ])

        # Log tọa độ và góc của goal
        rospy.loginfo("Goal Position: x=%.3f, y=%.3f, theta=%.3f rad",
                      self.goal_position[0], self.goal_position[1], self.goal_theta)

    def pose_callback(self, msg):
        """Callback xử lý thông điệp từ topic local_plan"""
        # Kiểm tra nếu msg là một Path (danh sách các PoseStamped)
        if msg.poses:
            # Lấy vị trí của robot từ Pose đầu tiên trong Path
            robot_pose = msg.poses[0]  # Lấy Pose đầu tiên trong local_plan (thường là vị trí gần nhất)
            
            self.robot_position = (robot_pose.pose.position.x, robot_pose.pose.position.y)

            # Chuyển đổi quaternion sang Euler để lấy góc yaw (theta)
            orientation = robot_pose.pose.orientation
            _, _, self.robot_theta = euler_from_quaternion([
                orientation.x, 
                orientation.y, 
                orientation.z, 
                orientation.w
            ])
            
            # In ra vị trí và góc yaw của robot
            rospy.loginfo("Vị trí robot (từ local_plan): (%f, %f)", self.robot_position[0], self.robot_position[1])
            rospy.loginfo("Hướng robot (yaw): %f", self.robot_theta)
        
    def odom_callback(self, msg):
        """Callback cho topic /rtabmap/odom"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Log dữ liệu odometry
        rospy.loginfo("Odometry: x=%.3f, y=%.3f, theta=%.3f rad", position.x, position.y, yaw)

if __name__ == '__main__':
    try:
        GoalAndOdomLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass