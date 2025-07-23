#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import serial

class GoalAndOdomLogger:
    def __init__(self):
        # Khởi tạo node ROS
        rospy.init_node('goal_and_odom_logger', anonymous=True)

        # Subscriber cho /move_base_simple/goal và /rtabmap/odom
        self.goal_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        # Biến lưu goal hiện tại
        self.goal_position = None
        self.goal_theta = None  # Góc yaw của goal

        # Khởi tạo kết nối serial với Arduino
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        rospy.loginfo("Serial connection established with Arduino")

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

    def odom_callback(self, msg):
        """Callback cho topic /rtabmap/odom"""
        # Lấy tọa độ và góc từ odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Log dữ liệu odometry
        rospy.loginfo("Odometry: x=%.3f, y=%.3f, theta=%.3f rad", position.x, position.y, yaw)

        # Nếu đã có goal, tính toán sai số và in ra
        if self.goal_position and self.goal_theta is not None:
            ex = self.goal_position[0] - position.x 
            ey = self.goal_position[1] - position.y  
            eth = self.goal_theta - yaw             
            
            # Đưa góc eth về khoảng [-pi, pi]
            while eth > 3.14159:
                eth -= 2 * 3.14159
            while eth < -3.14159:
                eth += 2 * 3.14159

            # Log thông tin sai số
            rospy.loginfo("Error: ex=%.3f, ey=%.3f, eth=%.3f rad",
                          ex, ey, eth)

            # Log lại thông tin goal và odometry để dễ so sánh
            rospy.loginfo("Goal Position: x=%.3f, y=%.3f, theta=%.3f rad",
                          self.goal_position[0], self.goal_position[1], self.goal_theta)
            # rospy.loginfo("Odometry: x=%.3f, y=%.3f, theta=%.3f rad", position.x, position.y, yaw)

            data_to_send = f"{ex:.3f},{ey:.3f},{eth:.3f},{yaw:.3f}\n"
            self.serial_conn.write(data_to_send.encode('utf-8'))
            # rospy.loginfo("Sent to Arduino: %s", data_to_send)

if __name__ == '__main__':
    try:
        GoalAndOdomLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'self.serial_conn' in locals() and self.serial_conn.is_open:
            self.serial_conn.close()
