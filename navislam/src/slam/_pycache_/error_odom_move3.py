#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path  # Import Path tại đây
from tf.transformations import euler_from_quaternion
import serial
import time

class GoalAndOdomLogger:
    def __init__(self):
        try:
            # Mở kết nối serial
            self.serial_conn = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("Serial connection established with Arduino on /dev/ttyUSB0")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port /dev/ttyUSB0: {e}")
            self.serial_conn = None
            rospy.signal_shutdown("Unable to open serial port. Shutting down.")
            return

        # Khởi tạo node ROS
        rospy.init_node('arduino_communication', anonymous=True)

        # Subscriber cho /move_base_simple/goal và /odom
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)   # Odometry của xe
        self.pose_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.pose_callback)  # Đọc dữ liệu từ local plan
        self.pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        
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

        # Nếu đã có goal, tính toán sai số và gửi dữ liệu qua serial
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
            # rospy.loginfo("Error: ex=%.3f, ey=%.3f, eth=%.3f rad", ex, ey, eth)
        else:
            ex=0.00
            ey=0.00
            eth=0.00
            print(333)
            # Gửi dữ liệu sai số qua serial

        data_to_send = f"{ex},{ey},{eth},{yaw}\n"
        # data_to_send = f"{ex:.3f},{ey:.3f},{eth:.3f},{yaw:.3f}\n"
        self.serial_conn.write(data_to_send.encode('utf-8'))


    # def imu_data(self):
    #     """Đọc dữ liệu IMU từ serial và trả về message Imu"""
    #     imu_msg = Imu()
    #     try:
    #         line = self.serial_conn.readline().decode('utf-8').strip()  # Đọc một dòng từ serial
    #         if '>' in line or not line:  # Check if the line contains '>' or is empty
    #             rospy.loginfo("Skipping invalid line: %s", line)
    #             return None
    #         data = [float(value) for value in line.split(',')]  # Chuyển đổi thành danh sách float

    #         if len(data) != 10:  # Kiểm tra dữ liệu có đủ 10 giá trị không
    #             rospy.logwarn("Invalid IMU data received, defaulting to zeros.")
    #             data = [0.0] * 10

    #         imu_msg.header.stamp = rospy.Time.now()
    #         imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = data[:4]
    #         imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = data[4:7]
    #         imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = data[7:10]

    #         rospy.loginfo("IMU Data - Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
    #                       imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
    #         rospy.loginfo("IMU Data - Acceleration: x=%.3f, y=%.3f, z=%.3f", 
    #                       imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)
    #         rospy.loginfo("IMU Data - Velocity: x=%.3f, y=%.3f, z=%.3f",
    #                       imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)
                          
    #     except serial.SerialException as e:
    #         rospy.logerr(f"Serial communication error: {e}")
    #         return None
    #     return imu_msg

    def run(self):
        """Vòng lặp chính để đọc dữ liệu và publish"""
        if not self.serial_conn:
            rospy.logerr("Serial connection not initialized. Exiting.")
            return

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            imu_value = self.imu_data()  # Đọc dữ liệu IMU
            if imu_value:  # Chỉ publish nếu IMU data hợp lệ
                # rospy.loginfo("Publishing IMU data")
                self.pub.publish(imu_value)
            rate.sleep()


if __name__ == '__main__':
    try:
        start = GoalAndOdomLogger()
        # if start.serial_conn:  # Chỉ chạy nếu kết nối serial thành công
            # start.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'start' in locals() and start.serial_conn and start.serial_conn.is_open:
            start.serial_conn.close()