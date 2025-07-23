#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import serial
import time

# Khởi tạo kết nối Serial
arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, timeout=1)  # Chỉnh cổng phù hợp
time.sleep(2)  # Chờ Arduino sẵn sàng
class GoalAndOdomLogger:
    def __init__(self):
        try:
            self.serial_conn = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            rospy.loginfo("Serial connection established with Arduino on /dev/ttyUSB0")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port /dev/ttyUSB0: {e}")
            self.serial_conn = None
            rospy.signal_shutdown("Unable to open serial port. Shutting down.")
            return

        rospy.init_node('arduino_communication', anonymous=True)
        # self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.pose_callback)

        self.goal_position = None
        self.goal_theta = None
        self.robot_position = None
        self.robot_theta = None

        rospy.loginfo("Node initialized, waiting for messages...")

    def goal_callback(self, msg):
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        orientation = msg.pose.orientation
        _, _, self.goal_theta = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        rospy.loginfo("Goal Position: x=%.10f, y=%.10f, theta=%.10f rad",
                      self.goal_position[0], self.goal_position[1], self.goal_theta)
        
    def pose_callback(self, msg):
        if msg.poses:
            robot_pose = msg.poses[0]
            self.robot_position = (robot_pose.pose.position.x, robot_pose.pose.position.y)
            orientation = robot_pose.pose.orientation
            _, _, self.robot_theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            rospy.loginfo("Vị trí robot (từ local_plan): (%f, %f,%f)", self.robot_position[0], self.robot_position[1],self.robot_theta)

    def odom_callback(self, msg):
        rospy.loginfo("Đã nhận dữ liệu từ /rtabmap/odom")

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        if self.robot_position is None or self.robot_theta is None:
            rospy.logwarn("Robot position or theta is not available yet. Waiting for data...")
            return

        ex = self.robot_position[0] - position.x
        ey = self.robot_position[1] - position.y
        eth = self.robot_theta - yaw


        rospy.loginfo("Odom: x=%.10f, y=%.10f, theta=%.10f rad", position.x, position.y, yaw)
        rospy.loginfo("Sai số: Δx=%.10f, Δy=%.10f, Δθ=%.10f rad", ex, ey, eth)

    # Gửi dữ liệu qua Serial nếu kết nối mở
        if self.serial_conn and self.serial_conn.is_open:
            data_to_send = f"{ex:.10f},{ey:.10f},{eth:.10f}\n"
            self.serial_conn.write(data_to_send.encode('utf-8'))
        else:
            rospy.logwarn("Serial connection is not open!")

def send_data(ex, ey, eth):
    # Gửi cả 3 giá trị trong 1 dòng, cách nhau bằng dấu phẩy
    data_to_send = f"{ex},{ey},{eth}\n"
    arduino.write(data_to_send.encode())
    print(f"Gửi: {data_to_send.strip()}")

def receive_data():
    if arduino.in_waiting > 0:  # Kiểm tra nếu có dữ liệu sẵn
        data = arduino.readline().decode().strip()  # Đọc một dòng và loại bỏ ký tự xuống dòng
        print(f"Nhận: {data}")
        return data
    return None

if __name__ == '__main__':
    try:
        start = GoalAndOdomLogger()
        # if start.serial_conn:
        #     start.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'start' in locals() and start.serial_conn and start.serial_conn.is_open:
            start.serial_conn.close()
try:
    while True:
        # Gửi dữ liệu xuống Arduino
        send_data(ex, ey, eth)
        # Nhận phản hồi từ Arduino
        response = receive_data()
        if response:
            print(f"Phản hồi từ Arduino: {response}")

        time.sleep(1)  # Điều chỉnh tốc độ gửi (0.5 giây/lần)

except KeyboardInterrupt:
    print("Kết thúc chương trình")
finally:
    arduino.close()