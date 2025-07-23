#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import time


class CmdVelToArduino:
    def __init__(self):
        # Khởi động node ROS
        rospy.init_node('cmd_vel_to_arduino', anonymous=True)

        # Tìm và kết nối với cổng serial
        self.serial_conn = self.connect_serial()

        if self.serial_conn is None:
            rospy.signal_shutdown("Unable to open any serial port. Shutting down.")
        else:
            rospy.loginfo("Serial connection established successfully.")
            time.sleep(2)  # Chờ Arduino khởi động

        # Đăng ký Subscriber để lắng nghe topic /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def connect_serial(self):
        """Thử kết nối với cổng /dev/ttyUSB1 hoặc /dev/ttyUSB2"""
        ports = ['/dev/ttyUSB1', '/dev/ttyUSB2']
        for port in ports:
            try:
                rospy.loginfo(f"Trying to connect to {port}...")
                serial_conn = serial.Serial(port, 115200, timeout=1)
                rospy.loginfo(f"Connected to {port}")
                return serial_conn
            except serial.SerialException as e:
                rospy.logwarn(f"Failed to connect to {port}: {e}")
        return None

        # Đăng ký Subscriber để lắng nghe topic /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Lấy giá trị vận tốc từ topic /cmd_vel
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        rospy.loginfo(f"{linear_x},{linear_y},{angular_z}")

        # Chuẩn bị dữ liệu gửi xuống Arduino
        data_to_send = f"{linear_x:.2f},{linear_y:.2f},{angular_z:.2f}\n"

        # Gửi dữ liệu qua Serial
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(data_to_send.encode('utf-8'))
            rospy.loginfo(f"Sent to Arduino: {data_to_send.strip()}")
        else:
            rospy.logwarn("Serial connection is not open!")

    def run(self):
        rospy.spin()
        # Đóng kết nối serial khi dừng node
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            rospy.loginfo("Serial connection closed.")



if __name__ == '__main__':
    try:
        node = CmdVelToArduino()
        node.run()
    except rospy.ROSInterruptException:
        pass
