#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo, Image
from rtabmap_msgs.msg import RGBDImage

def sub():
    pub=rospy.Publisher()
    rospy.init_node("decompress_msg", RGBDImage, queue=10)

