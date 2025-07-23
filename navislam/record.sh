#!/bin/bash
# Tạo thư mục lưu nếu chưa tồn tại
mkdir -p ~/bags
cd ~/bags
# Ghi dữ liệu với tên file chứa timestamp
rosbag record /cmd_vel /odom /move_base_simple/goal \
    /move_base/DWAPlannerROS/local_plan \
    /move_base/local_costmap/footprint \
    /move_base/DWAPlannerROS/global_plan \
    /move_base/global_costmap/footprint \
    /map\
    