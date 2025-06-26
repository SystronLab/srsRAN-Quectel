#!/bin/bash

loc="/home/wheeltec/Projects/OranDemos/Demo1"
log=$loc"/robotlog"

echo "launching ROS2 api for ROSBOT..."
echo "launching ROS2 api for ROSBOT..." > $log
source /opt/ros/humble/setup.bash
source /home/wheeltec/yolo_ws/install/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
/opt/ros/humble/bin/ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py &

echo "running Oran Demo 1 ROS application ..."
echo "running Oran Demo 1 ROS application ..." >> $log
sleep 30
source /home/wheeltec/Projects/OranDemos/Demo1/ROS/src/install/setup.bash
/opt/ros/humble/bin/ros2 run demo1_5g_robot_controller robot_controller

