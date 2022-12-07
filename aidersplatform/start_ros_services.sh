#!/bin/bash
ros_setup_path=${APP_DIR}/django_api/logic/djiswarmcatkinws/devel/setup.bash
source $ros_setup_path
export ROS_IP="$NET_IP"
export ROS_MASTER_URI=http://"${NET_IP}":11311
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 2
roslaunch fkie_master_discovery master_discovery.launch &
sleep 2
roslaunch fkie_master_sync master_sync.launch &

