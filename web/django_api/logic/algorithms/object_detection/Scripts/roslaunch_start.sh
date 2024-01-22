#!/bin/bash

roslaunch rosbridge_server rosbridge_websocket.launch &

echo $! >> rosbagPID.txt

