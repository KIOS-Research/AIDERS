#!/bin/bash
echo "-- Running entrypoint.ros.sh --"

export ROS_IP="$NET_IP"
export ROS_MASTER_URI=http://"${ROS_IP}":11311

echo "ROS MASTER URI: ${ROS_MASTER_URI}"

# source the default and custom ROS files
source /opt/ros/noetic/setup.bash
source /opt/ros_ws/devel/setup.bash

# add the ROS files to the bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


# start ROS services
roscore &
sleep 2
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 2
roslaunch fkie_master_discovery master_discovery.launch &
sleep 2
roslaunch fkie_master_sync master_sync.launch &
sleep 2

rosrun rosserial_python serial_node.py /dev/loraReceiver 2> /dev/null &
sleep 2

# wait for mysql
while ! nc -z $SQL_HOST $SQL_PORT; do
    sleep 0.1
done

# run the python application
echo -e "\n** Running Python main app **"
python -u /app/main.py &

sleep 2

# start the HTTP server
# echo -e "\n** Starting the HTTP server **"
# cd app && gunicorn -w 5 -b 0.0.0.0:8765 server:app &


echo "-- END entrypoint.ros.sh --"

# keep the container alive
tail -f /dev/null