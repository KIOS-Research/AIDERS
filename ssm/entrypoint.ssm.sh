#!/bin/bash
echo "-- Running entrypoint.ssm.sh --"

# Code that is commented is for ConSert
# cd app/ConSertSetup/wheels

# pip install *
# python3 -m pip install --upgrade pip

# cd ../../..

# source the default and custom ROS files
source /opt/ros/noetic/setup.bash
# source app/ConSertSrc/catkin_ws/devel/setup.bash
source /opt/ros_ws/src/ros_msgs/ros_msgs_sinadra/devel/setup.bash

# add the ROS files to the bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# echo "source app/ConSertSrc/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /opt/ros_ws/src/ros_msgs/ros_msgs_sinadra/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


# wait for mysql
while ! nc -z $SQL_HOST $SQL_PORT; do
    sleep 0.1
done

# run sinadra
roslaunch kios_sinadra_scenario_2 eddi_monitor.launch &



# run the python application
echo -e "\n** Running Python main app **"
python3 -u /app/api.py &

sleep 2

echo "-- END entrypoint.ssm.sh --"

# keep the container alive
tail -f /dev/null