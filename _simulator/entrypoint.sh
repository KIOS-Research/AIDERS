#!/bin/bash
echo "--- Running entrypoint.sh ---"

# source the default and custom ROS files
source /opt/ros/noetic/setup.bash
source /opt/ros_ws/devel/setup.bash

# add the ROS files to the bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "--- END entrypoint.sh ---"

python -u /app/main.py

# keep the container alive
tail -f /dev/null