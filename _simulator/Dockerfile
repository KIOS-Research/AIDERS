FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

#  install ROS and python dependencies and other required packages
RUN apt update && apt-get install -y \
    ros-noetic-gps-common \
    ros-noetic-rosbridge-server \
    ros-noetic-fkie-multimaster \
    python3 \
    python3-pip \
    python3-rospy \
    ffmpeg

RUN pip install ffmpeg-python

# Set the default Python version
RUN ln -s /usr/bin/python3 /usr/bin/python    

# Copy custom ROS message definitions
COPY ros_msgs /opt/ros_ws/src/ros_msgs

# Build custom ROS messages
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd /opt/ros_ws && \
    catkin_make

# copy the entrypoint bash script
COPY entrypoint.sh .

# Copy the python application
COPY app /app

# run the entrypoint bash script
ENTRYPOINT ["/bin/bash", "./entrypoint.sh"]

