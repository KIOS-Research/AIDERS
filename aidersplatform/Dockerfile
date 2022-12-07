#================= BASE IMAGE=================================
#FROM ros:noetic-ros-base as apt_dependencies
FROM georgesofro/kios:cuda_opencv_torch_07_Jun_2022 as apt_dependencies
	
# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV APP_DIR='/usr/src/aidersplatform'
WORKDIR ${APP_DIR}

# We use this mode when we need zero interaction (user input) while installing or upgrading the system via apt		
ENV DEBIAN_FRONTEND=noninteractive 

#  install ros dependencies and other required packages
RUN     apt update && apt-get install -y \
        binutils libproj-dev gdal-bin \
        ros-noetic-ros-base \
	ros-noetic-rosbridge-server \	
	ros-noetic-gps-common \	
	ros-noetic-fkie-multimaster \	
	lsof \	
	python3-pip \	
	ros-noetic-catkin \
	netcat \
	python3-pip \
	python3.8-tk \
	net-tools \
	&& rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt && apt-get clean

ENV CPLUS_INCLUDE_PATH=/usr/include/gdal
ENV C_INCLUDE_PATH=/usr/include/gdal



#=============== PYTHON DEPENDENCIES ================================
FROM  apt_dependencies as python_dependencies

COPY requirements.txt .
RUN pip install -r requirements.txt --ignore-installed


#================ COPY FILES STAGE ==================================
FROM  python_dependencies as copy_files
ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache
COPY . .


#=============== BUILD ROS ==========================================
FROM copy_files as build_ros_package

#This avoids creating the __pycache__ files in the source tree directory	
ENV PYTHONPYCACHEPREFIX="$HOME/.cache/cpython/"
	




# Build the ros workspace that has the custom ros messages
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash; \	
		cd /usr/src/aidersplatform/django_api/logic/djiswarmcatkinws; \	
		catkin_make;\	
		source /usr/src/aidersplatform/django_api/logic/djiswarmcatkinws/devel/setup.bash;'

RUN ["chmod", "+x", "/usr/src/aidersplatform/entrypoint.sh"]

# Build the plugin needed for the detection
RUN /bin/bash -c 'cd /usr/src/aidersplatform/django_api/logic/algorithms/object_detection/src/plugins/; \	
		make clean; \	
		make;'	

#============== RUN FINAL IMAGE =====================================
FROM build_ros_package as run
ENTRYPOINT ["/usr/src/aidersplatform/entrypoint.sh"]











