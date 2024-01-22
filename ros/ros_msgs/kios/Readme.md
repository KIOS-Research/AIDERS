# Kios Swarm

Ros nodes for connecting DJI drones to the ros newtork

## Getting Started
Clone the folder from this github in your "catkin_ws" and rename it to "kios" (or name it whatever you want and change package.xml and CmakeLists.txt contents)
Run 
```
cd catkin_ws/src
git clone [THIS GIT] kios
```

Install the Rosbridge server using the following command
```
sudo apt-get install ros-<rosdistro>-rosbridge-server
```

Also download mapviz from source SEE [THIS](https://github.com/swri-robotics/mapviz)
```
git clone https://github.com/swri-robotics/mapviz.git --branch $ROS_DISTRO-devel
rosdep install --from-paths src --ignore-src
```
Install dependencies and catkin_make it
```
rosdep install kios
catkin_make
```
Maybe some more python dependecies need to be installed on runtime for some nodes. (Find them from pipFreeze text file in kios directory)
```
pip install geopy
pip install termcolor
```

### Connect a spark/mavin to PC-Ros.
For each drone: 
+ Connect android to PC using usb and enable USB tethering
+ Connect android to the RC wifi network
+ Using "ipconfig" find the name of the usb-ethernet that was created (eg. enp0s20f0u4)
+ Run this to change set the ip to what you want it to be (Optional)
```
sudo ifconfig enp0s20f0u4 192.168.42.19 netmask 255.255.255.0
```
+ Enable Port Forwarding  (Optional)
```
sudo "echo 1 > /proc/sys/net/ipv4/ip_forward"
sudo iptables -t nat -F
sudo iptables -t nat -A POSTROUTING -j MASQUERADE
```


### Start the Google maps tiles for mapviz
Run
```
sudo docker run -p 8080:8080 -d -t -v ~/catkin_ws/src/kios/data/mapviz-google-map:/mapproxy danielsnider/mapproxy
```
Then start mapviz launcher from kios package
```
roslaunch kios mapviz.launch
```
More info see [this](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite)

(Optional) Add some example configurations to mapviz. In mapviz File->OpenConfig and find them in /kios/data/mapvizConfig.

DEMO: Launcher that starts mapviz and sents a number of coordinates to mapviz and to a topic that is picked up by the android.
```
roslaunch kios demoMapvizGPScoordinates.launch
```

### Run ROS
+ Edit dji_launch with the id and number of drones that are going to be added and run:
```
roslaunch kios dji_launch.launch --screen
```
+ Run GUI to see camera and telemetry (in rviz: see example config in /data) info (if needed):
```
rosrun rqt_gui rqt_gui
