A web based platform where users can view and interact with drones in real time

## Instructions

1. Clone the project
2. Remove the node_modules folder rm -R node_modules
3. sudo apt-get install glup
   that update the drones' position and layers in real time from ROS
4. Start the platform by npm start

for the drones: 5. Run the ' python python_Drone_API/droneAPI.py' to start a localhost server
for the web api 6. Run the 'python python_LocUpdate_Script/LocUpdate.py' which performs PUT requests, for real time with ROS use LocUpdateRos.py

![screenshot](demoScreen.png)

Server runs on http://localhost:3000
