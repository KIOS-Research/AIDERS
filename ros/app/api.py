import os
import threading

# custom libs
import ros.publishers
import ros.subscribers
from flask import Flask, jsonify, request

app = Flask(__name__)

ALLOWED_IPS=[
    os.environ.get("WEB_IP"),
    "127.0.0.1"
]

def start(_port):
    app.run(host='0.0.0.0', port=_port)   # start the http server

@app.before_request
def restrict_access():
    clientIp = request.remote_addr
    if clientIp not in ALLOWED_IPS:
        return "Access denied. Your IP address is not allowed.", 403

# returns the active threads in this app
@app.route('/ros/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# receive a mission and publish it to a drone
@app.route('/ros/droneMission', methods=['POST'])
def handleDroneMission():
    data = request.get_json()
    print(" ")
    print(data)
    ros.publishers.publishMission(data) # send mission to drone through ROS
    return "200"


# receive a start or stop command for build map and publish it to a drone
@app.route('/ros/droneStartOrStopBuildMap', methods=['POST'])
def handleStartOrStopBuildMap():
    data = request.get_json()
    ros.publishers.startOrStopBuildMap(data["droneName"], data["command"], data["overlap"]) # send command to drone through ROS
    return "200"


# request to open to water sampler valve
@app.route('/ros/droneOpenWaterSamplingValve', methods=['POST'])
def handleOpenWaterSamplingValve():
    data = request.get_json()
    ros.publishers.openWaterSamplingValve(data["droneName"]) # send command to drone through ROS
    return "200"


# receive a start or stop command for lidar and publish it to a drone
@app.route('/ros/droneStartOrStopLidar', methods=['POST'])
def handleStartOrStopLidar():
    data = request.get_json()
    ros.publishers.startOrStopLidar(data["droneName"], data["command"]) # send command to drone through ROS
    if data["command"] == "START":
        ros.subscribers.createDroneLidarSubscriber(data["droneId"], data["droneName"], data["lidarSessionId"]) # create ROS subscriber
    else:
        ros.subscribers.stopDroneLidarSubscriber(data["droneName"]) # stop ROS subscriber
    return "200"


@app.route('/ros/healthCheck', methods=['GET'])
def handleHealthCheckRequest():
    response_data = {
        "status": "OK",
        "message": "Health check successful"
    }
    return jsonify(response_data), 200