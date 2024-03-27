from flask import Flask, jsonify, request
import threading

# custom libs
import ros.publishers
import ros.subscribers

app = Flask(__name__)

def start(_port):
    app.run(port=_port)   # start the http server
    

# returns the active threads in this app
@app.route('/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# receive a mission and publish it to a drone
@app.route('/droneMission', methods=['POST'])
def handleDroneMission():
    data = request.get_json()
    print(" ")
    print(data)
    ros.publishers.publishMission(data) # send mission to drone through ROS
    return "200"


# receive a start or stop command for build map and publish it to a drone
@app.route('/droneStartOrStopBuildMap', methods=['POST'])
def handleStartOrStopBuildMap():
    data = request.get_json()
    ros.publishers.startOrStopBuildMap(data["droneName"], data["command"], data["overlap"]) # send command to drone through ROS
    return "200"


# request to open to water sampler valve
@app.route('/droneOpenWaterSamplingValve', methods=['POST'])
def handleOpenWaterSamplingValve():
    data = request.get_json()
    ros.publishers.openWaterSamplingValve(data["droneName"]) # send command to drone through ROS
    return "200"


# receive a start or stop command for lidar and publish it to a drone
@app.route('/droneStartOrStopLidar', methods=['POST'])
def handleStartOrStopLidar():
    data = request.get_json()
    ros.publishers.startOrStopLidar(data["droneName"], data["command"]) # send command to drone through ROS
    if data["command"] == "START":
        ros.subscribers.createDroneLidarSubscriber(data["droneId"], data["droneName"], data["lidarSessionId"]) # create ROS subscriber
    else:
        ros.subscribers.stopDroneLidarSubscriber(data["droneName"]) # stop ROS subscriber
    return "200"


# TODO: temp
@app.route('/test/<string:droneName>', methods=['GET'])
def test(droneName):
    ros.subscribers.stopDroneSubscribers(droneName)
    return "200"

