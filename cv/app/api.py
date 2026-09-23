import os
import threading

# custom libs
import requestHandler
from flask import Flask, jsonify, request

app = Flask(__name__)

# start the http server, called on launch
def start(_port):
    app.run(host='0.0.0.0', port=_port)   
    

# returns the active threads in this app
@app.route('/cv/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# called when a user requests detection start
@app.route('/cv/startDetection', methods=['POST'])
def handleStartDetectionRequest():
    data = request.get_json()
    print("Detection start requested.")
    print(data)
    if(data["detectionType"] == "DISASTER_CLASSIFICATION"):
        print("Starting disaster classification detection.")
        requestHandler.startDroneDisasterClassification(data)
    elif(data["detectionType"] == "CROWD_LOCALIZATION"):
        print("Starting crowd detection.")
        requestHandler.startDroneCrowdDetector(data)        
    else:
        print("Starting Aiders tracker detection.")
        requestHandler.startDroneStreamTracker(data)
    return "200"


# called when a user requests detection stop
@app.route('/cv/stopDetection', methods=['POST'])
def handleStopDetectionRequest():
    data = request.get_json()
    print("Detection stop requested.")
    print(data)
    requestHandler.stopDetection(data)
    return "200"


@app.route('/cv/healthCheck', methods=['GET'])
def handleHealthCheckRequest():
    return "200"
