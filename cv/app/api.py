import os
import threading
from flask import Flask, request, jsonify

# custom libs
import requestHandler

app = Flask(__name__)

# start the http server, called on launch
def start(_port):
    app.run(port=_port)   
    

# returns the active threads in this app
@app.route('/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# called when a user requests detection start
@app.route('/startDetection', methods=['POST'])
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
@app.route('/stopDetection', methods=['POST'])
def handleStopDetectionRequest():
    data = request.get_json()
    print("Detection stop requested.")
    print(data)
    requestHandler.stopDetection(data)
    return "200"


