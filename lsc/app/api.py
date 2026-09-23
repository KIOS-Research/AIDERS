import threading

# custom libs
import streamCapture
from flask import Flask, jsonify, request

app = Flask(__name__)

# start the http server, called on launch
def start(_port):
    app.run(host='0.0.0.0', port=_port)   
    

# returns the active threads in this app
@app.route('/lsc/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# called when a drone connects or when a CV model starts running
@app.route('/lsc/startDroneStreamCapture', methods=['POST'])
def handleDroneStartStreamCaptureRequest():
    data = request.get_json()
    # print(data)
    streamCapture.startDroneStreamCapture(data["droneId"], data["droneName"]) # start capturing live feed frames
    return "200"


# called when a drone disconnects or a user stops the live feed
@app.route('/lsc/stopDroneStreamCapture', methods=['POST'])
def handleDroneStopStreamCaptureRequest():
    data = request.get_json()
    # print(data)
    streamCapture.stopDroneStreamCapture(data["droneName"]) # stop capturing live feed frames
    return "200"



@app.route('/lsc/healthCheck', methods=['GET'])
def handleHealthCheckRequest():
    response_data = {
        "status": "OK",
        "message": "Health check successful"
    }
    return jsonify(response_data), 200
