import threading
from flask import Flask, request, jsonify

# custom libs
import streamCapture

app = Flask(__name__)

# start the http server, called on launch
def start(_port):
    app.run(port=_port)   
    

# returns the active threads in this app
@app.route('/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


# called when a drone connects or re-connects
@app.route('/startDroneStreamCapture', methods=['POST'])
def handleDroneStartStreamCaptureRequest():
    data = request.get_json()
    # print(data)
    streamCapture.startDroneStreamCapture(data["droneId"], data["droneName"]) # start capturing live feed frames
    return "200"

# # called when a drone disconnects
# @app.route('/stopDroneStreamCapture', methods=['POST'])
# def handleDroneStopStreamCaptureRequest():
#     data = request.get_json()
#     streamCapture.stopDroneStreamCapture(data["droneId"], data["droneName"]) # stop capturing live feed frames
#     return "200"
