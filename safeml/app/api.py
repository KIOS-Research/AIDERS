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
    if(data["detectionType"] == "SAFEML"):
        print("Starting safeml detection.")
        requestHandler.startDroneSafeMLDetection(data)
        return "200"
    elif(data["detectionType"] == "DEEPKNOWLEDGE"):
        print("Starting deepknowledge detection.")
        requestHandler.startDroneDeepKnowledgeDetection(data)
        return "200"
    return "400"


# called when a user requests detection stop
@app.route('/stopDetection', methods=['POST'])
def handleStopDetectionRequest():
    data = request.get_json()
    print("Detection stop requested.")
    requestHandler.stopDetection(data)
    return "200"


