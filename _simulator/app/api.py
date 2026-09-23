import threading
from flask import Flask, request, jsonify
import os

# custom libs
import utils

from crisisClassification import CrisisClassification


app = Flask(__name__)

# if kafka is local, get KAFKA_LOCAL_IP instead of KAFKA_IP
if os.environ.get("KAFKA_LOCAL", "0") == "1":
    kafkaIp = os.environ.get("KAFKA_LOCAL_IP")
else:
    kafkaIp = os.environ.get("KAFKA_IP")

kafkaHost = kafkaIp + ":" + os.environ.get("KAFKA_PORT")

kafkaCrisisClassificationTopic = os.environ.get("KAFKA_CRISIS_CLASSIFICATION_TOPIC")

cc = CrisisClassification(kafkaHost, kafkaCrisisClassificationTopic)


# returns the active threads in this app
@app.route('/threads', methods=['GET'])
def getThreads():
    threadList = [{'name': t.name} for t in threading.enumerate() if not t.name.startswith(('Thread-', 'Dummy-'))]
    return jsonify(threadList)


@app.route('/setDroneParameter', methods=['POST'])
def handleDroneStartStreamCaptureRequest():
    data = request.get_json()
    index = data['droneIndex']-1
    param = data['parameter']
    val = data['value']
    drones = app.drones_manager.drones

    print(data)
    try:
        if param == 'altitude':
            drones[index].altitude = val
        elif param == 'heading':
            drones[index].heading = val
        elif param == 'gimbalAngle':
            drones[index].gimbalAngle = val
        elif param == 'batteryPercentage':
            drones[index].batteryPercentage = val            
        elif param == 'demoAltitude':
            drones[index].demoAltitude = True if val == 1 else False
        elif param == 'demoHeading':
            drones[index].demoHeading = True if val == 1 else False
        elif param == 'demoGimbalAngle':
            drones[index].demoGimbalAngle = True if val == 1 else False
        else:
            utils.myPrint("\nERROR: Unknown parameter")
            return "400"

        utils.myPrint(f"\nDrone '{drones[index].name}' set '{param}' to '{val}'")
    except Exception as e:
        utils.myPrint("\nERROR")
        utils.myPrint(e)
    return "200"


@app.route('/generateCrisisIncident', methods=['POST'])
def generateCrisisIncident():
    data = request.get_json()
    incident_id = data.get('id', None)
    incident_type = data.get('type', None)
    severity_level = data.get('severity', None)

    if incident_id and incident_type and severity_level:
        cc.generateCrisisIncident(incident_id, incident_type, severity_level)
        return jsonify({"status": "success", "message": "Crisis incident generated"}), 200
    else:
        return jsonify({"status": "error", "message": "Missing parameters"}), 400