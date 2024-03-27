import threading
from flask import Flask, request, jsonify

# custom libs
import utils


app = Flask(__name__)


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
            drones[index].altitude = data['value']
        elif param == 'heading':
            drones[index].heading = data['value']
        elif param == 'gimbalAngle':
            drones[index].gimbalAngle = data['value']
        if param == 'batteryPercentage':
            drones[index].batteryPercentage = data['value']            
        elif param == 'demoAltitude':
            drones[index].demoAltitude = True if data['value'] == 1 else False
        elif param == 'demoHeading':
            drones[index].demoHeading = True if data['value'] == 1 else False
        elif param == 'demoGimbalAngle':
            drones[index].demoGimbalAngle = True if data['value'] == 1 else False
        else:
            utils.myPrint("\nERROR: Unknown parameter")
            return "400"

        utils.myPrint(f"\nDrone '{drones[index].name}' set '{param}' to '{data['value']}'")
    except Exception as e:
        utils.myPrint("\nERROR")
        utils.myPrint(e)
    return "200"
