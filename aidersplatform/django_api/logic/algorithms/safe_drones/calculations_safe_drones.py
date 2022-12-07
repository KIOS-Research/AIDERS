import time
from datetime import datetime
import sys
import requests
import json
from sys import platform
import inspect
import os
from aiders import models,serializers
from inspect import getsourcefile
from os.path import abspath
if platform == "linux" or platform == "linux2":
    currentdir = os.path.dirname(os.path.realpath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)
elif platform == "win32":
    # in Windows, the first path entry contains the directory where the notebook is
    sys.path.insert(0, os.path.dirname(sys.path[0]))



starttime = time.time()

# from .SafeDrones import Safe_Drones
def startCalculatingMotorFailureRisk(frequency):
    global FROM_DOCKER
    if (FROM_DOCKER):
        print("WILL TRY TO IMPORT IT")
        import SafeDrones
    else:
        import importlib.util

        currentWorkingDir = os.path.abspath(os.path.join(abspath(getsourcefile(lambda: 0)), os.pardir))
        path = currentWorkingDir + '/SafeDrones.py'
        spec = importlib.util.spec_from_file_location("module.name", path)
        SafeDrones = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(SafeDrones)

    global API_URL
    while True:
        # response = requests.request("GET", "http://172.20.81.33:5000/operation_code0/drones/", headers={'Content-Type': 'application/json'})
        # connected_drones = response.json()['drones']
        # drones = drones['drones']
        allRisks = []
        eval = SafeDrones.Safe_Drones()
        connected_drones = models.Drone.objects.filter(is_connected_with_platform=True)
        for drone in connected_drones:
            droneID = drone.drone_name
            seconds = round((time.time() - starttime),2)

            motorPfail,motorMttf = eval.Motor_Failure_Risk_Calc([1,1,1,1,1,1], 'PPNNPN', 0.001, drone['properties']['secondsOn'])
            motorPfail, motorMttf = float(motorPfail), float(motorMttf)

            batteryPfail,batteryMttf = eval.Battery_Failure_Risk_Calc(3,drone['properties']['secondsOn'])
            batteryPfail, batteryMttf = float(batteryPfail), float(batteryMttf)

            chipPfail,chipMTTF = eval.Chip_MTTF_Model(400, 30, 50, 1, 1, drone['properties']['secondsOn'])
            chipPfail, chipMTTF = float(chipPfail) , float(chipMTTF)

            dt_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            r = {
                'DroneID': droneID,
                'motorPfail': motorPfail,
                'motorMTTF': motorMttf,
                'batteryPfail': batteryPfail,
                'batteryMTTF': batteryMttf,
                'chipPfail': chipPfail,
                'chipMTTF': chipMTTF,
                'DateTime': dt_string,
                'Seconds':drone['properties']['secondsOn']
            }
            # r = {'DroneID': droneID, 'P_FAIL': motorPfail, 'MTTF': motorMttf, 'DateTime': dt_string, 'Seconds':drone['properties']['secondsOn']}

            allRisks.append(r)
        # print(allRisks)
        if len(allRisks) > 0:
            requests.request("POST", API_URL + "/motor_failure_risk", headers={'Content-Type': 'application/json'}, data=json.dumps(allRisks))
        time.sleep(frequency - ((time.time() - starttime) % frequency))
def main():
    global API_URL,FROM_DOCKER
    if (len(sys.argv) < 3):
        print("\nPlease make sure you provide the following arguments:"
              "\n1)API URL"
              "\n2)FROM_DOCKER"
              )
        exit()
    else:
        API_URL = sys.argv[1]
        FROM_DOCKER = True if sys.argv[2] == "True" else False
    print("Motor Failure Risk calculator script has started! Waiting for drones to join...")

    startCalculatingMotorFailureRisk(frequency=1)




if __name__ == '__main__':

    main()

