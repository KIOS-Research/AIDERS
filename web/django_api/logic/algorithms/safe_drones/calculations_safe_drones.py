import inspect
import os
import sys
import threading
import time
from datetime import datetime
from sys import platform

import logic.algorithms.safe_drones.SafeDrones as SafeDrones
from aiders import models

if platform == "linux" or platform == "linux2":
    currentdir = os.path.dirname(os.path.realpath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)
elif platform == "win32":
    # in Windows, the first path entry contains the directory where the notebook is
    sys.path.insert(0, os.path.dirname(sys.path[0]))


stopRunning = False

starttime = time.time()


def startCalculatingMotorFailureRisk(frequency):
    global stopRunning

    print(datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES PROCESSING THREAD STARTED", flush=True)

    while not stopRunning:
        start_time = time.time()
        allRisks = []
        eval = SafeDrones.SafeDrones()
        connected_drones = models.Drone.objects.filter(is_connected_with_platform=True)
        # print(connected_drones, flush=True)
        
        print(datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES CALCULATING...", flush=True)

        for drone in connected_drones:

            droneID = drone.id
            droneName = drone.drone_name

            # get latest telemetry for Drone
            telemetry = models.Telemetry.objects.filter(drone_id=droneID).last()
            secondsOn = telemetry.secondsOn

            motorPfail,motorMttf = eval.Motor_Failure_Risk_Calc(MotorStatus=[1,1,1,1], Motors_Configuration='PNPN', Motors_Lambda=0.00001, time=secondsOn)
            motorPfail, motorMttf = float(motorPfail), float(motorMttf)

            batteryPfail,batteryMttf = eval.Battery_Failure_Risk_Calc(BatteryLevel=telemetry.battery_percentage, Lambda=0.000001, Battery_degradation_rate=0.000064, time=secondsOn)
            batteryPfail, batteryMttf = float(batteryPfail), float(batteryMttf)

            droneMonitoringData = models.ControlDevice.objects.filter(drone_id=droneID).last() # get the values from controldevice table
            if droneMonitoringData is not None:
                cpuUsage = round(droneMonitoringData.cpu_usage / 100, 2)
                cpuTemp = droneMonitoringData.cpu_temp
            else:
                cpuUsage = 1
                cpuTemp = 40

            chipPfail,chipMTTF = eval.Chip_MTTF_Model(Ta=cpuTemp, u=cpuUsage, time=secondsOn)
            chipPfail, chipMTTF = float(chipPfail) , float(chipMTTF)

            gpsPfail,gpsMTTF = eval.GPS_Failure_Risk_Calc(SatStatus=telemetry.satellites, Lambda=0.000005, time=secondsOn)
            gpsPfail, gpsMTTF = float(gpsPfail) , float(gpsMTTF)


            dt_string = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            dt = datetime.strptime(dt_string, "%Y-%m-%d %H:%M:%S")

            result = models.SafeDroneResults(
                drone_id=droneID,
                motorPfail=motorPfail,
                motorMTTF=motorMttf,
                batteryPfail=batteryPfail,
                batteryMTTF=batteryMttf,
                chipPfail=chipPfail,
                chipMTTF=chipMTTF,
                gpsPfail=gpsPfail,
                gpsMTTF=gpsMTTF,
                Seconds=secondsOn,
                DateTime=dt
            )
            result.save()
                
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"{datetime.now().strftime('%H:%M:%S')} * SAFE DRONES RESULTS CALCULATED IN {round(elapsed_time, 1)} seconds.", flush=True)
        if stopRunning:
            break
    
        time.sleep(frequency - ((time.time() - starttime) % frequency))


    print(datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES PROCESSING THREAD STOPPED", flush=True)




def start():
    global stopRunning 
    stopRunning = False
    threadName = "SafeDronesThread"
    if not is_thread_running(threadName):
        calculations_thread = threading.Thread(target=startCalculatingMotorFailureRisk, args=(10,))
        calculations_thread.name = threadName
        calculations_thread.start()
        # time.sleep(3)
    else:
        # print("SAFE DRONES THREAD IS ALREADY RUNNING")
        pass

def stop():
    global stopRunning 
    stopRunning = True


def is_thread_running(thread_name):
    for thread in threading.enumerate():
        if thread.name == thread_name:
            return True
    return False