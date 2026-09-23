import signal
import sys
import os
import threading
import random
import time

# custom libs
import utils
from droneWS import DroneWS
from device import Device
# from lora import LoraMaster
from constants import droneNames, droneNamesROS, droneModels, deviceNames, deviceModels, loraMasterNames

from api import app
from drones_manager import DronesManager



def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    platformIp = os.environ.get("PLATFORM_IP")

    # numberOfDronesROS = int(os.environ.get("NUM_DRONES_ROS"))
    numberOfDronesWS = int(os.environ.get("NUM_DRONES_WS"))
    droneFrequency = float(os.environ.get("DRONE_FREQ"))
    liveStreamActive = int(os.environ.get("DRONE_LIVE_STREAM"))
    numberOfDevices = int(os.environ.get("NUM_DEVICES"))
    deviceFrequency = float(os.environ.get("DEVICE_FREQ"))
    numberOfLoraMasters = int(os.environ.get("NUM_LORA_MASTERS"))
    numberOfLoraClients = int(os.environ.get("NUM_LORA_CLIENTS"))
    loraFrequency = float(os.environ.get("LORA_FREQ"))
    
    # if(numberOfDronesROS > 0 or numberOfDevices > 0 or numberOfLoraMasters > 0):
    #     rospy.init_node('SIMULATOR')    # initialize a ROS node if needed  
    
    # initialize simulators and run them on a new thread each
    threads = []
    spawnDelay = 0.25
    
    #drones
    app.drones_manager = DronesManager()
    dronesCounter = 0

    # # ROS Drones
    # if numberOfDronesROS > 0:
    #     utils.myPrint("\n")
    #     for i, _ in enumerate(range(numberOfDronesROS)):
    #         dronesCounter = dronesCounter + 1
    #         ip = ".".join(str(random.randint(0, 255)) for _ in range(4))
    #         drone = DroneROS(droneNamesROS[dronesCounter-1], droneModels[dronesCounter-1], ip, droneFrequency, rosIp, liveStreamActive, dronesCounter-1)
    #         thread = threading.Thread(target=drone.start)
    #         thread.daemon = False  # set the thread as non-daemonic
    #         thread.start()
    #         threads.append(thread)
    #         app.drones_manager.add_drone(drone)
    #         time.sleep(spawnDelay)

    # Websocket Drones
    if numberOfDronesWS > 0:
        utils.myPrint("\n")
        for i, _ in enumerate(range(numberOfDronesWS)):
            dronesCounter = dronesCounter + 1
            ip = ".".join(str(random.randint(0, 255)) for _ in range(4))
            drone = DroneWS(droneNames[dronesCounter-1], droneModels[dronesCounter-1], ip, droneFrequency, platformIp, liveStreamActive, dronesCounter-1)
            thread = threading.Thread(target=drone.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            threads.append(thread)
            app.drones_manager.add_drone(drone)
            time.sleep(spawnDelay)

    # devices
    if numberOfDevices > 0:
        utils.myPrint("\n")
        for i, _ in enumerate(range(numberOfDevices)):
            ip = ".".join(str(random.randint(0, 255)) for _ in range(4))
            device = Device(deviceNames[i], deviceModels[i], ip, deviceFrequency, platformIp)
            thread = threading.Thread(target=device.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            threads.append(thread)
            time.sleep(spawnDelay)

    # # lora masters
    # if numberOfLoraMasters > 0:
    #     utils.myPrint("\n")
    #     for i, _ in enumerate(range(numberOfLoraMasters)):
    #         loraMaster = LoraMaster(loraMasterNames[i], numberOfLoraClients, loraFrequency, i)
    #         thread = threading.Thread(target=loraMaster.start)
    #         thread.daemon = False  # set the thread as non-daemonic
    #         thread.start()
    #         threads.append(thread)
    #         time.sleep(spawnDelay)

    utils.myPrint(f"\nSimulators started:")
    utils.myPrint(f"-------------------")
    # utils.myPrint(f"   ROS Drones: {numberOfDronesROS} ({droneFrequency}Hz)")
    utils.myPrint(f"   Websocket Drones: {numberOfDronesWS} ({droneFrequency}Hz)")
    utils.myPrint(f"   Devices: {numberOfDevices} ({deviceFrequency}Hz)")
    utils.myPrint(f"   Lora: {numberOfLoraMasters} x {numberOfLoraClients} ({loraFrequency}Hz)")

    app.run(port=8991)   # start the http server


if __name__ == '__main__':
    main()


