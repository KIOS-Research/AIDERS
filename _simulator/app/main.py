import signal
import sys
import os
import rospy
import threading
import random
import time

# custom libs
import utils
from drone import Drone
from device import Device
from lora import LoraMaster
from constants import droneNames, droneModels, deviceNames, deviceModels, loraMasterNames


def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rosIp = os.environ.get("ROS_IP")
    os.environ['ROS_MASTER_URI'] = f"http://{rosIp}:11311"
    utils.myPrint(f"\nStarting simulator on {os.environ['ROS_MASTER_URI']}")
    utils.myPrint(f"\nWaiting for connection with ROS master...")


    numberOfDrones = int(os.environ.get("NUM_DRONES"))
    droneFrequency = float(os.environ.get("DRONE_FREQ"))
    liveStreamActive = int(os.environ.get("DRONE_LIVE_STREAM"))
    numberOfDevices = int(os.environ.get("NUM_DEVICES"))
    deviceFrequency = float(os.environ.get("DEVICE_FREQ"))
    numberOfLoraMasters = int(os.environ.get("NUM_LORA_MASTERS"))
    numberOfLoraClients = int(os.environ.get("NUM_LORA_CLIENTS"))
    loraFrequency = float(os.environ.get("LORA_FREQ"))
    
    rospy.init_node('SIMULATOR')    # initialize a ROS node   
    
    # initialize simulators and run them on a new thread each
    threads = []
    spawnDelay = 0.25
    
    #drones
    if numberOfDrones > 0:
        utils.myPrint("\n")
        for i, _ in enumerate(range(numberOfDrones)):
            ip = ".".join(str(random.randint(0, 255)) for _ in range(4))
            drone = Drone(droneNames[i], droneModels[i], ip, droneFrequency, rosIp, liveStreamActive, i)
            thread = threading.Thread(target=drone.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            threads.append(thread)
            time.sleep(spawnDelay)

    # devices
    if numberOfDevices > 0:
        utils.myPrint("\n")    
        for i, _ in enumerate(range(numberOfDevices)):
            ip = ".".join(str(random.randint(0, 255)) for _ in range(4))
            device = Device(deviceNames[i], deviceModels[i], ip, deviceFrequency, rosIp)
            thread = threading.Thread(target=device.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            threads.append(thread)
            time.sleep(spawnDelay)

    # lora masters
    if numberOfLoraMasters > 0:
        utils.myPrint("\n")    
        for i, _ in enumerate(range(numberOfLoraMasters)):
            loraMaster = LoraMaster(loraMasterNames[i], numberOfLoraClients, loraFrequency, i)
            thread = threading.Thread(target=loraMaster.start)
            thread.daemon = False  # set the thread as non-daemonic
            thread.start()
            threads.append(thread)
            time.sleep(spawnDelay)

    utils.myPrint(f"\nSimulators started:")
    utils.myPrint(f"-------------------")
    utils.myPrint(f"   Drones: {numberOfDrones} ({droneFrequency}Hz)")
    utils.myPrint(f"  Devices: {numberOfDevices} ({deviceFrequency}Hz)")
    utils.myPrint(f"     Lora: {numberOfLoraMasters} x {numberOfLoraClients} ({loraFrequency}Hz)")

    
    rospy.spin() # keep the app running



if __name__ == '__main__':
    main()


