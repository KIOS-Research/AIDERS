import math
import os
import time
import rospy

# custom libs
import ros.callbacks
import database.queries

# ROS messages
from std_msgs.msg import String, Bool
from kios.msg import Telemetry, TelemetryDevice, TerminalHardware, MissionDji, BuildMap
from trisonica_ros.msg import trisonica_msg


runningSubscribers = {}


###################################
############## CORE ###############
###################################


# create ROS subscribers that listen for clients requesting to connect/disconnect
# called on application start-up
def createCoreSubscribers():
    os.environ['ROS_MASTER_URI'] = f"http://{os.environ['NET_IP']}:11311" # get from .env
    rospy.init_node('AIDERS_MAIN')   # initialize the platform's ROS node
    rospy.Subscriber("/droneIds", String, ros.callbacks.droneConnectedOrDisconnected)
    rospy.Subscriber("/deviceIds", String, ros.callbacks.deviceConnectedOrDisconnected)
    rospy.Subscriber("/loraIds", String, ros.callbacks.loraMasterConnectedOrDisconnected)
    print("ROS subscribers initialized. Waiting for clients to connect.\n")
    rospy.spin()    # keep the app running


#####################################
############## DRONES ###############
#####################################


# create subscribers for a newly connected drone
def createDroneSubscribers(_id, _name):
    createSubscriber(f"/{_name}/Telemetry", Telemetry, ros.callbacks.droneTelemetryReceived, (time.time(), _id))
    createSubscriber(f"/{_name}/Error", String, ros.callbacks.droneErrorReceived, (_id, ))
    createSubscriber(f"/{_name}/WeatherStation", trisonica_msg, ros.callbacks.droneWeatherDataReceived, (_id, ))
    createSubscriber(f"/{_name}/TerminalHardware", TerminalHardware, ros.callbacks.droneMonitoringDataReceived, (_id, ))
    createSubscriber(f"/{_name}/crps/initiate_collaboration_request/Get_State_Enable", Bool, ros.callbacks.droneRequestingCollaboration, (_id, ))
    createSubscriber(f"/{_name}/crps/handle_collaboration_request/Get_State_Enable", Bool, ros.callbacks.droneRespondingToCollaboration, (_id, ))
    tempPublisher = rospy.Publisher(f"/{_name}/Mission", MissionDji, queue_size=10) # this fixes the problem where the first attempt to publish the mission was not functioning
    tempPublisher = rospy.Publisher(f"/{_name}/BuildMapRequest", BuildMap, queue_size=10) # this fixes the problem where the first attempt to publish the buildmap was not functioning
    tempPublisher = rospy.Publisher(f"/{_name}/StartOrStopPointCloud", String, queue_size=10) # this fixes the problem where the first attempt to publish the lidar was not functioning

# called when an API request is received to start lidar
def createDroneLidarSubscriber(_droneId, _droneName, _lidarSessionId):
    print("creating lidar subscriber")
    createSubscriber(f"/{_droneName}/PointCloud", String, ros.callbacks.droneLidarReceived, (_droneId, _lidarSessionId,))

# stop all the subscribers realted to a specific drone
def stopDroneSubscribers(_name):
    stopSubscriberByName(f"/{_name}/Telemetry")
    stopSubscriberByName(f"/{_name}/Error")
    stopSubscriberByName(f"/{_name}/WeatherStation")
    stopSubscriberByName(f"/{_name}/TerminalHardware")
    stopSubscriberByName(f"/{_name}/PointCloud")
    stopSubscriberByName(f"/{_name}/crps/initiate_collaboration_request/Get_State_Enable")
    stopSubscriberByName(f"/{_name}/crps/handle_collaboration_request/Get_State_Enable")

# stop the lidar subscriber for a specific drone
def stopDroneLidarSubscriber(_name):
    print("deleting lidar subscriber")
    stopSubscriberByName(f"/{_name}/PointCloud")


#####################################
############# DEVICES ###############
#####################################


# create subscribers for a newly connected device
def createDeviceSubscribers(_id, _name):
    createSubscriber(f"/{_name}/TelemetryDevice", TelemetryDevice, ros.callbacks.deviceTelemetryReceived, (time.time(), _id))

# stop all the subscribers realted to a specific device
def stopDeviceSubscribers(_name):
    stopSubscriberByName(f"/{_name}/TelemetryDevice")


##################################
############# LORA ###############
##################################


# create subscribers for a newly connected lora
def createLoraMasterSubscribers(_id, _name):
    createSubscriber(f"/{_name}/TelemetryLora", String, ros.callbacks.loraTelemetryReceived, (time.time(), _id))
    createSubscriber(f"/{_name}/MonitorLora", String, ros.callbacks.loraMonitorDataReceived, (time.time(), _id))

# stop all the subscribers realted to a specific lora
def stopLoraMasterSubscribers(_name):
    stopSubscriberByName(f"/{_name}/TelemetryLora")
    stopSubscriberByName(f"/{_name}/MonitorLora")


###################################
############# UTILS ###############
###################################


# create a ROS subscriber with the given parameters
def createSubscriber(subscriberName, msgType, callbackFunction, args=None):
    global runningSubscribers
    if subscriberName not in runningSubscribers:
        if args is not None:
            runningSubscribers[subscriberName] = rospy.Subscriber(subscriberName, msgType, callbackFunction, args)
        else:
            runningSubscribers[subscriberName] = rospy.Subscriber(subscriberName, msgType, callbackFunction)

# stop a subscriber by name
def stopSubscriberByName(subscriberName):
    if subscriberName in runningSubscribers:
        subscriber = runningSubscribers[subscriberName]
        subscriber.unregister()
        del runningSubscribers[subscriberName] # remove the subscriber from the dictionary
        print(f"Subscriber '{subscriberName}' has been stopped.")
    else:
        print(f"No subscriber with name '{subscriberName}' is currently running.")