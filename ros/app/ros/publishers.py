import rospy
import threading
import time

# custom libs
import database.queries

# ROS messages
from std_msgs.msg import String, Int32, Bool
from kios.msg import GpsInput, MissionCommandDJI, MissionDji, BuildMap


# publish a message in the client's handshake topic to confirm communication
def performHandshake(_clientName):
    handshake = rospy.Publisher(f"/{_clientName}/handshake", String, queue_size=10)
    msg = String()
    msg.data = "1"
    handshake.publish(msg)


# publish a message to start or stop sending images for build map
def startOrStopBuildMap(_droneName, _command, _overlap):
    publisher = rospy.Publisher(f"/{_droneName}/BuildMapRequest", BuildMap, queue_size=10)
    t = BuildMap()
    t.buildmap = True if _command == "START" else False
    t.overlap = int(_overlap)
    publisher.publish(t)


# publish a message to open the water sampler valve
def openWaterSamplingValve(_droneName):
    publisher = rospy.Publisher(f"/{_droneName}/WaterSampler", Int32, queue_size=10)
    t = Int32()
    t.data = 1
    publisher.publish(t)


# publish a message to start or stop lidar broadcasting
def startOrStopLidar(_droneName, _command):
    publisher = rospy.Publisher(f"/{_droneName}/StartOrStopPointCloud", String, queue_size=10)
    msg = String()
    msg.data = _command
    publisher.publish(msg)


# publish a mission to a specific drone
def publishMission(_data):
    droneId = _data["droneId"]
    droneName = _data["droneName"]
    grid = _data["grid"]
    action = _data["action"]
    missionPath = _data["missionPath"]
    missionSpeed = _data["missionSpeed"]
    missionGimbal = _data["missionGimbal"]
    missionRepeat = _data["missionRepeat"]
    captureAndStoreImages = _data["captureAndStoreImages"]

    try:
        publisher = rospy.Publisher(f"/{droneName}/Mission", MissionDji, queue_size=10)

        missionMsg = MissionDji()
        missionMsg.name = droneName
        missionMsg.header.frame_id = droneName
        missionMsgCommand = MissionCommandDJI()

        if action == "START_MISSION":
            missionMsgCommand.missionCommand = missionMsgCommand.start
            missionMsg.grid = grid
            missionMsg.captureAndStoreImages = captureAndStoreImages
            missionMsg.repeat = missionRepeat
            tempSpeed = ""
            tempGimbal = ""

            for i in range(0, len(missionPath)):
                if isinstance(missionSpeed, str):
                    tempSpeed = float(missionSpeed)
                elif isinstance(missionSpeed, list):
                    tempSpeed = float(missionSpeed[i])
                if isinstance(missionGimbal, str):
                    tempGimbal = missionGimbal
                elif isinstance(missionGimbal, list):
                    tempGimbal = missionGimbal[i]
                
                gpsInput = GpsInput()
                gpsInput.latitude = missionPath[i][1]
                gpsInput.longitude = missionPath[i][0]
                gpsInput.altitude = float(missionPath[i][2]) # ORIGINAL
                gpsInput.speed = tempSpeed
                gpsInput.gimbalAngle = tempGimbal
                gpsInput.stayTime = 0
                gpsInput.photo = False
                missionMsg.gpsInput.append(gpsInput)
        else:
            if action == "PAUSE_MISSION":
                missionMsgCommand.missionCommand = missionMsgCommand.pause
            elif action == "RESUME_MISSION":
                missionMsgCommand.missionCommand = missionMsgCommand.resume
            elif action == "CANCEL_MISSION":
                missionMsgCommand.missionCommand = missionMsgCommand.stop

        missionMsg.missionCommand = missionMsgCommand
        
        # publish mission to ROS on a new thread
        thread = threading.Thread(target=publishMissionUntilReceived, args=(droneId, publisher, missionMsg, action))
        thread.start()

    except Exception as e:
        print(e)


# published the mission to the drone until its state is changed to the expected one
def publishMissionUntilReceived(_droneId, _publisher, _missionMsg, _action):
    expectedState = ""
    if _action == "START_MISSION":
        expectedState = "In_Mission"
    elif _action == "PAUSE_MISSION":
        expectedState = "Paused_Mission"
    elif _action == "RESUME_MISSION":
        expectedState = "In_Mission"
    elif _action == "CANCEL_MISSION":
        expectedState = "Flying"
    else:
        print("\nError: invalid mission action command!", flush=True)
        return

    retries = 0
    while True:
        _publisher.publish(_missionMsg) # send the mission

        time.sleep(3)
        state = database.queries.getDroneState(_droneId) # get drone's current state
        droneState = state[0]

        if(droneState == expectedState):
            print(f"\nMission command received by drone '{_missionMsg.name}'", flush=True)
            break

        if(retries > 4):
            print(f"\nMission command for drone '{_missionMsg.name}' failed to be delivered.", flush=True)
            break

        retries = retries + 1      
        print(f"\nRetrying to send mission command to drone '{_missionMsg.name}' ({retries}).", flush=True)

# Safedrone GPS reliability Publish
def safeDroneReliableNavigation(_droneName, _command):
    publisher = rospy.Publisher(f"/{_droneName}/safedrone/reliableNavigation", Bool, queue_size=10)
    msg = Bool()
    msg.data = _command
    publisher.publish(msg)