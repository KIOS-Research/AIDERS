import os
import sys
import time
import threading
import cv2
from datetime import datetime
import pytz

# custom libs
from database.connection import MySQLConnector
import database.queries
import utils

timezone = pytz.utc # timezone = pytz.timezone(os.environ.get("TZ"))

# surpress ffmpeg opencv warnings
os.environ['OPENCV_FFMPEG_LOGLEVEL'] = 'quiet'

# Global dictionary to keep track of stop events for threads by name
thread_stop_events = {}

# save the frames of a drone's RTMP stream to the disk and its info to the database
def saveDroneRtmpFrames(_droneId, _droneName, _sessionId):
    threadName = f"frameCapture_{_droneName}"
    stop_event = thread_stop_events.get(threadName)

    dbStreamUrl = database.queries.getDroneStreamURL(_droneId)  # retrieve stream URL from db

    if dbStreamUrl is not None:
        if dbStreamUrl[0] is not None:
            streamUrl = dbStreamUrl[0]  # custom stream URL from the database
        else:
            streamUrl = f"rtmp://{os.environ['NET_IP']}/live/{_droneName}"  # RTMP stream URL        

    else:
        return
    
    print(f"\n\U0001F3AC Connecting to stream for '{_droneName}' at '{streamUrl}'")

    retries = 0
    maxRetries = 5

    # stream connection loop
    while True:
        # print("START")
        videoStream = cv2.VideoCapture(streamUrl)
        if not videoStream.isOpened():
            if retries == maxRetries:
                print(f"\n\U0001F480 Failed reading video stream for '{_droneName}' after {retries} attempts.")
                videoStream.release()
                cv2.destroyAllWindows()
                database.queries.updateSessionEnd(_sessionId)   # mark session end time
                return
            retries += 1
            print(f"\n\U0001F6AB Error reading video stream for '{_droneName}'. Will retry.")
            time.sleep(5)
            print(f"\n\U0001F504 RETRYING: '{_droneName}'...")
        else:
            break

    print(f"\n\U0001F4AA Stream capture from drone '{_droneName}' started.")
    startTime = time.time()
    database.queries.updateLiveStreamConnectionStatus(_droneId)
    captureInterval = 1 / int(os.environ.get("COMPUTER_VISION_FPS"))  # Capture X frames per second
    nextCaptureTime = startTime + captureInterval
    frameCount = 0

    outputDirectory = f"/media/session{_sessionId}_{_droneName}"   # output directory
    if not os.path.exists(outputDirectory):
        os.mkdir(outputDirectory)        

    numberOfFailedReads = 0
    # stream capture loop
    while videoStream.isOpened():
        # Check for stop event in the capture loop
        if stop_event and stop_event.is_set():
            print(f"\n\U0001F6D1 Stop signal received for '{_droneName}'.")
            break

        ret, frame = videoStream.read()
        # print(ret)
        if not ret:
            if(numberOfFailedReads > 1):
                print(f"\n\U0001F4A9 Failed reading stream frame from drone '{_droneName}'...")
                break
            else:            
                numberOfFailedReads = numberOfFailedReads + 1
                # print("INCREASED FAILS ")
                # print(numberOfFailedReads)
                continue
        else: 
            numberOfFailedReads = 0
            # print("RESET FAILS ")
            # print(numberOfFailedReads)



        # print(numberOfFailedReads)
        currentTime = time.time()
        if currentTime >= nextCaptureTime:
            currentDateTime = datetime.now(timezone).time()
            formattedDateTime = currentDateTime.strftime('%H-%M-%S')            
            
            framePath = f"{outputDirectory}/frame{frameCount:05d}_{formattedDateTime}.jpg"
            # print(framePath)
            cv2.imwrite(framePath, frame)                              # save the frame to the hard drive
            database.queries.saveFrame(_droneId, _sessionId, framePath) # save the frame info in the database
            frameCount += 1
            nextCaptureTime += captureInterval

    videoStream.release()
    cv2.destroyAllWindows()

    # check if drone is still connected and start capturing again
    if stop_event and stop_event.is_set():
        print(f"\n\U0001F6D1 Stop signal received for '{_droneName}'. Will not try to reconnect.")
        sys.stdout.flush()
        return
    
    isStillConnected = database.queries.getDroneConnectionState(_droneId)
    if isStillConnected[0] == 1:
        # startDroneStreamCapture(_droneId, _droneName)
        print(f"\n\U0000267B Drone '{_droneName}' is still connected. Restarting stream capture.")
        saveDroneRtmpFrames(_droneId, _droneName, _sessionId)
    else:
        # legacyLSC
        # database.queries.updateSessionEnd(_sessionId)   # mark session end time
        print(f"\n\U0001F480 Drone '{_droneName}' is no longer connected. Stopping stream capture.")
        sys.stdout.flush()


# start the capturing loop on a new thread
def startDroneStreamCapture(_droneId, _droneName):
    threadName = f"frameCapture_{_droneName}"

    if utils.threadStarted(threadName):
        print(f"\n\U0001F6A8 Stream capture thread for '{_droneName}' already running.")
        return

    session = database.queries.getActiveDroneLiveSession(_droneId)  # retrieve last ACTIVE live stream session from DB
    if session is not None:
        sessionId = session[0]
        print(f"\n\U0001F6A8 There is an active live session for drone '{_droneName}'.")
    else:
        print(f"\n\U0001F6A8 Creating a new session for drone '{_droneName}'.")
        sessionId = database.queries.deactivateSessionsAndCreateNew(_droneId)

    print(f"\n\U0001F4C5 Session ID: {sessionId} for drone '{_droneName}'.")

    # start a thread that captures and saves the stream frames
    if sessionId > 0:
        stop_event = threading.Event()
        thread_stop_events[threadName] = stop_event
        thread = threading.Thread(target=saveDroneRtmpFrames, args=(_droneId, _droneName, sessionId))
        thread.name = threadName
        thread.start()
    else:
        print(f"\n\U0001F6AB No active live stream session found for drone '{_droneName}'. Cannot start capture.")


# stop the capturing thread
def stopDroneStreamCapture(_droneName):
    threadName = f"frameCapture_{_droneName}"
    stop_event = thread_stop_events.get(threadName)
    if stop_event:
        stop_event.set()
        print(f"\n\U0001F6D1 Stop signal sent to thread '{threadName}'.")
    else:
        print(f"\n\U0001F50D No stop event found for thread '{threadName}'.")
