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

# save the frames of a drone's RTMP stream to the disk and its info to the database
def saveDroneRtmpFrames(_droneId, _droneName, _sessionId):

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
    captureInterval = 1 / int(os.environ.get("STREAM_CAPTURE_FPS"))  # Capture X frames per second
    nextCaptureTime = startTime + captureInterval
    frameCount = 0

    outputDirectory = f"/media/session{_sessionId}_{_droneName}"   # output directory
    if not os.path.exists(outputDirectory):
        os.mkdir(outputDirectory)        

    # stream capture loop
    while videoStream.isOpened():
        ret, frame = videoStream.read()
        if not ret:
            print(f"\n\U0001F4A9 Failed reading stream frame from drone '{_droneName}'...")
            break

        numberOfFailedReads = 0
        currentTime = time.time()
        if currentTime >= nextCaptureTime:
            currentDateTime = datetime.now(timezone).time()
            formattedDateTime = currentDateTime.strftime('%H-%M-%S')            
            
            framePath = f"{outputDirectory}/frame{frameCount:05d}_{formattedDateTime}.jpg"

            cv2.imwrite(framePath, frame)                               # save the frame to the hard drive
            database.queries.saveFrame(_droneId, _sessionId, framePath) # save the frame info in the database
            frameCount += 1
            nextCaptureTime += captureInterval

    videoStream.release()
    cv2.destroyAllWindows()

    # check if drone is still connected and start capturing again
    isStillConnected = database.queries.getDroneConnectionState(_droneId)
    if isStillConnected[0] == 1:
        # startDroneStreamCapture(_droneId, _droneName)
        print(f"\n\U0000267B Drone '{_droneName}' is still connected. Restarting stream capture.")
        saveDroneRtmpFrames(_droneId, _droneName, _sessionId)
    else:
        database.queries.updateSessionEnd(_sessionId)   # mark session end time
        print(f"\n\U0001F480 Drone '{_droneName}' is no longer connected. Stopping stream capture.")
        sys.stdout.flush()


# start the capturing loop on a new thread
def startDroneStreamCapture(_droneId, _droneName):
    threadName = f"frameCapture_{_droneName}"
    session = database.queries.getActiveDroneLiveSession(_droneId)  # retrieve last ACTIVE live stream session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            print(f"\n\U0001F6A8 Stream capture thread for '{_droneName}' already running.")
            return
    sessionId = database.queries.deactivateSessionsAndCreateNew(_droneId)  # deactivate drone's sessions and create a new one

    # start a thread that captures and saves the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=saveDroneRtmpFrames, args=(_droneId, _droneName, sessionId))
        thread.name = threadName
        thread.start()
