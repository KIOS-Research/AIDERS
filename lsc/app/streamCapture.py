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
    
    print(f"\nStream URL: '{streamUrl}'")
    
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
            print(f"\n\U0001F4A9 Stream from drone '{_droneName}' can no longer be read.")
            break

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
    database.queries.updateSessionEnd(_sessionId)   # mark session end time
    print(f"\n\U0001F480 Thread for drone '{_droneName}' has stopped.")
    sys.stdout.flush()


# start the capturing loop on a new thread
def startDroneStreamCapture(_droneId, _droneName):
    threadName = f"frameCapture_{_droneName}"
    session = database.queries.getActiveDroneLiveSession(_droneId)  # retrieve last ACTIVE live stream session from DB
    if session is not None:
        if utils.threadStarted(threadName):
            return
    sessionId = database.queries.deactivateSessionsAndCreateNew(_droneId)  # deactivate drone's sessions and create a new one

    # start a thread that captures and saves the stream frames
    if not utils.threadStarted(threadName):
        thread = threading.Thread(target=saveDroneRtmpFrames, args=(_droneId, _droneName, sessionId))
        thread.name = threadName
        thread.start()


'''
BASED ON FRAME-RATE INSTEAD OF TIME -- WORKS!
'''

# # Create the capture object
# rtmp_url = "rtmp://192.168.0.10/live/SIM_Alpha"
# cap = cv2.VideoCapture(rtmp_url)

# # Create the folder for captured frames if it doesn't exist
# output_folder = "test_capture_frames"
# if not os.path.exists(output_folder):
#     os.makedirs(output_folder)

# # Set the desired capture rate (5 frames per second)
# capture_rate = 5  # frames per second
# frame_interval = int(cap.get(cv2.CAP_PROP_FPS) / capture_rate)

# frame_count = 0
# while cap.isOpened():
#     ret, frame = cap.read()
    
#     if not ret:
#         break
    
#     # Save the captured frame
#     if frame_count % frame_interval == 0:
#         frame_filename = os.path.join(output_folder, f"frame_{frame_count}.jpg")
#         cv2.imwrite(frame_filename, frame)
#         print(f"Saved frame {frame_count}")
    
#     frame_count += 1

# # Release the capture object
# cap.release()

# print("Capture complete.")
