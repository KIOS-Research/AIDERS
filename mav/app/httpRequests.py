import os
import time
import json
import requests
import threading

streamCaptureBaseUrl = f"http://localhost:{os.environ['LSC_API_PORT']}"
detectionBaseUrl = f"http://localhost:{os.environ['CV_API_PORT']}"


def makePostRequestWithRetries(_url, _payload):
    thread = threading.Thread(target=postRequestThread, args=(_url, _payload))
    thread.start()

def postRequestThread(_url, _payload):
    headers = {
        'Content-Type': 'application/json'
    }
    json_payload = json.dumps(_payload)
    retries = 1
    while True:
        response = requests.post(_url, data=json_payload, headers=headers) # send the POST request
        if response.status_code == 200:
            break  # exit the loop
        if retries == 3:
            print(f"Request '{_url}' failed. Status code:", response.status_code)
            break
        retries += 1
        time.sleep(3)   # wait before retrying


# send a post request to the lsc container to start capturing the stream
def startDroneLiveStreamCapture(_droneId, _droneName):
    url = f"{streamCaptureBaseUrl}/startDroneStreamCapture"
    payload = {
        "droneId": _droneId,
        "droneName": _droneName
    }
    makePostRequestWithRetries(url, payload)


# send a post request to the CV container to stop detection
def stopDroneDetector(_droneId, _droneName):
    url = f"{detectionBaseUrl}/stopDetection"
    payload = {
        "droneId": _droneId,
        "droneName": _droneName
    }
    makePostRequestWithRetries(url, payload)