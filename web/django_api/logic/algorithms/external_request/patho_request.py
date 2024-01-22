import os
import subprocess
import time
from datetime import datetime
import requests
from aiders import models
import threading

def patho_get_auth_token():
    # Define the URL of the token endpoint and the client credentials
    url = "https://idm.digital-enabler.eng.it/auth/realms/pathocert/protocol/openid-connect/token"
    client_id = "pathodrone"
    client_secret = "1x0jjy5AHneuA9f1fEwcfQ2Lyan8coFN"

    # Define the request headers and data
    headers = {"Content-Type": "application/x-www-form-urlencoded"}
    data = {"client_id": client_id, "client_secret": client_secret, "grant_type": "client_credentials"}

    # Send the POST request to the token endpoint
    response = requests.post(url, headers=headers, data=data)

    # Parse the response JSON and get the access token
    if response.status_code == 200:
        response_data = response.json()
        access_token = response_data.get("access_token")
        print(f"Access token: {access_token}")
        return access_token
    else:
        print(f"Failed to get access token with status code: {response.status_code}")
        return


def patho_post_drone_data(access_token, drone, id):
    fileName="limassol_drone_"+datetime.now().strftime("%Y_%m_%d_%H%M%S")+'.jpg'
    url = "https://dataconnector-pathocert.opsi.lecce.it/dataconnector/api/v1/pathodrone/?scenario=limassol"
    # Define the headers and JSON data to send
    headers = {"x-token": access_token, "Content-Type": "application/json"}
    json_data = {
        "id": id,
        "image_metadata": {
            "location": {
                "type": "Point",
                "coordinates": [models.Telemetry.objects.filter(drone=drone).last().lon, models.Telemetry.objects.filter(drone=drone).last().lat],
            },
            "name": fileName,
            "datetime": datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
        },
    }

    # Send the POST request with headers and JSON data
    response = requests.post(url, headers=headers, json=json_data)
    # # Check the response status code
    if response.status_code == 200:
        result = subprocess.run(
        [
            "scp",
            "-i",
            "/app/logic/algorithms/external_request/pathodrone_keypem.pem",
            "/app/aiders"+models.LiveStreamSession.objects.filter(drone=drone).last().latest_frame_url,
            "pathodrone@77.241.216.25:/mnt/pathocertvolume/sharedvolume/dataproducts/pathodrone/"
            + fileName,
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        print(result.stdout, flush=True)
        print(result.stderr, flush=True)

def patho_post_device_data(access_token, device, id):
    fileName="limassol_camera_"+datetime.now().strftime("%Y_%m_%d_%H%M%S")+'.jpg'
    url = "https://dataconnector-pathocert.opsi.lecce.it/dataconnector/api/v1/pathodrone/?scenario=limassol"
    # Define the headers and JSON data to send
    headers = {"x-token": access_token, "Content-Type": "application/json"}
    json_data = {
        "id": id,
        "image_metadata": {
            "location": {
                "type": "Point",
                "coordinates": [models.DeviceTelemetry.objects.filter(device=device).last().longitude, models.DeviceTelemetry.objects.filter(device=device).last().latitude],
            },
            "name": fileName,
            "datetime": datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
        },
    }

    # Send the POST request with headers and JSON data
    response = requests.post(url, headers=headers, json=json_data)
    # # Check the response status code
    if response.status_code == 200:
        result = subprocess.run(
        [
            "scp",
            "-i",
            "/app/logic/algorithms/external_request/pathodrone_keypem.pem",
            "/app/aiders/media/"+models.DeviceImage.objects.filter(device=device).last().path,
            "pathodrone@77.241.216.25:/mnt/pathocertvolume/sharedvolume/dataproducts/pathodrone/"
            + fileName,
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        print(result.stdout, flush=True)
        print(result.stderr, flush=True)


def sendDataFromDrones(operation,access_token):
    while True:
        for drone in models.Drone.objects.filter(operation=models.Operation.objects.get(operation_name=operation), is_connected_with_platform=True):
            patho_post_drone_data(access_token, drone, datetime.now().strftime("%d_%m_%Y__%H_%M_%S"))
        time.sleep(30)

def sendDataFromDevices(operation,access_token):
    while True:
        for device in models.Device.objects.filter(operation=models.Operation.objects.get(operation_name=operation), is_connected_with_platform=True):
            patho_post_device_data(access_token, device, datetime.now().strftime("%d_%m_%Y__%H_%M_%S"))
        time.sleep(60)


def main(operation="Operation_Name 1"):
    access_token = patho_get_auth_token()
    my_thread = threading.Thread(target=sendDataFromDevices, args=(operation, access_token), name="pathocert_send_data_from_devices")
    my_thread.start()
    my_thread = threading.Thread(target=sendDataFromDrones, args=(operation, access_token), name="pathocert_send_data_from_drones")
    my_thread.start()
    sendDataFromDevices(operation, access_token)



if __name__ == "__main__":
    main()
