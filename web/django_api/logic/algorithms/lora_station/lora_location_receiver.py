import contextlib
import os
import random
import signal
import subprocess
import threading
import time
from threading import Thread

import rospy
import serial
from aiders import models, serializers, views
from logic.Constants import Constants
from std_msgs.msg import String

lat = 35.007696676570056
lon = 33.2726922542


# def receiveLocations(port, op_name):
#     global flag
#     global lat
#     global lon
#     flag = True
#     baud = 4800
#     firstTime = True
#     try:
#         connection = serial.Serial(port, baud, timeout=1)
#     except serial.serialutil.SerialException as e:
#         print(e)
#         print("No lora receiver station plugged on this PC. Waiting...")
#         views.TelemetryRetrieveAPIView.save_error_data_in_db("No lora receiver station plugged on this PC. Waiting...", op_name)
#         connection = keepRetrying(port, baud)
#     while flag:
#         try:

#             data = connection.readline()
#             print("DATA= ", data)
#             try:
#                 data = data.split()[0].decode("UTF-8").split(",")
#                 tagName, uptime, currentLat, currentLon = data[0], data[1], data[2], data[3]
#             except (IndexError, UnicodeDecodeError) as e:
#                 # SOmething is wrong with the received information.
#                 # Usually if the received data are corrupted, we get an exception when we try to decode
#                 continue
#         except serial.serialutil.SerialException as e:
#             print(e)
#             print("Seems like the lora receiver station was disconnected! ")
#             print("Waiting for lora receiver station to reconnect...")
#             connection = keepRetrying(port, baud)

#         with contextlib.suppress(ValueError):
#             # if (firstTime):
#             #     prevLat = lat
#             #     prevLon = lon
#             #     currentLat = lat
#             #     currentLon = lon
#             #     firstTime = False
#             # else:
#             #     currentLon = prevLon
#             #     currentLat =  prevLat
#             #     if (bool(random.getrandbits(1))):
#             #         prevLat = prevLat + random.uniform(0.0001, 0.0003)
#             #     else:
#             #         prevLon = prevLon + random.uniform(0.0001, 0.0003)

#             obj = models.LoraTransmitter.objects.filter(tagName=tagName).first()
#             # This lora device does not exist in the database yet. Let's add it
#             if obj is None and isTagNameCorrectFormat(tagName):
#                 operation = models.Operation.objects.get(operation_name=op_name)
#                 serializer = serializers.LoraTransmitterSerializer(data={"tagName": tagName, "upTime": uptime, "operation": operation.pk})

#                 if serializer.is_valid():
#                     obj = serializer.save()
#                 else:
#                     print("ERRORS: ", serializer.errors)
#             obj.upTime = uptime
#             obj.save()

#             loraTransmitterObj = {"loraTransmitter": obj.id, "lat": currentLat, "lon": currentLon}

#             serializer = serializers.LoraTransmitterLocationSerializer(data=loraTransmitterObj)

#             if serializer.is_valid():
#                 serializer.save()
#             else:
#                 print("ERRORS: ", serializer.errors)
#     connection.close()


# def keepRetrying(port, baud):
#     connection = None
#     while 1:
#         time.sleep(1)
#         try:
#             connection = serial.Serial(port, baud, timeout=1)
#             connection.flush()
#             print("Balora station reconnected!")
#             break
#         except Exception as e:
#             #             print("retrying..")
#             # print("Exception occured!")
#             # print(e)

#             continue

#     return connection


# Requirements for tagname. We want to make sure to cover the case when receiving corrupted lora data, which happens quite often


def isTagNameCorrectFormat(tagName):
    return bool((tagName[0] == "T") and tagName[1].isdecimal() and len(tagName) == 2)


def start_script():
    return subprocess.Popen(
        [
            "rosrun",
            "rosserial_python",
            "serial_node.py",
            Constants.LORA_RECEIVER_DEVICE_SYMLINK_PATH,
        ]
    )


def stop_script(process):
    process.send_signal(signal.SIGINT)
    process.terminate()


def setUpLoraStationUSBConnecting():
    ser = None
    script_process = None

    while True:
        if not ser:
            try:
                if os.path.exists("/dev/loraStation") == True:
                    ser = serial.Serial("/dev/loraStation")
                else:
                    ser = None
            except serial.serialutil.SerialException:
                ser = None
                continue
            if not script_process and ser != None:
                script_process = start_script()
        else:
            if os.path.exists("/dev/loraStation") == False:
                ser = None
            if ser is None:
                if script_process:
                    stop_script(script_process)
                    script_process = None
                ser = None
        time.sleep(5)
