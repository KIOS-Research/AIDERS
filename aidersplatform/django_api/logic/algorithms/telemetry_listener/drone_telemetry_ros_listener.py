import requests
import logging
import rospy
import sys
from kios.msg import InputDJI, Telemetry
import csv
import json
import time
from aiders import models, views

logger = logging.getLogger(__name__)

input_data = 0


def rosCallback(data, args):
    global input_data
    if input_data == 2 or args[2] != "MATRICE_300_RTK":
        dji_name = args[0]
        startTime = args[1]
        dronePK = args[2]
        seconds = round((time.time() - startTime), 2)
        mission_log = None
        if data.droneState == 'In_Mission':
            mission_log = models.MissionLog.objects.filter(
                drone=dronePK, action="START_MISSION").last()
            mission_log = mission_log.pk

        telemetryObj = {
            "drone": dronePK,
            "battery_percentage": data.batteryPercentage,
            "gps_signal":  data.gpsSignal,
            "satellites": data.satelliteNumber,
            "heading": data.heading,
            "velocity": data.velocity,
            "homeLat": data.homeLatitude,
            "homeLon": data.homeLongitude,
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude,
            "drone_state": data.droneState,
            "secondsOn": seconds,
            "mission_log": mission_log,
            "gimbal_angle": data.gimbalAngle,
            "water_sampler_in_water": False,
        }
        input_data = 0
        views.TelemetryRetrieveAPIView.save_telemetry_in_db(telemetryObj)
    else:
        input_data = input_data+1
        pass


'''
Setup listeners/subscribers
Main loop!
'''


def listener(dji_name="kios_mavic1g", canGetMission=False):
    # print(dji_name)
    starttime = time.time()
    # print("OVER_HERE1")
    drone = models.Drone.objects.get(drone_name=dji_name)

    subscriber = rospy.Subscriber(
        "/" + dji_name + "/Telemetry", Telemetry, rosCallback, (dji_name, starttime, drone.pk, drone.model))
    logger.info('Telemetry handeler for drone {} started.'.format(dji_name))
    rospy.spin()


def main(drone_name):
    print("**Drone Telemetry Listener script started for drone {}**".format(drone_name))
    listener(dji_name=drone_name)
