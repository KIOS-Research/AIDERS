import logging
import rospy
from kios.msg import Telemetry
import time
from aiders import models, views

logger = logging.getLogger(__name__)

def rosCallback(data, args):
    mission_log = models.MissionLog.objects.filter(drone=args[2], action="START_MISSION").values("pk").order_by("-pk").first()
    mission_log = mission_log["pk"] if mission_log else None

    views.TelemetryRetrieveAPIView.save_telemetry_in_db(
        {
            "drone": args[2],
            "battery_percentage": data.batteryPercentage,
            "gps_signal": data.gpsSignal,
            "satellites": data.satelliteNumber,
            "heading": data.heading,
            "velocity": data.velocity,
            "homeLat": data.homeLatitude,
            "homeLon": data.homeLongitude,
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude,
            "drone_state": data.droneState,
            "secondsOn": round((time.time() - args[1]), 2),
            "mission_log": mission_log,
            "gimbal_angle": data.gimbalAngle,
            "water_sampler_in_water": False,
        }
    )


"""
Setup listeners/subscribers
Main loop!
"""


def listener(dji_name="kios_mavic1g", canGetMission=False):
    drone = models.Drone.objects.get(drone_name=dji_name)

    rospy.Subscriber(f"/{dji_name}/Telemetry", Telemetry, rosCallback, (dji_name, time.time(), drone.pk, drone.model), queue_size=1)
    logger.info(f"Telemetry handler for drone {dji_name} started.")
    rospy.spin()


def main(drone_name):
    print(f"**Drone Telemetry Listener script started for drone {drone_name}**")
    listener(dji_name=drone_name)
