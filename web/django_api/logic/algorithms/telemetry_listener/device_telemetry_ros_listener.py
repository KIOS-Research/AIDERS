import logging
import rospy

from kios.msg import TelemetryDevice
import time
from aiders import models, views

logger = logging.getLogger(__name__)


def rosCallback(data, args):
    views.TelemetryRetrieveAPIView.save_telemetry_device_in_db(
        {
            "device": args[2],
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude,
            "heading": data.heading,
            "orientation_x": data.orientation_x,
            "orientation_y": data.orientation_y,
            "orientation_z": data.orientation_z,
            "accelerometer_x": data.accelerometer_x,
            "accelerometer_y": data.accelerometer_y,
            "accelerometer_z": data.accelerometer_z,
            "gyroscope_x": data.gyroscope_x,
            "gyroscope_y": data.gyroscope_y,
            "gyroscope_z": data.gyroscope_z,
            "geomagnetic_x": data.geomagnetic_x,
            "geomagnetic_y": data.geomagnetic_y,
            "geomagnetic_z": data.geomagnetic_z,
            "light": data.light,
            "step": data.step,
            "pressure": data.pressure,
            "proximity": data.proximity,
            "battery_percentage": data.battery_percentage,
            "secondsOn": round((time.time() - args[1]), 2),
        }
    )


"""
Setup listeners/subscribers
Main loop!
"""


def listener(name="kios_android1"):
    device = models.Device.objects.get(name=name)

    rospy.Subscriber(
        f"/{name}/TelemetryDevice",
        TelemetryDevice,
        rosCallback,
        (name, time.time(), device.pk, device.model),
    )
    logger.info(f"Telemetry handler for device {name} started.")
    rospy.spin()


def main(name):
    print(f"**Device Telemetry Listener script started for device {name}**")
    listener(name=name)
