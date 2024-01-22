import logging
import time

import rospy
from aiders import models, views
from std_msgs.msg import String

logger = logging.getLogger(__name__)


def rosCallback(data, args):
    data = data.data.split(",")
    if len(data) == 6:
        views.BaloraList.save_lora_telemetry_to_db(
            {
                "baloraMaster": args[0].id,
                "balora": views.BaloraList.save_lora_network_to_db(data[0], args[0]).id,
                # "time":args[1],
                "latitude": float(data[2]),
                "longitude": float(data[3]),
                "pm1": None,
                "pm25": None,
                "secondsOn": round((time.time() - args[1]), 2),
                "received_signal_strength_indication": float(data[4]),
                "SignalToNoiseRatio": float(data[5]),
            }
        )
    elif len(data) == 8:
        views.BaloraList.save_lora_telemetry_to_db(
            {
                "baloraMaster": args[0].id,
                "balora": views.BaloraList.save_lora_network_to_db(data[0], args[0]).id,
                # "time":args[1],
                "latitude": float(data[2]),
                "longitude": float(data[3]),
                "pm1": float(data[4]),
                "pm25": float(data[5]),
                "secondsOn": round((time.time() - args[1]), 2),
                "received_signal_strength_indication": float(data[6]),
                "SignalToNoiseRatio": float(data[7]),
            }
        )


def listener(name="lora1"):
    rospy.Subscriber(
        f"/{name}/loremetry",
        String,
        rosCallback,
        (
            models.BaloraMaster.objects.get(name=name),
            time.time(),
        ),
    )
    logger.info(f"Telemetry handler for lora {name} started.")
    rospy.spin()


def main(lora_name):
    print(f"**Balora Telemetry Listener script started for lora {lora_name}**")
    listener(name=lora_name)
