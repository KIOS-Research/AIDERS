import logging
import rospy
from aiders import models, views
from kios.msg import DroneMovement
from aiders import views

logger = logging.getLogger(__name__)


def jetson_data_callback(data, args):
    views.DroneMovementAPIView.create_data_to_db(data, args[0])


def start_listening(dji_name="kios_mavic1g"):
    try:
        subscriber = rospy.Subscriber("/" + dji_name + "/DroneMovement", DroneMovement, jetson_data_callback, (dji_name,))
        rospy.spin()
    except Exception as e:
        pass
