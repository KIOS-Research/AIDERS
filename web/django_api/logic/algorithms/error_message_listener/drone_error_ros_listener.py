from aiders import views

import logging
import rospy
from std_msgs.msg import String, Int8
import json
from aiders import models

logger = logging.getLogger(__name__)


def rosCallback(data, args):
    dronePK = args[0]
    message = json.loads(data.data)
    errorObj = {
        "message": str(message),
        "drone": dronePK,
    }
    views.TelemetryRetrieveAPIView.save_error_drone_data_in_db(errorObj)


def listener(dji_name="kios_mavic1g", canGetMission=False):
    dronePK = models.Drone.objects.get(drone_name=dji_name).pk
    rospy.Subscriber("/" + dji_name + "/Error", String, rosCallback, (dronePK,))
    logger.info("Error handler drone {} to server started.".format(dji_name))
    rospy.spin()


def main(dji_name):
    try:
        listener(dji_name=dji_name)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
