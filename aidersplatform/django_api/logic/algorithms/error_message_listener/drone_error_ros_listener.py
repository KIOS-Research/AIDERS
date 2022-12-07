import pprint
from aiders import views

import requests
import logging
import rospy
import sys
from kios.msg import InputDJI,Telemetry
from std_msgs.msg import String, Int8
import csv
import json
import time
from aiders import models,serializers

logger = logging.getLogger(__name__)

def rosCallback(data, args):
    dronePK  = args[0]
    message = json.loads(data.data)
    errorObj={
        "message":str(message),
        "drone": dronePK,
    }
    views.TelemetryRetrieveAPIView.save_error_drone_data_in_db(errorObj)

def listener(dji_name="kios_mavic1g", canGetMission = False):
    # print(dji_name)
    starttime = time.time()
    # print("OVER_HERE1")
    dronePK = models.Drone.objects.get(drone_name=dji_name).pk
    subscriber = rospy.Subscriber("/" + dji_name + "/Error", String, rosCallback, (dronePK,))
    logger.info('Error handler drone {} to server started.'.format(dji_name))
    rospy.spin()

def main(dji_name):
    # global BASE_URL_DRONE_API
    # if (len(sys.argv) < 2):
    #     print("\nPlease make sure you provide the following arguments:"
    #           "\n1)Drone Name (e.g kios_mavic2h)"
    #           "\n2)URL of the DroneAPI (e.g http://localhost:5000)\n")
    #     exit()
    # else:
        # dji_name = sys.argv[1]
    print("dji_name: " + dji_name)
    try:
        listener(dji_name=dji_name)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()