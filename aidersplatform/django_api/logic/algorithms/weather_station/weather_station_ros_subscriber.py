import json
import logging
import queue
import sys
import threading
import time
from datetime import datetime

import requests
import rospy
from aiders import models, views
from django.urls import reverse
from std_msgs.msg import String
from trisonica_ros.msg import trisonica_msg

logger = logging.getLogger(__name__)


def weatherStationsCallback(data, args):
    operation = args[0]
    views.WeatherStationAPIView.addWeatherStationDataToDB(data, operation)


def listener(operation):
    sub = rospy.Subscriber("/"+operation+"/trisonica", trisonica_msg,
                           weatherStationsCallback, (operation,))

    while not rospy.is_shutdown():
        rospy.sleep(1)
    # rospy.spin()


def main(operation, threadName):
    try:
        logger.info('Starting subscriber Weather Station in operation {operation}'.format(
            operation=operation))
        # if rospy.get_node_uri() == None: #This is the case where script is started as separate terminal and thus, rospy.init_noe should be called
        #     rospy.init_node('dji_input', anonymous=True)
        # sys.argv = []
        if rospy.get_node_uri() == None:
            try:
                rospy.init_node('dji_input', anonymous=True)
            except rospy.exceptions.ROSException as e:
                logger.error('Ros Node exception: {e}'.format(e=e))
        logger.info('Waiting to listen for Weather Station data.')
        listener(operation)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
