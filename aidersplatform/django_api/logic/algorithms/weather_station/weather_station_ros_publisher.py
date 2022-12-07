#!/usr/bin/env python
import logging
import os
import stat
import sys
import threading
import time
from optparse import OptionParser

import numpy as np
import rospy
import serial
from trisonica_ros.msg import trisonica_msg

logger = logging.getLogger(__name__)


class Trisonica(object):
    def __init__(self, port="/dev/ttyUSB0", topic='/trisonica', baud=115200, rate=80):
        baud = 115200
        self.rate = 1  # rate to check/publish new data, Hz
        self.port = port
        self.baud = baud
        logger.info('Connecting to: {port}'.format(port=self.port))
        # print('Connecting to: ', self.port )
        try:
            self.connection = serial.Serial(port, self.baud, timeout=0.01)
            self.connection.flush()
        except serial.serialutil.SerialException as e:
            # print(e)
            logger.error(e)
            logger.info("No weather station plugged on this PC. Waiting.")
            # print("No weather station plugged on this PC. Waiting...")
            self.keepRetrying()

        self.publisher = rospy.Publisher(topic, trisonica_msg, queue_size=10)
        time.sleep(1)
        logger.info('Weather station connected!')
        # print('Weather station connected!')

    def main(self, operation, currentThread):
        # print("ON MAIN OF WEATHER STATION")
        msg = trisonica_msg()

        rate = rospy.Rate(self.rate)  # Hz, trisonica set to 40 Hz

        while not rospy.is_shutdown() and not currentThread.stopped():
            """msg.header.stamp.secs = rospy.Time.now().secs
                msg.header.stamp.nsecs = rospy.Time.now().nsecs
                msg.speed=2
                msg.speed2d=3
                msg.direction=4
                msg.northsouth=5
                msg.westeast=6
                msg.updown= 7
                msg.temperature=8
                msg.pressure=9
                msg.humidity=10
                msg.pitch =11
                msg.roll=12
                msg.heading=13
                msg.levelx=14
                msg.levely=15
                msg.levelz=16

            self.publisher.publish(msg) """

            try:
                data = self.connection.readline()
            except serial.serialutil.SerialException as e:
                logger.warning("Seems like weather station was disconnected!")
                logger.info("Waiting for weather station to reconnect.")
                # print("Seems like weather station was disconnected! ")
                # print("Waiting for weather station to reconnect...")
                self.connection.close()
                data = self.keepRetrying()

            if data is not None and len(data) > 10:
                if 1:  # data[0] == 'S':
                    msg.header.stamp.secs = rospy.Time.now().secs
                    msg.header.stamp.nsecs = rospy.Time.now().nsecs
                    try:
                        msg.speed = float(data.decode().split('S ')[
                                          1].lstrip().split(' ')[0])
                    except:
                        #msg.speed = np.nan
                        pass

                    try:
                        msg.speed2d = float(data.decode().split('S2 ')[
                                            1].lstrip().split(' ')[0])
                    except:
                        #msg.speed2d = np.nan
                        pass

                    try:
                        msg.direction = float(data.decode().split('D ')[
                                              1].lstrip().split(' ')[0])
                    except:
                        #msg.direction = np.nan
                        pass

                    try:
                        msg.northsouth = float(data.decode().split('U ')[
                                               1].lstrip().split(' ')[0])
                    except:
                        #msg.northsouth = np.nan
                        pass

                    try:
                        msg.westeast = float(data.decode().split('V ')[
                                             1].lstrip().split(' ')[0])
                    except:
                        #msg.westeast = np.nan
                        pass

                    try:
                        msg.updown = float(data.decode().split('W ')[
                                           1].lstrip().split(' ')[0])
                    except:
                        #msg.updown = np.nan
                        pass

                    try:
                        msg.temperature = float(data.decode().split('T ')[
                                                1].lstrip().split(' ')[0])
                    except:
                        #msg.temperature = np.nan
                        pass

                    try:
                        msg.pressure = float(data.decode().split('P ')[
                                             1].lstrip().split(' ')[0])
                    except:
                        #msg.pressure = np.nan
                        pass

                    try:
                        msg.humidity = float(data.decode().split('H ')[
                                             1].lstrip().split(' ')[0])
                    except:
                        #msg.humidity = np.nan
                        pass

                    try:
                        msg.pitch = float(data.decode().split('P ')[
                                          2].lstrip().split(' ')[0])
                    except:
                        try:
                            msg.pitch = float(data.decode().split(
                                'PI ')[1].lstrip().split(' ')[0])
                        except:
                            #msg.pitch = np.nan
                            pass

                    try:
                        msg.roll = float(data.decode().split(
                            'RO ')[1].lstrip().split(' ')[0])
                    except:
                        #msg.roll = np.nan
                        pass

                    try:
                        msg.heading = float(data.decode().split('MD ')[
                                            1].lstrip().split(' ')[0])
                    except:
                        #msg.heading = np.nan
                        pass

                    try:
                        msg.levelx = float(data.decode().split('AX ')[
                                           1].lstrip().split(' ')[0])
                    except:
                        #msg.levelx = np.nan
                        pass

                    try:
                        msg.levely = float(data.decode().split('AY ')[
                                           1].lstrip().split(' ')[0])
                    except:
                        #msg.levely = np.nan
                        pass

                    try:
                        msg.levelz = float(data.decode().split('AZ ')[
                                           1].lstrip().split(' ')[0])
                    except:
                        #msg.levelz = np.nan
                        pass

                    # print("MSG: ", msg)
                    self.publisher.publish(msg)
                    # print("==========================================================")
            rate.sleep()
        self.connection.close()

    def keepRetrying(self):
        while(1):
            time.sleep(1)
            try:
                self.connection = serial.Serial(
                    self.port, self.baud, timeout=0.01)
                self.connection.flush()
                data = self.connection.readline()
                logger.info('Weather station reconnected!')
                # print("Weather station reconnected!")
                break
            except Exception as e:
                # print("Exception occured!")
                # print(e)

                continue
        return data


def main(operation, threadName):
    logger.info('Starting publisher Weather Station in operation {operation}'.format(
        operation=operation))
    if (len(sys.argv) < 2):
        logger.error("No path provided for weather symlink. Exiting.")
        # print("No path provided for weather symlink. Exiting..")
        sys.exit()
    else:
        trisonicaSymlinkPatch = sys.argv[1]

    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default=trisonicaSymlinkPatch,
                      help="port to which trisonica is connected")
    parser.add_option("--topic", type="str", dest="topic", default='/'+operation+'/trisonica',
                      help="rostopic to publish to")
    parser.add_option("--rate", type="int", dest="rate", default=80,
                      help="ROS rate to check for data and publish")
    parser.add_option("--baud", type="int", dest="baud", default=11520,
                      help="baudrate")
    parser.add_option("--nodename", type="str", dest="nodename", default='trisonica',
                      help="name of the node")

    (options, args) = parser.parse_args()
    if rospy.get_node_uri() == None:
        try:
            rospy.init_node('dji_input', anonymous=True)
        except rospy.exceptions.ROSException as e:
            logger.error("Ros Node exception: {e}".format(e=e))
            # print("Ros Node exception: " + e)
    trisonica = Trisonica(port=options.port, topic=options.topic,
                          baud=options.baud, rate=options.rate)
    logger.info(
        'A weather station is plugged on this PC! Will now start forwarding weather data!')
    # print("\n\nA weather station is plugged on this PC! Will now start forwarding weather data!")
    currentThread = ''
    for thread in threading.enumerate():
        print(thread.name, threadName)
        print(thread.name == threadName)
        if thread.name == threadName:
            currentThread = thread
    trisonica.main(operation, currentThread)


if __name__ == '__main__':
    main()
