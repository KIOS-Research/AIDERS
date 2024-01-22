#!/usr/bin/env python
import serial
import numpy as np
import rospy
from optparse import OptionParser
from trisonica_ros.msg import trisonica_msg

class Trisonica(object):
    def __init__(self, port="/dev/ttyUSB0", topic='/trisonica', baud=115200, rate=80):
        baud = 115200
        self.rate = 80 # rate to check/publish new data, Hz
        print('Connecting to: ', port)
        self.connection = serial.Serial(port, baud, timeout=0.01)
        self.connection.flush()
        print('Connected.')
        self.publisher = rospy.Publisher(topic, trisonica_msg, queue_size=10)

    def main(self):
        msg = trisonica_msg()
        rate = rospy.Rate(self.rate) # Hz, trisonica set to 40 Hz
        while not rospy.is_shutdown():
            data = self.connection.readline()
            if data is not None and len(data) > 10:
                if 1: #data[0] == 'S':
                    msg.header.stamp.secs = rospy.Time.now().secs
                    msg.header.stamp.nsecs = rospy.Time.now().nsecs
                    try:
                        msg.speed       = float( data.decode().split('S ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed = np.nan 
                        pass

                    try:
                        msg.speed2d       = float( data.decode().split('S2 ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.speed2d = np.nan  
                        pass

                    try:
                        msg.direction   = float( data.decode().split('D ')[1].lstrip().split(' ')[0])
                    except:
                        #msg.direction = np.nan   
                        pass

                    try:
                        msg.northsouth  = float( data.decode().split('U ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.northsouth = np.nan  
                        pass

                    try:
                        msg.westeast    = float( data.decode().split('V ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.westeast = np.nan  
                        pass

                    try:
                        msg.updown      = float( data.decode().split('W ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.updown = np.nan  
                        pass

                    try:
                        msg.temperature = float( data.decode().split('T ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.temperature = np.nan  
                        pass

                    try:
                        msg.pressure = float( data.decode().split('P ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.pressure = np.nan  
                        pass

                    try:
                        msg.humidity = float( data.decode().split('H ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.humidity = np.nan   
                        pass

                    try:
                        msg.pitch = float( data.decode().split('P ')[2].lstrip().split(' ')[0] )
                    except:
                        try:
                            msg.pitch = float( data.decode().split('PI ')[1].lstrip().split(' ')[0] )
                        except:
                            #msg.pitch = np.nan  
                            pass

                    try:
                        msg.roll = float( data.decode().split('RO ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.roll = np.nan   
                        pass

                    try:
                        msg.heading = float( data.decode().split('MD ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.heading = np.nan   
                        pass

                    try:
                        msg.levelx = float( data.decode().split('AX ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levelx = np.nan   
                        pass

                    try:
                        msg.levely = float( data.decode().split('AY ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levely = np.nan   
                        pass

                    try:
                        msg.levelz = float( data.decode().split('AZ ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.levelz = np.nan   
                        pass




                    self.publisher.publish(msg)

            rate.sleep()

        self.connection.close()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--port", type="str", dest="port", default='/dev/ttyUSB0',
                        help="port to which trisonica is connected")
    parser.add_option("--topic", type="str", dest="topic", default='/trisonica',
                        help="rostopic to publish to")
    parser.add_option("--rate", type="int", dest="rate", default=80,
                        help="ROS rate to check for data and publish")
    parser.add_option("--baud", type="int", dest="baud", default=11520,
                        help="baudrate")
    parser.add_option("--nodename", type="str", dest="nodename", default='trisonica',
                        help="name of the node")

    (options, args) = parser.parse_args()

    rospy.init_node(options.nodename, anonymous=True)
    print("PORT: " + str(options.port))
    trisonica = Trisonica(port=options.port, topic=options.topic, baud=options.baud, rate=options.rate)
    trisonica.main()
