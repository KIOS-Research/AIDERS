import logging
import sys
import time

import rospy
from kios.msg import Telemetry
from pymavlink import mavutil
from std_msgs.msg import String

logger = logging.getLogger(__name__)

class mavlinkData:
    def __init__(self, master):
        self.master = master
        self.conn = True
        out = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=30)
        if not out:
            self.conn = False
            pubdroneID(False)
            print("Mavlink drone disconnected")
            sys.exit()

        self.homelat = out.lat
        self.homelon = out.lon
        reveiced_all = False
        while True:
            msg = self.master.recv_match(blocking=True, timeout=15)
            if not msg:
                self.conn = False
                pubdroneID(False)
                print("Mavlink drone disconnected")
                sys.exit()
            try:
                self.battery_percentage = master.messages['BATTERY_STATUS'].battery_remaining
                self.gps_signal = master.messages['GPS_RAW_INT'].satellites_visible
                self.satellites = master.messages['GPS_RAW_INT'].satellites_visible
                self.heading = master.messages['ATTITUDE'].yaw
                self.velocity = master.messages['GPS_RAW_INT'].vel
                self.lat = master.messages['GLOBAL_POSITION_INT'].lat
                self.lon = master.messages['GLOBAL_POSITION_INT'].lon
                self.alt = master.messages['GLOBAL_POSITION_INT'].alt
                break
            except:
                continue
    def getFromMavlink(self):
        msg = self.master.recv_match(timeout=15)
        if not msg:
            self.conn = False
            pubdroneID(False)
            print("Mavlink drone disconnected")
            sys.exit()
        try:
            self.battery_percentage = self.master.messages['BATTERY_STATUS'].battery_remaining
            self.gps_signal = self.master.messages['GPS_RAW_INT'].satellites_visible
            self.satellites = self.master.messages['GPS_RAW_INT'].satellites_visible
            self.heading = self.master.messages['ATTITUDE'].yaw
            self.velocity = self.master.messages['GPS_RAW_INT'].vel
            self.lat = self.master.messages['GLOBAL_POSITION_INT'].lat
            self.lon = self.master.messages['GLOBAL_POSITION_INT'].lon
            self.alt = self.master.messages['GLOBAL_POSITION_INT'].alt
        except:
            print("Mavlink error in: getFromMavlink()")
            pass
    def MavlinkToRosTelemetry(self, pub):
        self.getFromMavlink()
        newdata = Telemetry()
        newdata.batteryPercentage  = self.battery_percentage
        newdata.gpsSignal  = self.gps_signal
        newdata.satelliteNumber  = self.satellites
        newdata.heading  = self.heading
        newdata.velocity  = self.velocity 
        newdata.latitude  = self.lat
        newdata.longitude  = self.lon
        newdata.altitude  = self.alt
        newdata.homeLatitude = self.homelat
        newdata.homeLongitude = self.homelon
        pub.publish(newdata)



def pubdroneID(con):
    if con is True:
        data = '{"DroneName" : "mavlinkDrone", "Connected" : "True", "Model": "undefined", "cameraName": "unknown_mavlink_camera"}'
    else: 
        data = '{"DroneName" : "mavlinkDrone", "Connected" : "False", "Model": "undefined", "cameraName": "unknown_mavlink_camera"}}'
    stringObj = String()
    
    
    stringObj.data =data
    print("published in droneIds topic: \n") 
    print("\tPUBLISHED DATA: ", data)
    pub = rospy.Publisher("/droneIds", String, queue_size=10)
    pub.publish(stringObj)



def talker():
    #import pdb; pdb.set_trace()
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14445')
    out = master.recv_match(blocking=True, timeout=30)
    if not out:
        pubdroneID(False)
        print("Mavlink drone disconnected")
        sys.exit()

    pub = rospy.Publisher("/mavlinkDrone/Telemetry", Telemetry, queue_size=10)
    rate  = rospy.Rate(10) #1hz
    newdata = mavlinkData(master)   
    while (newdata.conn is True) and (not rospy.is_shutdown()):
        print("Connection: ", newdata.conn)
        newdata.MavlinkToRosTelemetry(pub)
        rate.sleep()
    
def main():
    print("RECEIVED MAVLINK HEARTBEAT, MAIN THREAD HAS STARTED")
    try:
        talker()
    except rospy.ROSInterruptedException:
        pass

def tryconnect():    
    #import pdb; pdb.set_trace()
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14445')
    logger.info("Listener script for Mavlink drones has started! Waiting for Mavlink drone to send a heart beat...")
    out = master.recv_match(type='HEARTBEAT', blocking=True)
    for x in range(2):
        pubdroneID(True)
        time.sleep(5)

