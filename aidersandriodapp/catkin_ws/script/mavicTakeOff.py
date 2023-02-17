#!/usr/bin/env python

# Author: Theo
# Email: theodosis8@hotmail.com
# Date :31/01/2018

# license removed for brevity
import rospy
import traceback
import sys
from std_msgs.msg import String,Header
from termcolor import colored
import re
import tf
from multiprocessing import Process
from kios.msg import InputDJI
import json

'''
A node to test the basic commands to send to drone 
(sometimes the android does not pick up the command that is why is in a loop)
(hint: check Error topic of that drone, sometimes there is an error and it does not run the command)
'''

'''
Ros Callback
'''
def rosCallback(data):
	pass

'''
Setup listeners/subscribers
Main loop!
'''	
def listener(dji_name = "Mavic_2_Enterprise_Kios"):
	subscriber=rospy.Subscriber("/"+ dji_name + "/Telemetry", String, rosCallback)
	publisher=rospy.Publisher("/"+ dji_name + "/Input", InputDJI, queue_size=10)
	rateHz = 1
	rate = rospy.Rate(rateHz)  
	rate.sleep()

	if not rospy.is_shutdown(): 			
		t = InputDJI()
		t.isBasicCommand=True
		t.basicCommand.command = t.basicCommand.takeOff
		publisher.publish(t)	
		print("Published: ")
		print(t)
		rate.sleep()
		rate.sleep()
		rate.sleep()
		rate.sleep()
		rate.sleep()
		t = InputDJI()
		t.isGimbalData=True
		t.gimbalControlData.roll=-1.0
		t.gimbalControlData.yaw=-1.0
		t.gimbalControlData.pitch=-90.0
		t.gimbalControlData.time=1
		publisher.publish(t)	
		print("Published: ")
		print(t)
		rate.sleep()
		rate.sleep()
		rate.sleep()
		rate.sleep()
		rate.sleep()
		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('dji_input', anonymous=True)
		private_param = rospy.get_param('~dji_name', "Mavic_2_Enterprise_Kios") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass