#!/usr/bin/env python

# Author: Andreas
# Email: aanast01@ucy.ac.cy
# Date :26/03/2019

# license removed for brevity
import rospy
import traceback
import sys
from termcolor import colored
from std_msgs.msg import String,Header
from kios.msg import Stream
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
	print(data)

'''
Setup listeners/subscribers
Main loop!
'''	
def listener(dji_name = "spark_blue"):
	subscriber=rospy.Subscriber("/"+ dji_name + "/Error", String, rosCallback)
	publisher=rospy.Publisher("/"+ dji_name + "/Stream", Stream, queue_size=10)
	rateHz = 1
	rate = rospy.Rate(rateHz)  
	rate.sleep()

	if not rospy.is_shutdown():
		t = Stream()
		t.command = t.sendIP
		publisher.publish(t)	
		print("Published: ")
		print(t)
		rate.sleep()

		t = Stream()
		t.command = t.start
		publisher.publish(t)	
		print("Published: ")
		print(t)
		rate.sleep()
		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('dji_input', anonymous=True)
		private_param = rospy.get_param('~dji_name', "spark_blue") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass