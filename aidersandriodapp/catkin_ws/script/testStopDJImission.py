import rospy
import traceback
import sys
from std_msgs.msg import String,Header
from termcolor import colored
from kios.msg import GpsInput, MissionDji, InputDJI, MissionCommandDJI
import json

def rosCallback(data):
	print(data)
	
def listener(dji_name = "spark_blue"):
	subscriber=rospy.Subscriber("/"+ dji_name + "/Error", String, rosCallback)
	publisher=rospy.Publisher("/"+ dji_name + "/Mission", MissionDji, queue_size=10)
	rateHz = 0.5
	rate = rospy.Rate(rateHz) 
	once = True
	rate.sleep()
	if not rospy.is_shutdown(): 			
		t = MissionDji()
		t.name="Test"
		t.header.stamp = rospy.get_rostime()
		t.header.frame_id = dji_name
		s = MissionCommandDJI()
		s.missionCommand = s.stop
		t.missionCommand = s

		publisher.publish(t)	
		print("Published: ")
		print(t)

		rate.sleep()
		rate.sleep()


'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('dji_input', anonymous=True)
		private_param = rospy.get_param('~dji_name', "spark_green") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
