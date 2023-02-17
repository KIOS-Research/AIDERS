import rospy
import traceback
import sys
from std_msgs.msg import String,Header
from termcolor import colored
from kios.msg import GpsInput, MissionDji, InputDJI, MissionCommandDJI
import json

def rosCallback(data):
	print(data)
	
def listener(dji_name = "Mavic_2_Enterprise_Kios"):
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
		s.missionCommand = s.start
		t.missionCommand = s
		if once:
			for i in range(1,2):
				k = GpsInput()
				k.latitude = 35.144159
				k.longitude = 33.412992
				k.altitude = 10
				k.speed = 2.5
				k.stayTime = 0
				k.photo= False
				t.gpsInput.append(k)

				k = GpsInput()
				k.latitude = 35.144159
				k.longitude = 33.412992
				k.altitude = 10
				k.speed = 2.5
				k.stayTime = 0
				k.photo= False
				t.gpsInput.append(k)
				
				k = GpsInput()
				k.latitude = 35.143789
				k.longitude = 33.413099
				k.altitude = 10
				k.speed = 2.5
				k.stayTime= 0
				k.photo = False
				t.gpsInput.append(k)

				k = GpsInput()
				k.latitude = 35.143363
				k.longitude = 33.412861
				k.altitude = 20
				k.speed = 2.5
				k.stayTime= 0
				k.photo = False
				t.gpsInput.append(k)

				k = GpsInput()
				k.latitude = 35.143756
				k.longitude = 33.412630
				k.altitude = 25
				k.speed = 2.5
				k.stayTime = 0
				k.photo= False
				t.gpsInput.append(k)
				
				k = GpsInput()
				k.latitude = 35.144387
				k.longitude = 33.413113
				k.altitude = 30
				k.speed = 2.5
				k.stayTime= 0
				k.photo = False
				t.gpsInput.append(k)

				k = GpsInput()
				k.latitude = 35.1446
				k.longitude = 33.4130
				k.altitude = 35
				k.speed = 2.5
				k.stayTime= 0
				k.photo = False
				t.gpsInput.append(k)

			publisher.publish(t)	
			print("Published: ")
			print(t)
			once = False
		
		rate.sleep()
		rate.sleep()
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
