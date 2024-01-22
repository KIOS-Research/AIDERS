import threading
from threading import Thread
#from kios.msg import InputDJI,Telemetry
from collections import deque
from subprocess import Popen
import time
import logging
from object_detection import src
import os
logger = logging.getLogger(src.__name__)
#import rospy
import random


class Controller(Thread):
	def __init__(self, export_fh=None, START_ROS_MASTER_BASH_SCRIPT = '/djiswarmcatkinws/start_ros_master.bash'
				 ,drone_name="kios_mavic1g", result_file="dji_ros", ros_ip="127.0.0.1"):
		Thread.__init__(self,)


		self.ros_stop = False
		self.ros_telemetry=None
		self.drone_name=drone_name
		self.altitude = deque([0], maxlen=30)
		self.gps = deque([35.14543793645089, 33.409062440422986], maxlen=30)
		self.export=export_fh
		self.timer = 0
		self.txt=None
		self.videoinfo_path = ''
		if self.export is not None:
			self.videoinfo_path = f'{self.export.folder_name}/{self.export.filename}_' \
								  f'{result_file}_{self.export.timetmp}.{self.export.extension}'
			txt = open(self.videoinfo_path, 'w')
			txt.write('Altitude,Longitude,Latitude\n')
			txt.close()
			self.txt = open(self.videoinfo_path, 'a')

	def run(self,):
		print("THREAD",threading.currentThread())

		try:

			rospy.init_node('dji_input',  log_level=rospy.DEBUG, disable_signals=True)
			print('ROSPY SHUTDOWN?',rospy.is_shutdown())
			logger.info('trying subscribing')
			print(self.drone_name)
			subscriber = rospy.Subscriber("/" + self.drone_name + "/Telemetry", Telemetry, self.rosCallback)
			rospy.spin()
			# print('SELF ROS STOP',self.ros_stop)
			# print('ROSPY SHUTDOWN?',rospy.is_shutdown())
			# while(not rospy.is_shutdown()):
			# 	time.sleep(2)
			# print('ROSPY SHUTDOWN?',rospy.is_shutdown())
			# print('EVAOSEN')
			# self.ros_stop = True
			print('rospy shut down')
		except rospy.ROSInterruptException:
			self.stop()
			pass
		self.stop()

	def writetofile(self):
		if self.txt is not None:
			if self.ros_telemetry != None:
				self.txt.write(str(self.ros_telemetry.altitude) + ',' + str(self.ros_telemetry.longitude) +
							   ',' + str(self.ros_telemetry.latitude) + '\n')


	def rosCallback(self,data):
		if not self.ros_stop:
			self.ros_telemetry = data
			if (time.time() - self.timer) >= 3:
				self.writetofile()
				self.timer = time.time()


	def stop(self):
		print('EMPIN POMPIN STO STOP')
		self.ros_stop=True
		if self.txt != None:
			self.txt.close()
		rospy.signal_shutdown('Shutting down Rospy')

		# self.join()

class rosservice():

	def __init__(self, START_ROS_MASTER_BASH_SCRIPT = 'roslaunch_start.sh',
				 drone_name="kios_mavic1g", result_file="dji_ros", ros_ip="127.0.0.1", stop=False):
		self.ros_ip = ros_ip
		os.environ['ROS_IP'] = self.ros_ip
		os.environ['ROS_MASTER_URI'] = f"http://{self.ros_ip}:11311"
		# START_ROS_MASTER_BASH_SCRIPT = '/djiswarmcatkinws/start_ros_master.bash'
		print(os.getcwd())
		self.START_ROS_MASTER_COMMAND =  'Scripts/' + START_ROS_MASTER_BASH_SCRIPT
		self.STOP_ROS_MASTER_COMMAND = self.START_ROS_MASTER_COMMAND.replace('start','stop')
		GIVE_PERMISSIONS_FOR_ROS_LAUNCH_BASH_SCRIPT = 'chmod +x ' + self.START_ROS_MASTER_COMMAND
		GIVE_PERMISSIONS_FOR_ROS_LAUNCH_STOP_BASH_SCRIPT = 'chmod +x ' + self.STOP_ROS_MASTER_COMMAND
		Popen(GIVE_PERMISSIONS_FOR_ROS_LAUNCH_BASH_SCRIPT, shell=True)
		Popen(GIVE_PERMISSIONS_FOR_ROS_LAUNCH_STOP_BASH_SCRIPT, shell=True)
		time.sleep(2)
		self.process=None
		if stop:
			self.ros_kill()
		else:
			self.ros_start()

	def ros_start(self):
		self.process = Popen(self.START_ROS_MASTER_COMMAND)
		print('self process',self.process)
		time.sleep(1)

	def ros_kill(self):
		process_stop = Popen(self.STOP_ROS_MASTER_COMMAND)
		time.sleep(1)
		process_stop.kill()
		time.sleep(1)

	def ros_stop(self):
		self.process.kill()
		time.sleep(1)
		process_stop = Popen(self.STOP_ROS_MASTER_COMMAND)
		time.sleep(1)
		process_stop.kill()
		time.sleep(1)
		# rospy.signal_shutdown('Shutting down Rospy')
