# !/usr/bin/env python

import sys

from past.builtins import raw_input


def show_exception_and_exit(exc_type, exc_value, tb):
    import traceback
    traceback.print_exception(exc_type, exc_value, tb)
    raw_input("Press key to exit.")
    sys.exit(-1)
sys.excepthook = show_exception_and_exit


import threading

import rospy
from aiders.models import Telemetry
from std_msgs.msg import Bool
from kios.msg import BuildMap

def buildMapPublisherSingleMessageMultispectral(drone_name, data, overlap):
	publisher = rospy.Publisher("/" + drone_name + "/MultispectralBuildMapRequest", BuildMap, queue_size=10)
	t = BuildMap()
	t.buildmap = data
	t.overlap = int(overlap)
	publisher.publish(t)

def buildMapPublisherSingleMessage(drone_name, data, overlap):
	publisher = rospy.Publisher("/" + drone_name + "/BuildMapRequest", BuildMap, queue_size=10)
	t = BuildMap()
	t.buildmap = data
	t.overlap = int(overlap)
	publisher.publish(t)

def buildMapPublisher(thread, drone_name, photo_capture_frequency_if_drone_is_still):
	rateHz = photo_capture_frequency_if_drone_is_still
	rate = rospy.Rate(rateHz)

	publisher = rospy.Publisher("/" + drone_name + "/BuildMapRequest", Bool, queue_size=10)
	while not rospy.is_shutdown():
		t = Bool()
		t.data = True
		publisher.publish(t)
		rate = adjustRate(drone_name, photo_capture_frequency_if_drone_is_still)
		print("Send request to: " + drone_name + "/BuildMapRequest")
		if thread.stopped():
			print('THREAD IS STOP')
			break
		rate.sleep()

def adjustRate(drone_name, photo_capture_frequency_if_drone_is_still):
	drone_data = getDroneData(drone_name)
	alt = drone_data.alt
	speed = drone_data.velocity
	if (alt > 0):
		if speed>=2:
			rateHz = (1.4 * speed ) / alt
			rate = rospy.Rate(rateHz)
		else:
			rateHz = photo_capture_frequency_if_drone_is_still
			rate = rospy.Rate(rateHz)
		return rate


def getDroneData(drone_name):

	drone_data = Telemetry.objects.filter(drone__drone_name = drone_name).last()
	return drone_data



def main(dji_name, data):
	try:
		rospy.init_node("dji_input", anonymous=True)
		buildMapPublisherSingleMessage(dji_name, data)
	except rospy.ROSInterruptException:
		pass
	finally:
		pass

if __name__ == '__main__':
	main()

