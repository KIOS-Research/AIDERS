
import rospy
from std_msgs.msg import Int32

water_sampler_under_water = False


def publish_message(drone_name, data):
    publisher = rospy.Publisher(
        "/" + drone_name + "/WaterSampler", Int32, queue_size=10)
    t = Int32()
    t.data = data
    publisher.publish(t)


def watersamplerSensorCB(reading):
    waterCapacitanceLimit = 400
    global water_sampler_under_water
    if reading.data < waterCapacitanceLimit:
        water_sampler_under_water = True
    else:
        water_sampler_under_water = False


def check_sensor(drone_name):
    try:
        rospy.Subscriber('/matrice300/wsmSensor', Int32, watersamplerSensorCB)
        rospy.spin()
    except Exception as e:
        pass
