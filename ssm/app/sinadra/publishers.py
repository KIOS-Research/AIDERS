import rospy
from std_msgs.msg import Bool, Float32
import numpy as np

def rosInit():
    rospy.init_node('SSM')

def areaDistanceToCatastropyEpicenter(_distaceValue):
    publish = rospy.Publisher(f"/area_distance_to_catastrophy_epicenter", Float32, queue_size=10)
    msg = Float32()
    msg.data = np.float32(_distaceValue)
    publish.publish(msg)

def denseBuildingArea(_denseValue):
    publish = rospy.Publisher(f"/control_station/dense_building_area", Bool, queue_size=10)
    msg = Bool()
    msg.data = _denseValue
    publish.publish(msg)

def riskOfFireAndExplosionsInArea(_riskValue):
    publish = rospy.Publisher(f"/control_station/risk_of_fire_and_explosions_in_area", Bool, queue_size=10)
    msg = Bool()
    msg.data = _riskValue
    publish.publish(msg)

def highestTemperatureInArea(_temperatureValue):
    publish = rospy.Publisher(f"/drone/highest_temperature_in_area", Float32, queue_size=10)
    msg = Float32()
    msg.data = np.float32(_temperatureValue)
    publish.publish(msg)
