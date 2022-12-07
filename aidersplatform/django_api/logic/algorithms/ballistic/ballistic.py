
import rospy
from std_msgs.msg import Int32


def publish_message(drone_name, data):
    publisher = rospy.Publisher(
        "/" + drone_name + "/WaterSampler", Int32, queue_size=10)
    t = Int32()
    t.data = data
    publisher.publish(t)
