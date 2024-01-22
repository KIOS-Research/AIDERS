import logging
import rospy
from std_msgs.msg import String

logger = logging.getLogger(__name__)


# def rosCallback(data, args):

#     dji_name = args[0]
#     lidar_session = args[1]
#     data = data.data
#     json_data = json.loads(str(data))
#     pointCloudData = json_data["PointCloud:"]
#     views.LidarPointsAPIView.save_point_in_db(
#         pointCloudData, dji_name, lidar_session)

# def listener(dji_name="kios_mavic1g", lidar_session=None):
#     subscriber = rospy.Subscriber(
#         "/" + dji_name + "/PointCloud", String, rosCallback, (dji_name, lidar_session))
#     rospy.spin()


def buildMapPublisherSingleMessage(drone_name, data):
    publisher = rospy.Publisher("/" + drone_name + "/StartOrStopPointCloud", String, queue_size=10)
    t = String()
    t.data = data
    publisher.publish(t)
