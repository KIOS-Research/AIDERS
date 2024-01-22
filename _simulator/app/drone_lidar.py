import csv
import json
import threading
import rospy
from std_msgs.msg import String

NUMBER_OF_POINTS_PER_MESSAGE = 5000
NUMBER_OF_MESSAGES_PER_SECOND = 10


class DroneLidar:
    def __init__(self, _drone):
        self.drone = _drone
        self.running = True
        self.rosPublisher = rospy.Publisher(f"/{_drone.name}/PointCloud", String, queue_size=10)
        self.rate = rospy.Rate(NUMBER_OF_MESSAGES_PER_SECOND)

    def start(self):
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def readPointsFomCsvFile(self, _filePath):
        data = []
        with open(_filePath, "r") as csvFile:
            csvReader = csv.reader(csvFile)
            for row in csvReader:
                try:
                    x, y, z, red, green, blue, intensity = map(float, row[:7])
                    data.append((x, y, z, intensity, red, green, blue))
                except:
                    # Skip the first row or if there is a wrong float
                    pass
        return data

    def loop(self):
        filePath = "/lidarPoints/pointSessionInLab.csv"
        csv_data = self.readPointsFomCsvFile(filePath)

        index = 0
        while self.running:
            lidarPointMessage = {
                "PointCloud": [
                    {
                        "x": x,
                        "y": y,
                        "z": z,
                        "intensity": intensity,
                        "red": red,
                        "green": green,
                        "blue": blue,
                    }
                    for x, y, z, intensity, red, green, blue in csv_data[index : index + NUMBER_OF_POINTS_PER_MESSAGE]
                ]
            }
            self.rosPublisher.publish(json.dumps(lidarPointMessage))

            index = (index + NUMBER_OF_POINTS_PER_MESSAGE) % len(csv_data)
            self.rate.sleep()

    def stop(self):
        self.running = False
        self.thread.join()
