import threading
import requests
from datetime import datetime
import time


class DroneBuildMap:
    def __init__(self, drone, interval, netIp):
        self.drone = drone
        self.interval = interval
        self.running = True
        self.url = f"http://{netIp}:8888/postBuildMapImg/"

    def start(self):
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        image_filename = f"/pics/{self.drone.name}.jpg"
        with open(image_filename, "rb") as image_file:
            image_data = image_file.read()
        while self.running:
            # rename image file before posting it
            currentDateTime = datetime.now().time()
            formattedDateTime = currentDateTime.strftime("%H-%M-%S")
            new_filename = f"{self.drone.name}_{formattedDateTime}.jpg"
            files = {"image_file": (new_filename, image_data)}
            payload = {
                "image_name": new_filename,
                "drone_name": self.drone.name,
                "lat": self.drone.latitude,
                "lon": self.drone.longitude,
                "alt": self.drone.altitude,
                "bearing": self.drone.heading,
                "d_roll": 0,
                "d_pitch": 0,
                "d_yaw": 0,
                "g_roll": 0,
                "g_pitch": 0,
                "g_yaw": 0,
            }
            response = requests.post(self.url, data=payload, files=files)
            if response.status_code == 200:
                print(f"Request {self.url} successful!")
            else:
                print(f"Request {self.url} failed.")
            pass
            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self.thread.join()
