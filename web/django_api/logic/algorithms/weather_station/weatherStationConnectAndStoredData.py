#!/usr/bin/env python
import logging
import time
from aiders import views
from aiders.models import WeatherStation
import serial

logger = logging.getLogger(__name__)


class Trisonica(object):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.port = port
        self.baud = baud

        self.speed = None
        self.speed2d = None
        self.direction = None
        self.northsouth = None
        self.westeast = None
        self.updown = None
        self.temperature = None
        self.pressure = None
        self.humidity = None
        self.pitch = None
        self.roll = None
        self.heading = None
        self.levelx = None
        self.levely = None
        self.levelz = None

        logger.info("Connecting to port %s" % self.port)
        try:
            self.connection = serial.Serial(self.port, self.baud, timeout=0.01)
            self.connection.flush()
        except serial.serialutil.SerialException as e:
            logger.info("No weather station plugged on this PC. Waiting...")
            self.keepRetrying()
        time.sleep(1)
        logger.info("Weather station connected!")

    def main(self):
        logger.info("A weather station is plugged in on this PC! Will now start storing data")
        while 1:
            try:
                data = self.connection.readline()
            except serial.serialutil.SerialException as e:
                logger.warning("Seems like weather station was disconnected!")
                logger.info("Waiting for weather station to reconnect.")
                self.connection.close()
                data = self.keepRetrying()
            if data is not None and len(data) > 10:
                self.updateData(data)
                self.storeDataToDatabase()
        self.connection.close()

    def updateData(self, data):
        print(data, flush=True)
        try:
            self.speed = float(data.decode().split("S ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.speed2d = float(data.decode().split("S2 ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.direction = float(data.decode().split("D ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.northsouth = float(data.decode().split("U ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.westeast = float(data.decode().split("V ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.updown = float(data.decode().split("W ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.temperature = float(data.decode().split("T ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.pressure = float(data.decode().split("P ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.humidity = float(data.decode().split("H ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.pitch = float(data.decode().split("P ")[2].lstrip().split(" ")[0])
        except:
            try:
                self.pitch = float(data.decode().split("PI ")[1].lstrip().split(" ")[0])
            except:
                pass
        try:
            self.roll = float(data.decode().split("RO ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.heading = float(data.decode().split("MD ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.levelx = float(data.decode().split("AX ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.levely = float(data.decode().split("AY ")[1].lstrip().split(" ")[0])
        except:
            pass
        try:
            self.levelz = float(data.decode().split("AZ ")[1].lstrip().split(" ")[0])
        except:
            pass
    def storeDataToDatabase(self):
        try:
            WeatherStation.objects.create(
                    wind_speed=self.speed,
                    wind_direction=self.direction,
                    temperature=self.temperature,
                    pressure=self.pressure,
                    humidity=self.humidity,
                    heading=self.heading,
                    operation=None,
                    drone=None,
                )
        except Exception as e:
            logger.warning("Could not store Weather data to database!")
    def keepRetrying(self):
        while 1:
            time.sleep(1)
            try:
                self.connection = serial.Serial(self.port, self.baud, timeout=0.01)
                self.connection.flush()
                data = self.connection.readline()
                logger.info("Weather station reconnected!")
                break
            except Exception as e:
                continue
        return data

def main():
    # TODO: Change port to custom dev rule port
    port = "/dev/weatherStation"
    baud = 115200
    trisonica = Trisonica(port=port, baud=baud)
    trisonica.main()

if __name__ == "__main__":
    main()
