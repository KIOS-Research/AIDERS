#!/usr/bin/env python
import sqlite3
import glob
import rospy
import time
import datetime
from std_msgs.msg import String
from trisonica_ros.msg import trisonica_msg

db_url = '/aiders/jetson/catkin_ws/mydatabase1.db'
droneName = "matrice210v2"
drone_id = 1
weather_id = 1


def sql_table_weather_data():
    try:
        sqliteConnection = sqlite3.connect(db_url)
        sqliteConnection.text_factory = str
        cursor = sqliteConnection.cursor()
        cursor.execute("CREATE TABLE weather_table(weather_id INTEGER PRIMARY KEY AUTOINCREMENT, wind_speed text, wind_speed2d text,northsouth text,westeast text,updown text,temperature text,pitch text,roll text,humidity text,heading text,wind_direction text, pressure text,drone_id INTEGER,mission_id INTEGER,CONSTRAINT fk_weather FOREIGN KEY (drone_id) REFERENCES drone_table(drone_id) ON DELETE CASCADE,CONSTRAINT fk_weathert FOREIGN KEY (mission_id) REFERENCES mission_table(mission_id) ON DELETE CASCADE)")
        print("Weather table created")
        sqliteConnection.commit()
        cursor.close()

    except sqlite3.Error as error:
        print("Failed to create weather table", error)

    finally:
        if sqliteConnection:
            sqliteConnection.close()
            print("the sqlite connection is closed")


def mission_callback(data):
    global drone_id
    json_data = data.data
    mission_id = json_data["mission_id"]
    ros_timestamp = json_data["ros_timestamp"]


def drone_callback(data):
    global mission_id
    json = data.data
    drone_id = json["drone_id"]
    # is_connected= json["connected"]  see starter_Script from platform


def callback(data):
    global drone_id
    drone_id = 1
    mission_id = 1
    wind_speed = data.speed
    wind_speed2d = data.speed2d
    wind_direction = data.direction
    pressure = data.pressure
    northsouth = data.northsouth
    westeast = data.westeast
    updown = data.updown
    temperature = data.temperature
    pitch = data.pitch
    roll = data.roll
    humidity = data.humidity
    heading = data.heading

    try:
        sqliteConnection = sqlite3.connect(db_url)
        sqliteConnection.text_factory = str
        cursor = sqliteConnection.cursor()
        ts = time.time()
        tstamp = datetime.datetime.fromtimestamp(
            ts).strftime('%Y-%m-%d %H:%M:%S')
        print("Connected to SQLite")
        sqlite_insert_blob_query = """ INSERT INTO weather_table                                 (wind_speed,wind_speed2d,northsouth,westeast,updown,temperature,pitch,roll,humidity,heading,wind_direction,pressure,mission_id,drone_id,ros_timestamp) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)"""
        # Convert data into tuple format
        data_tuple = (wind_speed, wind_speed2d, northsouth, westeast, updown, temperature, pitch,
                      roll, humidity, heading, wind_direction, pressure, mission_id, drone_id, tstamp)
        cursor.execute(sqlite_insert_blob_query, data_tuple)
        sqliteConnection.commit()
        print(
            "Weather data from our weather station inserted successfully into mission_table")
        cursor.close()

    except sqlite3.Error as error:
        print("Failed to insert weather data into sqlite table", error)

    finally:
        if sqliteConnection:
            sqliteConnection.close()
            print("the sqlite connection is closed")


def listener():
    rospy.init_node('listener_for_weather_data', anonymous=True)
    # flightSub = rospy.Subscriber("/" + droneName + "/Mission", String , mission_callback) # Topic reading from George's talker
    # droneSub=  rospy.Subscriber("/" + droneName + "/droneIds", String , drone_callback) # Topic reading from George's talker
    trisonica_sub = rospy.Subscriber('/trisonica', trisonica_msg, callback)
    rospy.spin()


if __name__ == '__main__':
    # sql_table_weather_data()
    listener()
