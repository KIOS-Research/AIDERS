import json
import logging
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from multiprocessing import Process
from subprocess import PIPE, Popen
from threading import Thread

import GPUtil
import psutil
import rospy
from aiders import models, serializers, views
from aiders.models import Detection
from django.shortcuts import get_object_or_404
from numpy import byte
from rosbridge_msgs.msg import ConnectedClients
from std_msgs.msg import String

from logic.algorithms.weather_station import (weather_station_ros_publisher,
                                              weather_station_ros_subscriber)

from .algorithms.drone_movement import DroneMovement
from .algorithms.build_map import build_map_request_handler, img_georeference
from .algorithms.error_message_listener import drone_error_ros_listener
from .algorithms.jetson_data_listener import jetson_data_listener
from .algorithms.live_stream import capture_live_stream
from .algorithms.lora_station import lora_location_receiver
from .algorithms.mesh_monitoring import mesh_monitoring
from .algorithms.mission import mission_request_handler
from .algorithms.publish_from_mavlink import mavlink_ros_publisher
from .algorithms.safe_drones import calculations_safe_drones
from .algorithms.telemetry_listener import drone_telemetry_ros_listener
from .algorithms.water_collector import water_collector
from .Constants import Constants

logger = logging.getLogger(__name__)

drone_active_terminals = {}
all_active_processes_pids = []
buildMap_Port = int(Constants.API_PORT) + 1
buildMap_drone_pids_dict = {}


'''
This handler is responsible for detecting newly connected drones and starting all of the scripts
that are associated with that drone
'''


class DroneChangesHandler():
    def __init__(self):
        try:
            self.droneInit()
            self.listener()
        except rospy.ROSInterruptException:
            pass

    '''
    Uppon starting the platform, it is certain that no drone is currently joined (we just started it).
    Therefore, make sure that in case there are drones that were left "Connected" from the last time,
    disconnect them here. We shouldn't reach at this point, except in cases the platform crashed last time,
    and does not disconnect normally from the database
    '''

    def droneInit(self):
        models.Drone.objects.all().update(is_connected_with_platform=False,
                                          build_map_activated=False)
        models.Detection.objects.all().update(
            detection_status=models.Detection.DetectionStatusChoices.DETECTION_DISCONNECTED)

    def droneExistsButNotConnectedWithPlatform(self, drone_name):
        droneQuery = models.Drone.objects.filter(drone_name=drone_name)
        if (droneQuery.exists()):
            drone = get_object_or_404(droneQuery)
            if (not drone.is_connected_with_platform):
                return True
        return False

    def droneExistsAndConnectedWithPlatform(self, drone_name):
        droneQuery = models.Drone.objects.filter(drone_name=drone_name)
        if (droneQuery.exists()):
            drone = get_object_or_404(droneQuery)
            if (not drone.is_connected_with_platform):
                return False
        return True

    def getDronePK(self, droneName):
        droneQuery = models.Drone.objects.filter(drone_name=droneName)
        if (droneQuery.exists()):
            drone = get_object_or_404(droneQuery)
            return drone.pk
        return None

    def droneExists(self, drone_name):
        return True if self.getDronePK(drone_name) is not None else False

    def getOpPK(self, opName):
        opQuery = models.Operation.objects.filter(operation_name=opName)
        if (opQuery.exists()):
            op = get_object_or_404(opQuery)
            return op.pk
        return None
    # def connectedClientsListenerCallback(self, data):
    #     data = data.data
    #     print(data)

    def droneIdListenerCallback(self, data):
        data = data.data
        json_data = json.loads(str(data))
        drone_name = json_data["DroneName"]
        isConnected = json_data["Connected"]
        droneModel = json_data["Model"]
        cameraModel = json_data.get("cameraName")
        DroneIp = json_data.get("DroneIp")
        ballistic_available = json_data.get('Ballistic', False)
        lidar_available = json_data.get('Lidar', False)
        multispectral_available = json_data.get('Multispectral', False)
        water_sampler_available = json_data.get('WaterSampler', False)
        weather_station_available = json_data.get('WeatherStation', False)

        if isConnected == "True":
            if not self.droneExists(drone_name):
                droneObj = {
                    'drone_name': drone_name,
                    'model': droneModel,
                    'ip': DroneIp,
                    'operation': None,
                    'is_connected_with_platform': True,
                    'mission': None,
                    'camera_model': cameraModel,
                    'water_sampler_available': water_sampler_available,
                    'weather_station_available': weather_station_available,
                    'multispectral_available': multispectral_available,
                    'lidar_available': lidar_available,
                    'drone_movement_available': False,
                    'ballistic_available': ballistic_available,
                }
                try:
                    views.DroneList.save_drone_to_db(droneObj)
                except Exception as e:
                    print(e)
                self.client_handshake(json_data, drone_name)
                self.drone_advance(drone_name)
                print("\n\n**" + datetime.now().strftime(
                    "%d/%m/%Y %H:%M:%S") + "**" + "\n***NEW DRONE CREATED AND CONNECTED*** - **" + drone_name + "**")
                logger.info(
                    'New drone {} created and connected'.format(drone_name))
                # self.post_drone_obj_to_api(drone_id, droneModel, isConnected)

            elif self.droneExistsButNotConnectedWithPlatform(drone_name):
                # models.Drone.objects.filter(drone_name=drone_name).update(operation=None,is_connected_with_platform=True)
                models.Drone.objects.filter(drone_name=drone_name).update(
                    is_connected_with_platform=True, build_map_activated=False)

                self.client_handshake(json_data, drone_name)
                print("\n\n**" + datetime.now().strftime(
                    "%d/%m/%Y %H:%M:%S") + "**" + "\n***NEW DRONE CONNECTED*** - **" + drone_name + "**")
                logger.info('Drone {} connected'.format(drone_name))
                # self.post_drone_obj_to_api(drone_id, droneModel, isConnected)

            # We have this case whenever the platform is forcefully closed
            # We cannot possibly know when the user decides to forcefully stop the platform
            # Therefore, drone remains "connected" with the platform although it is not
            elif self.droneExistsAndConnectedWithPlatform(drone_name):
                self.client_handshake(json_data, drone_name)
                print("\n\n**" + datetime.now().strftime(
                    "%d/%m/%Y %H:%M:%S") + "**" + "\n***DRONE CHECKED FOR CONNECTION*** - **" + drone_name + "**")
                logging.info(
                    'Drone {} checked for connection'.format(drone_name))

            self.start_telemetry_listener_for_this_drone(drone_name)
            self.start_capturing_live_feed_from_this_drone(drone_name)

        elif isConnected == "False":
            print("\n\n**" + datetime.now().strftime(
                "%d/%m/%Y %H:%M:%S") + "**" + "\n***DRONE DISCONNECTED*** - **" + drone_name + "**")

            models.Drone.objects.filter(drone_name=drone_name).update(
                is_connected_with_platform=False, build_map_activated=False)
            models.Detection.objects.filter(drone__drone_name=drone_name).update(
                detection_status=models.Detection.DetectionStatusChoices.DETECTION_DISCONNECTED)
            models.DetectionSession.objects.filter(
                drone__drone_name=drone_name).update(is_active=False)
            models.LiveStreamSession.objects.filter(
                drone__drone_name=drone_name).update(is_active=False)

    def listener(self):
        # Subscribing to droneIDs topic
        rospy.Subscriber("/droneIds", String, self.droneIdListenerCallback)
        # rospy.Subscriber("/connected_clients", ConnectedClients, self.connectedClientsListenerCallback)
        logger.info('Ready and waiting for drones.')

    def start_telemetry_listener_for_this_drone(self, droneName):

        print("====STARTING TELEMETRY LISTENER FOR DRONE: " +
              droneName + " ===========")

        sys.argv = [
            "", droneName
        ]

        if "mavlink" in droneName:
            mvthread = Thread(target=mavlink_ros_publisher.main)
            mvthread.start()
        threadName = self.start_telemetry_listener_for_this_drone.__name__ + "_" + droneName
        if (not threadStarted(threadName)):
            thread = Thread(target=drone_telemetry_ros_listener.main,
                            args=(droneName,), daemon=True)
            thread.name = self.start_telemetry_listener_for_this_drone.__name__ + "_" + droneName
            thread.start()
        thread = Thread(target=drone_error_ros_listener.main,
                        args=(droneName,), daemon=True)
        thread.start()

    def start_capturing_live_feed_from_this_drone(self, droneName):
        print("====STARTING CAPTURING LIVE FEED FOR DRONE: " +
              droneName + " ===========")
        threadName = self.start_capturing_live_feed_from_this_drone.__name__ + "_" + droneName
        liveStreamSessionId = models.LiveStreamSession.objects.filter(
            drone__drone_name=droneName).last()
        if (threadStarted(threadName) == False or liveStreamSessionId == None):
            thread = Thread(target=capture_live_stream.main,
                            args=(droneName,),  daemon=True)
            thread.name = self.start_capturing_live_feed_from_this_drone.__name__ + "_" + droneName
            thread.start()
        else:
            models.LiveStreamSession.objects.filter(
                id=liveStreamSessionId.id).update(is_active=True)

    def client_handshake(self, json_data, droneName):
        handshake = rospy.Publisher(
            "/" + droneName + "/handshake", String, queue_size=10)
        t = String()
        t.data = "1"
        handshake.publish(t)

    def drone_advance(drone_name):
        subscriberThread = views.MyThread(
            name=drone_name+'WeatherSubscriber', target=weather_station_ros_subscriber.main, args=(drone_name, 'MainWeatherSubscriber'))
        subscriberThread.start()
        p = Thread(name=drone_name+'WaterSensor', target=water_collector.check_sensor,
                   args=('drone_name',))
        p.start()
        p = Thread(name=drone_name+'TerminalHardware', target=jetson_data_listener.start_listening,
                   args=(drone_name,))
        p.start()
        p = Thread(name=drone_name+'DroneMovement', target=DroneMovement.start_listening,
                   args=(drone_name,))
        p.start()


'''
This function takes as a parameter the port of the process, and the name of the process.
It will then kill all of the processes with that name and that port.
For example, the drone api process is called "python3" and the port is 5000.
'''


def killProcess(port):
    process = Popen(
        ["lsof", "-i", ":{0}".format(port)], stdout=PIPE, stderr=PIPE)
    stdout, stderr = process.communicate()
    for process in str(stdout.decode("utf-8")).split("\n")[1:]:

        data = [x for x in process.split(" ") if x != '']
        if (len(data) <= 1):
            continue
        try:
            os.killpg(int(data[1]), signal.SIGKILL)
        except:
            pass


def threadStarted(threadName):
    for thread in threading.enumerate():
        if thread.name == threadName:
            return True
    return False


'''
Checks if a port is occupied by a process or not
'''


def isPortOccupied(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = True
    try:
        sock.bind(("0.0.0.0", port))
        # print("Port " + str(port) + " is NOT in use")
        result = False
    except:
        result = True
        # print("Port " + str(port) + " is in use")
    sock.close()
    return result


'''
Terminates all the processes that might have been left open from previous sessions
'''


def terminateAllProcesses():
    processName = b'python'
    killProcess(Constants.API_PORT)
    killProcess(Constants.WEB_SERVER_PORT)
    killProcess(Constants.ROS_MASTER_PORT)
    killProcess(Constants.ROS_MASTER_PORT_2)
    killProcess(Constants.ROS_MASTER_DISCOVERY_PORT)

    builMapPort = buildMap_Port
    # The ports for build map are given dynamically. For this reason we keep stopping the processes that use the incremented build map port
    while isPortOccupied(builMapPort):
        killProcess(builMapPort)
        builMapPort += 1


'''
Enables new terminal for that command
'''


def enableSeparateTerminal(command):
    isCommandString = isinstance(command, str)
    if (isCommandString):
        # prefix = 'gnome-terminal -x '
        prefix = 'gnome-terminal --disable-factory -- '
        command = prefix + command
    else:
        command.insert(0, '--')
        command.insert(0, '--disable-factory')
        command.insert(0, 'gnome-terminal')

    return command


def create_file_if_not_exists(txt_path):
    if (not os.path.exists(txt_path)):
        with open(txt_path, 'w') as fp:
            pass


def create_dir_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)


class Drone_disconnected(object):
    drones_online = []

    def get_drones(self):
        return self.drones_online

    def set_drones(self, drones):
        self.drones_online = drones

    def get_if_drone_was_online(self, drone):
        if drone in self.drones_online:
            return True
        else:
            return False


Drones_online_class_list = Drone_disconnected()


def drone_disconnect(data, args):
    Drones_online_class_list = args[0]
    droneIP = []
    for client in data.clients:
        droneIP.append(client.ip_address)
    for drone in Drones_online_class_list.get_drones():
        if not drone in droneIP:
            try:
                drone_object = models.Drone.objects.get(ip=drone)
                drone_object.update(
                    is_connected_with_platform=False, build_map_activated=False)
                models.Detection.objects.get(drone=drone_object.pk).update(detection_status=Detection.DetectionStatusChoices.DETECTION_DISCONNECTED,
                                                                           detection_type_str=Detection.DetectionTypeStrChoices.NO_ACTIVE_DETECTOR,
                                                                           detection_model=Detection.DetectionModelChoices.NO_ACTIVE_MODEL)
                logger.warning('Drone {} forcefully disconnected'.format(
                    models.Drone.objects.get(ip=drone).drone_name))
            except Exception as e:
                logger.warning('Drone object does not have the IP {}'.format(
                    models.Drone.objects.get(ip=drone).drone_name))
    Drones_online_class_list.set_drones(droneIP)


def listToString(s):
    # initialize an empty string
    str1 = ""
    # traverse in the string
    for ele in s:
        if str1 != "":
            str1 = str1+', '+str(ele)
        else:
            str1 = str1+str(ele)
    # return string
    return str1


def drone_disconnect_handler():
    time.sleep(1)
    rospy.Subscriber("/connected_clients", ConnectedClients, drone_disconnect,
                     (Drones_online_class_list,))  # Subscribing to droneIDs topic
    logger.info("Drone Disconnect handler started.")


_last_net_io_meta = None


def net_io_usage():
    global _last_net_io_meta
    net_counters = psutil.net_io_counters()
    tst = time.time()

    send_bytes = net_counters.bytes_sent
    recv_bytes = net_counters.bytes_recv
    if _last_net_io_meta is None:
        _last_net_io_meta = (send_bytes, recv_bytes, tst)
        return None

    last_send_bytes, last_recv_bytes, last_time = _last_net_io_meta
    delta_time = tst - last_time
    recv_speed = (recv_bytes - last_recv_bytes) / delta_time
    send_speed = (send_bytes - last_send_bytes) / delta_time
    _last_net_io_meta = (send_bytes, recv_bytes, tst)
    dict_data = {'recv_speed': recv_speed, 'send_speed': send_speed}
    return dict_data


_last_disk_io_meta = None


def disk_io_usage():
    global _last_disk_io_meta
    disk_counters = psutil.disk_io_counters()
    tst = time.time()

    read_bytes = disk_counters.read_bytes
    write_bytes = disk_counters.write_bytes
    if _last_disk_io_meta is None:
        _last_disk_io_meta = (read_bytes, write_bytes, tst)
        return None

    last_read_bytes, last_write_bytes, last_time = _last_disk_io_meta
    delta_time = tst - last_time
    write_speed = (write_bytes - last_write_bytes) / delta_time
    read_speed = (read_bytes - last_read_bytes) / delta_time
    _last_disk_io_meta = (read_bytes, write_bytes, tst)
    dict_data = {'write_speed': write_speed, 'read_speed': read_speed}
    return dict_data


def data_usage_handler():
    time.sleep(10)

    UPDATE_DELAY = 1  # in seconds

    logger.info("Monitoring data usage handler started.")
    while True:
        l1, l2, l3 = psutil.getloadavg()
        cpu_usage = (l3/os.cpu_count()) * 100
        cpu_core_usage = psutil.cpu_percent(interval=1, percpu=True)
        data = 0
        gpu_usage = 0
        gpu_memory = 0
        gpu_temp = 0
        for cpuData in cpu_core_usage:
            data += cpuData
        cpu_usage = round(data/len(cpu_core_usage), 1)
        cpu_core_usage = listToString(cpu_core_usage)
        ram_usage = psutil.virtual_memory().percent
        swap_memory = psutil.swap_memory().percent
        cpu_temp = 0
        for cpuTemp in psutil.sensors_temperatures()['coretemp'][1:]:
            cpu_temp = cpu_temp+cpuTemp.current
        cpu_temp = cpu_temp/(len(psutil.sensors_temperatures()['coretemp'])-1)
        # cpu_temp = psutil.sensors_temperatures()['coretemp'][0].current
        temp = list(psutil.sensors_temperatures().values())[0][0].current

        try:
            battery_percentage = psutil.sensors_battery().percent
        except:
            battery_percentage = None
        try:
            GPUs = GPUtil.getGPUs()
            for GPU in GPUs:
                gpu_usage = gpu_usage+float(GPU.load)*100
                gpu_memory = gpu_memory+float(GPU.memoryUtil)*100
            gpu_usage/len(GPUs)
            gpu_memory/len(GPUs)
            gpu_temp = list(psutil.sensors_temperatures().values())[
                1][0].current
        except Exception as e:
            pass

        dict_data = net_io_usage()
        if(dict_data != None):
            send_speed = dict_data["send_speed"]
            recv_speed = dict_data["recv_speed"]
        else:
            send_speed = 0
            recv_speed = 0

        dict_data = disk_io_usage()
        if(dict_data != None):
            read_speed = dict_data["read_speed"]
            write_speed = dict_data["write_speed"]
        else:
            read_speed = 0
            write_speed = 0
        views.system_monitoring_save_to_db(cpu_usage, cpu_core_usage, cpu_temp, gpu_usage, gpu_memory, gpu_temp, ram_usage,
                                           swap_memory, temp, send_speed, recv_speed, send_speed+recv_speed, read_speed, write_speed, battery_percentage)

        time.sleep(UPDATE_DELAY)


def initializeDetectionConfigPaths():
    def pairs(data):
        for k, v in data.items():
            if isinstance(v, dict):
                yield from pairs(v)
            else:
                # print("KEY: ", k)
                # print("VAL: ", v)

                # print("data now::::", data)
                if isinstance(v, str) and 'cfg/' in v:
                    t = {
                        'current_k_v': '"{}": "{}"'.format(k, v),
                        'correct_k_v': '"{}": "{}"'.format(k, v)
                    }
                    yield '"{}": "{}"'.format(k, v)
    # Read in the file
    with open(Constants.DETECTION_CONFIG_JSON_FILE, 'r') as file:
        filedata = file.read()

    for item in pairs(filedata):
        print(item)
        filedata = filedata.replace(
            '"cfg/', '"{}'.format(Constants.DETECTION_CONFIG_DIR))
    # Replace the target string

    # Write the file out again
    with open(Constants.DETECTION_CONFIG_JSON_FILE, 'w') as file:
        file.write(filedata)

    exit(1)
    # with open(Constants.DETECTION_CONFIG_JSON_FILE, "w") as jsonFile:
    #     json.dump(data, jsonFile)


def test_gui():
    import tkinter as tk
    window = tk.Tk()

    window.title("Welcome to AIDERS app")

    window.geometry('600x600')

    lbl = tk.Label(window, text="Hello")

    lbl.grid(column=0, row=0)

    btn = tk.Button(window, text="Click Me")

    btn.grid(column=1, row=0)

    window.mainloop()


def updateIPForOfflineMaps():
    # Open the json file, read it and update the variable of the "tiles" array
    with open(Constants.OFFLINE_DEMO_MAP_TILEJSON_PATH, "r") as jsonFile:
        data = json.load(jsonFile)
        # Get the IP that is currently reported in tiles.json
        # For example, from the following url we will get the 127.0.0.1
        # "http://127.0.0.1:8000/static/offline-maps/demo-map/tiles/{z}/{x}/{y}.pbf"
        # and replace it with the current network IP

        start = 'http://'
        end = ':' + os.environ.get('DJANGO_PORT')
        currentURL = data["tiles"][0]
        currentIP = currentURL[currentURL.find(
            start) + len(start):currentURL.rfind(end)]
        newURL = currentURL.replace(currentIP, Constants.NET_IP)
        data["tiles"][0] = newURL

    # Write the file out again
    with open(Constants.OFFLINE_DEMO_MAP_TILEJSON_PATH, "w") as jsonFile:
        json.dump(data, jsonFile, indent=4)


def start():
    # test_gui()
    print("\n\n============LAUNCHING THE BACKEND OF THE PLATFORM FROM DOCKER===========\n\n")
    # exit(1)
    # terminateAllProcesses()  # Let's make sure all of the processes are killed before proceeding to re-create them
    # initializeDetectionConfigPaths()
    # signal.signal(signal.SIGINT, signal_handler)
    create_dir_if_not_exists(Constants.MESH_RESULTS_FOLDER_DIR)

    # p = Popen(Constants.START_ROS_MASTER_COMMAND, preexec_fn=os.setpgrp)
    # all_active_processes_pids.append(p.pid)
    #
    #
    #
    # p = Popen(Constants.START_ROS_MASTER_DISCOVERY_COMMAND, preexec_fn=os.setpgrp)
    # all_active_processes_pids.append(p.pid)
    #
    # time.sleep(1)
    #
    # p = Popen(Constants.START_ROS_MASTER_SYNC_COMMAND, preexec_fn=os.setpgrp)
    # all_active_processes_pids.append(p.pid)

    # time.sleep(1)
    # p = Thread(target=weather_station_ros_publisher.main)
    # sys.argv = Constants.START_WEATHER_DATA_PUBLISHER_SCRIPT[1:]
    # p.start()

    # time.sleep(1)

    # p = Thread(target=weather_station_ros_subscriber.main)
    # p.start()

    time.sleep(1)

    t = Thread(target=drone_disconnect_handler, name="Disconnect Checker")
    t.start()

    time.sleep(1)

    t = Thread(target=data_usage_handler, name="Server Statistics")
    t.start()

    time.sleep(1)

    t = Thread(target=mavlink_ros_publisher.tryconnect)
    t.start()

    time.sleep(1)

    p = Process(target=mesh_monitoring.main)
    sys.argv = Constants.START_MESH_MONITORING_SCRIPT_COMMAND[1:]
    p.start()

    time.sleep(1)

    p = Thread(target=lora_location_receiver.receiveLocations,
               args=(Constants.LORA_RECEIVER_DEVICE_SYMLINK_PATH,))
    p.start()
    DroneChangesHandler()
    rospy.spin()


def main():
    start()
