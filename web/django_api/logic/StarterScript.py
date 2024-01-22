import contextlib
import json
import logging
import os
import signal
import socket
import sys
import threading
import time
from subprocess import PIPE, Popen

import GPUtil
import psutil
from aiders import models, views
from logic.algorithms.weather_station import weatherStationConnectAndStoredData

from .algorithms.mesh_monitoring import mesh_monitoring
from .Constants import Constants

logger = logging.getLogger(__name__)

buildMap_Port = int(Constants.API_PORT) + 1


"""
This handler is responsible for detecting newly connected drones and starting all of the scripts
that are associated with that drone
"""


class mainMonitorHandler:
    def __init__(self):
        self.dbInit()
        print("[ \033[32m\u2714\033[0m ] Initialize DataBase.")

    """
    Upon starting the platform, it is certain that no drone is currently joined (we just started it).
    Therefore, make sure that in case there are drones that were left "Connected" from the last time,
    disconnect them here. We shouldn't reach at this point, except in cases the platform crashed last time,
    and does not disconnect normally from the database
    """

    def dbInit(self):
        for drone in models.Drone.objects.all():
            drone.is_connected_with_platform = False
            drone.build_map_activated = False
            drone.save()
        models.Detection.objects.all().update(detection_status=models.Detection.DetectionStatusChoices.DETECTION_DISCONNECTED)
        for device in models.Device.objects.all():
            device.is_connected_with_platform = False
            device.save()
        for baloraMaster in models.BaloraMaster.objects.all():
            baloraMaster.is_connected_with_platform = False
            baloraMaster.save()

"""
This function takes as a parameter the port of the process, and the name of the process.
It will then kill all of the processes with that name and that port.
For example, the drone api process is called "python3" and the port is 5000.
"""


def killProcess(port):
    process = Popen(["lsof", "-i", ":{0}".format(port)], stdout=PIPE, stderr=PIPE)
    stdout, stderr = process.communicate()
    for process in str(stdout.decode("utf-8")).split("\n")[1:]:

        data = [x for x in process.split(" ") if x != ""]
        if len(data) <= 1:
            continue
        with contextlib.suppress(Exception):
            os.killpg(int(data[1]), signal.SIGKILL)


def threadStarted(threadName):
    return any(thread.name == threadName for thread in threading.enumerate())


"""
Checks if a port is occupied by a process or not
"""

def isPortOccupied(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = True
    try:
        sock.bind(("0.0.0.0", port))
        # print("Port " + str(port) + " is NOT in use")
        result = False
    except Exception:
        result = True
    sock.close()
    return result


"""
Terminates all the processes that might have been left open from previous sessions
"""

def terminateAllProcesses():
    processName = b"python"
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


def create_dir_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def listToString(s):
    # initialize an empty string
    str1 = ""
    # traverse in the string
    for ele in s:
        str1 = f"{str1}, {str(ele)}" if str1 != "" else str1 + str(ele)
    # return string
    return str1

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
    return {"recv_speed": recv_speed, "send_speed": send_speed}


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
    return {"write_speed": write_speed, "read_speed": read_speed}


def data_usage_handler():
    time.sleep(10)

    UPDATE_DELAY = 1  # in seconds

    logger.info("Monitoring data usage handler started.")
    while True:
        l1, l2, l3 = psutil.getloadavg()
        cpu_usage = (l3 / os.cpu_count()) * 100
        cpu_core_usage = psutil.cpu_percent(interval=1, percpu=True)
        gpu_usage = 0
        gpu_memory = 0
        gpu_temp = 0
        data = sum(cpu_core_usage)
        cpu_usage = round(data / len(cpu_core_usage), 1)
        cpu_core_usage = listToString(cpu_core_usage)
        ram_usage = psutil.virtual_memory().percent
        swap_memory = psutil.swap_memory().percent
        cpu_temp = 0
        for cpuTemp in psutil.sensors_temperatures()["coretemp"][1:]:
            cpu_temp = cpu_temp + cpuTemp.current
        cpu_temp = cpu_temp / (len(psutil.sensors_temperatures()["coretemp"]) - 1)
        # cpu_temp = psutil.sensors_temperatures()['coretemp'][0].current
        temp = list(psutil.sensors_temperatures().values())[0][0].current

        try:
            battery_percentage = psutil.sensors_battery().percent
        except Exception:
            battery_percentage = None
        with contextlib.suppress(Exception):
            GPUs = GPUtil.getGPUs()
            for GPU in GPUs:
                gpu_usage = gpu_usage + float(GPU.load) * 100
                gpu_memory = gpu_memory + float(GPU.memoryUtil) * 100
            gpu_usage / len(GPUs)
            gpu_memory / len(GPUs)
            gpu_temp = list(psutil.sensors_temperatures().values())[1][0].current
        dict_data = net_io_usage()
        if dict_data is None:
            send_speed = 0
            recv_speed = 0

        else:
            send_speed = dict_data["send_speed"]
            recv_speed = dict_data["recv_speed"]
        dict_data = disk_io_usage()
        if dict_data is None:
            read_speed = 0
            write_speed = 0
        else:
            read_speed = dict_data["read_speed"]
            write_speed = dict_data["write_speed"]
        views.system_monitoring_save_to_db(
            cpu_usage,
            cpu_core_usage,
            cpu_temp,
            gpu_usage,
            gpu_memory,
            gpu_temp,
            ram_usage,
            swap_memory,
            temp,
            send_speed,
            recv_speed,
            send_speed + recv_speed,
            read_speed,
            write_speed,
            battery_percentage,
        )

        time.sleep(UPDATE_DELAY)

def updateIPForOfflineMaps():
    # Open the json file, read it and update the variable of the "tiles" array
    with open(Constants.OFFLINE_DEMO_MAP_TILEJSON_PATH, "r") as jsonFile:
        data = json.load(jsonFile)
        # Get the IP that is currently reported in tiles.json
        # For example, from the following url we will get the 127.0.0.1
        # "http://127.0.0.1:8000/static/offline-maps/demo-map/tiles/{z}/{x}/{y}.pbf"
        # and replace it with the current network IP

        start = "http://"
        end = ":" + os.environ.get("WEB_PORT")
        currentURL = data["tiles"][0]
        currentIP = currentURL[currentURL.find(start) + len(start) : currentURL.rfind(end)]
        newURL = currentURL.replace(currentIP, Constants.NET_IP)
        data["tiles"][0] = newURL

    # Write the file out again
    with open(Constants.OFFLINE_DEMO_MAP_TILEJSON_PATH, "w") as jsonFile:
        json.dump(data, jsonFile, indent=4)


def start():
    print("\n\n============LAUNCHING THE BACKEND OF THE PLATFORM FROM DOCKER===========\n\n")
    # exit(1)
    # terminateAllProcesses()  # Let's make sure all of the processes are killed before proceeding to re-create them
    # signal.signal(signal.SIGINT, signal_handler)
    create_dir_if_not_exists(Constants.MESH_RESULTS_FOLDER_DIR)

    serverMonitoringThread = threading.Thread(target=data_usage_handler, name="Server Monitoring script")
    serverMonitoringThread.start()
    print("[ \033[32m\u2714\033[0m ] Server Monitoring script.")
    time.sleep(1)

    sys.argv = Constants.START_MESH_MONITORING_SCRIPT_COMMAND[1:]
    meshOrthophotoThread = threading.Thread(target=mesh_monitoring.main, name="Mesh Orthophoto script")
    meshOrthophotoThread.start()
    print("[ \033[32m\u2714\033[0m ] Mesh Orthophoto script.")
    time.sleep(1)

    weatherStationPlatformThread = threading.Thread(target=weatherStationConnectAndStoredData.main, name="Weather Station on PC script")
    weatherStationPlatformThread.start()
    print("[ \033[32m\u2714\033[0m ] Weather Station on PC script.")
    time.sleep(1)

    mainMonitorHandler()

def main():
    start()
