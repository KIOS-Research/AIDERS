import logging
import rospy
from aiders import models, views
from kios.msg import TerminalHardware
logger = logging.getLogger(__name__)


def jetson_data_callback(data, args):
    cpu_usage = 0
    cpu_core_usage = ''
    for core_usage in list(data.cpu_core_usage):
        cpu_usage = cpu_usage+float(core_usage)
        if cpu_core_usage != '':
            cpu_core_usage = cpu_core_usage+', '+str(core_usage)
        else:
            cpu_core_usage = cpu_core_usage+str(core_usage)
    cpu_usage = cpu_usage/len(data.cpu_core_usage)
    ram_usage = float(data.ram_use)*100/float(data.ram_max)
    swap_usage = float(data.swap_use)*100/float(data.swap_max)
    jetsonObj = {
        "drone": models.Drone.objects.get(drone_name=args[0]),
        "cpu_usage": round(cpu_usage, 1),
        "cpu_core_usage": str(cpu_core_usage),
        "cpu_core_frequency": str(data.cpu_core_freq),
        "cpu_temp": round(float(data.cpuTemp)/1000, 1),
        "cpu_fan_RPM": round(float(data.cpuFanRPM), 1),
        "gpu_usage": round(float(data.gr3d_usage), 1),
        "gpu_frequency": round(float(data.gr3d_freq), 1),
        'gpu_temp': round(float(data.gpuTemp)/1000, 1),
        "ram_usage": round(ram_usage, 1),
        "swap_usage": round(swap_usage, 1),
        "swap_cache": round(float(data.swap_cache), 1),
        "emc_usage": round(float(data.emc_usage), 1),
    }
    views.ControlDeviceDataAPIView.control_device_save_data_to_db(jetsonObj,)


def start_listening(dji_name="kios_mavic1g", canGetMission=False):
    subscriber = rospy.Subscriber(
        "/"+dji_name+"/TerminalHardware", TerminalHardware, jetson_data_callback, (dji_name,))
    logger.info('Control Device {} is ready for monitoring.'.format(dji_name))
    rospy.spin()
