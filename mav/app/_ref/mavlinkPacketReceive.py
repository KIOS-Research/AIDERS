"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""


# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
# master = mavutil.mavlink_connection('tcpin:169.254.1.20:8000')
master = mavutil.mavlink_connection('udpin:localhost:14445')#14550

#connect to groundControlStation
#gcs = mavutil.mavlink_connection('udpout:0.0.0.0:14550', source_system=1)


# Make sure the connection is valid
master.wait_heartbeat()

#send message to groundControlStation
#gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
#                           "Python is now receiving Telemetry".encode())


# Get some information !
while True:
    try:
        out = master.recv_match().to_dict()
        '''
        if out.get('mavpackettype','')=='GPS_RAW_INT':
            print('RAW GPS: LAT:', out.get('lat'), ' LON: ', out.get('lon'), ' VEL: ', out.get('vel'))
        elif out.get('mavpackettype','')=='ALTITUDE':
            print('AMSL ALT: ', out.get('altitude_amsl'), ' RELATIVE ALT: ', out.get('altitude_relative'))
        elif out.get('mavpackettype','')=='BATTERY1':
            print('BATT: ', out.get('battery_remaining'), '%\n')
        elif out.get('mavpackettype','')=='GLOBAL_POSITION_INT':
            print('RAW GPS: LAT:', out.get('lat'), ' LON: ', out.get('lon'), ' VEL: ', out.get('vel'))
        '''
        if out.get('mavpackettype','')=='GPS_RAW_INT':
            print(out)
        elif out.get('mavpackettype')=='GPS2_RAW':
            print(out)
        elif out.get('mavpackettype')=='GLOBAL_POSITION_INT': 
            print(out)
        elif out.get('mavpackettype')=='BATTERY_STATUS':
            print(out)
        elif out.get('mavpackettype')=='GPS_RAW_INT':
            print(out)
        #else:
        #    print('\n', out.get('mavpackettype'), '\n')
    except:
        pass
    time.sleep(0.1)

# Output:
# {'mavpackettype': 'AHRS2', 'roll': -0.11364290863275528, 'pitch': -0.02841472253203392, 'yaw': 2.0993032455444336, 'altitude': 0.0, 'lat': 0, 'lng': 0}
# {'mavpackettype': 'AHRS3', 'roll': 0.025831475853919983, 'pitch': 0.006112074479460716, 'yaw': 2.1514968872070312, 'altitude': 0.0, 'lat': 0, 'lng': 0, 'v1': 0.0, 'v2': 0.0, 'v3': 0.0, 'v4': 0.0}
# {'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 123, 'throttle': 0, 'alt': 3.129999876022339, 'climb': 3.2699999809265137}
# {'mavpackettype': 'AHRS', 'omegaIx': 0.0014122836291790009, 'omegaIy': -0.022567369043827057, 'omegaIz': 0.02394154854118824, 'accel_weight': 0.0, 'renorm_val': 0.0, 'error_rp': 0.08894175291061401, 'error_yaw': 0.0990816056728363}
