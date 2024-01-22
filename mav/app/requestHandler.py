import os
import sys
import time
import threading
import asyncio
from datetime import datetime

# custom libs
from database.connection import MySQLConnector
import database.queries
import utils
from uav import Uav

uavs = {}


# instantiate a new Uav object connect to the drone
# async def connectToUav(_name, _ip, _port, _type, _model, _operationId):
async def connectToUav(_name, _ip, _port, _model, _operationId):
    if _name not in uavs:
        print(f"'{_name}' CONNECTING...", flush=True)
        uav = Uav(_name, _ip, _port, _model, _operationId)
        uavs[_name] = uav
    else:
        print(f"'{_name}' RECONNECTING...", flush=True)
        uav = uavs[_name]
        uav.ip = _ip
        uav.port = _port
        uav.model = _model
        uav.operationId = _operationId

    await uav.connect()


# disconnect Uav
async def disconnectFromUav(_name):
    uav = uavs[_name]
    await uav.disconnect()


async def publishTakeoffCommand(_name, _alt):
    uav = uavs[_name]
    await uav.arm()
    await uav.takeoff(_alt)


async def publishLandCommand(_name):
    uav = uavs[_name]
    await uav.land()


async def publishMissionCommand(_name, _missionPoints, _speed):
    uav = uavs[_name]
    await uav.sendMission(_missionPoints, _speed)


async def publishPauseMissionCommand(_name):
    uav = uavs[_name]
    await uav.pauseMission()


async def publishResumeMissionCommand(_name):
    uav = uavs[_name]
    await uav.resumeMission()


async def publishCancelMissionCommand(_name):
    uav = uavs[_name]
    await uav.cancelMission()


async def publishSetSpeedCommand(_name, _speed):
    uav = uavs[_name]
    await uav.setSpeed(_speed)


async def publishTransitionCommand(_name, _mode):
    uav = uavs[_name]
    if _mode == "FW":
        await uav.transitionToFw()
    elif _mode == "MC":
        await uav.transitionToMc()
    else:
        print("Invalid transition mode received.", flush=True)


async def publishReturnCommand(_name):
    uav = uavs[_name]
    await uav.returnHome()


async def publishArmCommand(_name):
    uav = uavs[_name]
    await uav.arm()


async def publishDisarmCommand(_name):
    uav = uavs[_name]
    await uav.disarm()


async def publishKillCommand(_name):
    uav = uavs[_name]
    await uav.kill()


async def publishRebootCommand(_name):
    uav = uavs[_name]
    await uav.reboot()


async def publishShutdownCommand(_name):
    uav = uavs[_name]
    await uav.shutdown()


# async def publishGoToCommand(_name, _lat, _lon, _alt):
#     uav = uavs[_name]
#     await uav.goTo(_lat, _lon, _alt)


# # start reading telemetry asynchronously
# async def startTelemetry(_name):
#     uav = uavs[_name]
#     loop = asyncio.new_event_loop()
#     asyncio.set_event_loop(loop)
#     try:
#         await uav.receiveTelemetry()
#     finally:
#         loop.close()



