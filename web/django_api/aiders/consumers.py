import json
import logging
import threading
import time

import pytz
from channels.generic.websocket import AsyncWebsocketConsumer
from channels.db import database_sync_to_async
from django.conf import settings
from django.shortcuts import get_object_or_404
# from logic.algorithms.lidar_point_cloud import lidar_points
from logic.Constants import Constants

from .models import *
from .views import MyThread

logger = logging.getLogger(__name__)


class Error(object):
    error_msg = {}

    def get_message(self, operation_name):
        if operation_name in self.error_msg and self.error_msg[operation_name] == "" or operation_name not in self.error_msg:
            return ""
        error_msg = self.error_msg[operation_name]
        self.set_message(operation_name, "")
        return error_msg

    def set_message(self, operation_name, msg):
        self.error_msg[operation_name] = msg

    def set_message_and_error(self, logger, operation_name, msg):
        logger.error(msg)
        self.error_msg[operation_name] = msg


ErrorMsg = Error()


class ws_platform(AsyncWebsocketConsumer):
    @database_sync_to_async
    def get_all_drones_from_db(self, operation_name):
        return [
            {key: value for key, value in drone.items() if key != "time"}
            for drone in list(Drone.objects.filter(operation__operation_name=operation_name).values())
            if drone["is_connected_with_platform"]
        ]

    @database_sync_to_async
    def get_drone_telemetry_from_db(self, drone_name):
        try:
            telemetry = Telemetry.objects.filter(drone__drone_name=drone_name).values().last()
            telemetry["time"] = str(telemetry["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
            return telemetry
        except Exception as e:
            return []

    @database_sync_to_async
    def get_detection_frame_from_db(self, operation_name, drone_name):
        try:
            active_detection_session = DetectionSession.objects.get(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name
            )
            latest_frame = DetectionFrame.objects.filter(detection_session=active_detection_session).last()
            detected_objects = list(DetectedObject.objects.filter(frame=latest_frame).values())
            #logger.info(detected_objects)
            for objects in detected_objects:
                objects["detected_at"] = str(objects["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
                objects["time"] = str(objects["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
                track_id = objects["track_id"]
                description_object = DetectedObjectDescription.objects.filter(track_id=track_id).last()
                # logger.info(description_object)
                if description_object:
                    objects["description"] = description_object.description
                    objects["updated_by_username"] = description_object.updated_by.username
                else:
                    objects["description"] = ""
                    objects["updated_by_username"] = ""
            #logger.info(detected_objects)
            return detected_objects
        except Exception as e:
            return []

    # SafeML
    @database_sync_to_async
    def get_detection_frame_safeml_from_db(self, _operationName, _droneName):
        try:
            active_detection_session = SafemlDetectionSession.objects.get(
                is_active=True, operation__operation_name=_operationName, drone__drone_name=_droneName
            )
            return active_detection_session.latest_frame_url
        except Exception as e:
            return Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE
    @database_sync_to_async
    def get_detection_frame_deepknowledge_from_db(self, _operationName, _droneName):
        try:
            active_detection_session = DeepKnowledgeDetectionSession.objects.get(
                is_active=True, operation__operation_name=_operationName, drone__drone_name=_droneName
            )
            return active_detection_session.latest_frame_url
        except Exception as e:
            return Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE

    @database_sync_to_async
    def get_detection_frame_url_from_db(self, drone_name):
        try:
            active_detection_session = DetectionSession.objects.get(is_active=True, drone__drone_name=drone_name)
            return active_detection_session.latest_frame_url
        except Exception as e:
            return Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE

    @database_sync_to_async
    def get_detection_from_db(self, drone_name):
        try:
            return Detection.objects.filter(drone__drone_name=drone_name).values().last()
        except Exception as e:
            return []

    @database_sync_to_async
    def get_weather_station_from_db(self, operation_name):
        try:
            weather_data = WeatherStation.objects.filter().values().last()
            weather_data["time"] = str(weather_data["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            return weather_data
        except Exception:
            return ""

    @database_sync_to_async
    def get_drone_weather_from_db(self, drone_name):
        try:
            telemetry = WeatherStation.objects.filter(drone__drone_name=drone_name).values().last()
            telemetry["time"] = str(telemetry["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            return telemetry
        except Exception:
            return ""

    @database_sync_to_async
    def get_all_devices_from_db(self, operation_name):
        return [
            {key: value for key, value in device.items() if key != "time"}
            for device in list(Device.objects.filter(operation__operation_name=operation_name).values())
            if device["is_connected_with_platform"]
        ]

    @database_sync_to_async
    def get_device_telemetry_from_db(self, device_name):
        try:
            telemetry = DeviceTelemetry.objects.filter(device__name=device_name).values().last()
            telemetry["time"] = str(telemetry["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
            return telemetry
        except Exception as e:
            return []

    @database_sync_to_async
    def get_latest_image_from_device_db(self, device_name, operation_name):
        try:
            session = DeviceSession.objects.filter(
                is_active=True,
                device=Device.objects.get(name=device_name),
            ).last()
            # images = list(DeviceImage.objects.filter(session=session).values())
            images = list(DeviceImage.objects.filter(session=session).order_by('-time').values()[:50])
            for image in images:
                image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            return images
        except Exception as e:
            print(e)

    @database_sync_to_async
    def get_all_baloras_from_db(self, operation_name):
        return [
            {
                key: BaloraMaster.objects.get(id=balora["baloraMaster_id"]).name if key == "baloraMaster_id" else value
                for key, value in balora.items()
                if key != "time"
            }
            for balora in list(Balora.objects.filter(baloraMaster__operation__operation_name=operation_name).values())
            if BaloraMaster.objects.get(id=balora["baloraMaster_id"]).is_connected_with_platform
        ]

    @database_sync_to_async
    def get_balora_telemetry_from_db(self, balora_name):
        try:
            telemetry = BaloraTelemetry.objects.filter(balora__name=balora_name).values().last()
            telemetry["time"] = str(telemetry["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
            return telemetry
        except Exception as e:
            return []

    @database_sync_to_async
    def get_manually_set_objects_from_db(self, operation_name):
        try:
            mso_data = {}

            list_of_mso = list(ManuallySetObject.objects.filter(operation=Operation.objects.get(operation_name=operation_name)))

            # logger.info(list_of_mso)

            for mso in list_of_mso:

                mso_data[mso.id] = {}
                # mso_data[mso.id]["description"] = mso.description
                mso_data[mso.id]["created_by_username"] = mso.created_by.username

                mso_data[mso.id]["created_at"] = str(mso.created_at.astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))

                mso_location = ManuallySetObjectLocation.objects.filter(manually_set_object=mso).last()

                if mso_location != None:

                    mso_data[mso.id]["lon"] = mso_location.lon
                    mso_data[mso.id]["lat"] = mso_location.lat
                    mso_data[mso.id]["coords_set_at"] = str(
                        mso_location.received_at.astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S")
                    )
                    mso_data[mso.id]["coords_set_by_username"] = mso_location.received_by.username

                mso_description = ManuallySetObjectDescription.objects.filter(manually_set_object=mso).last()

                if mso_description != None:

                    mso_data[mso.id]["description"] = mso_description.description
                    mso_data[mso.id]["updated_at"] = str(
                        mso_description.updated_at.astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S")
                    )
                    mso_data[mso.id]["updated_by_username"] = mso_description.updated_by.username

            # logger.info( mso_data )
            return mso_data
        except Exception as e:
            # logger.info( str( e ) )
            return "get_mso_error"

    @database_sync_to_async
    def get_video_frame_url_from_db(self, drone_name):
        try:
            active_stream_session = LiveStreamSession.objects.get(is_active=True, drone__drone_name=drone_name)
            return active_stream_session.latest_frame_url
        except LiveStreamSession.DoesNotExist:
            return Constants.NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE

    @database_sync_to_async
    def get_latest_image_from_build_map_db(self, drone_name, operation_name):
        if Drone.objects.get(drone_name=drone_name).build_map_activated != True:
            return ""
        Session = BuildMapSession.objects.filter(
            operation=Operation.objects.get(operation_name=operation_name),
            drone=Drone.objects.get(drone_name=drone_name),
        ).last()
        images = list(BuildMapImage.objects.filter(session=Session).values())
        for image in images:
            image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
            image["top_left"] = [float(image["top_left"].coords[0]), float(image["top_left"].coords[1])]
            image["top_right"] = [float(image["top_right"].coords[0]), float(image["top_right"].coords[1])]
            image["bottom_left"] = [float(image["bottom_left"].coords[0]), float(image["bottom_left"].coords[1])]
            image["bottom_right"] = [float(image["bottom_right"].coords[0]), float(image["bottom_right"].coords[1])]
            image["centre"] = [float(image["centre"].coords[0]), float(image["centre"].coords[1])]
            image["altitude"] = float(image["altitude"])
            image["bearing"] = float(image["bearing"])
        return images

    @database_sync_to_async
    def update_lidar_to_db(self, lidar_object):
        operation_name = lidar_object["operation_name"]
        drone_name = lidar_object["drone_name"]
        if lidar_object["activate"] == True:
            if len(LidarPointSession.objects.filter(is_active=True)) == 0:
                lidar_session = LidarPointSession.objects.create(
                    user=User.objects.get(username=self.scope["user"]),
                    operation=Operation.objects.get(operation_name=operation_name),
                    drone=Drone.objects.get(drone_name=drone_name),
                    is_active=True,
                    is_process=False,
                )
                publisherThread = MyThread(
                    name=f"Lidar{drone_name}",
                    target=lidar_points.listener,
                    args=(drone_name, lidar_session),
                )
                publisherThread.start()
                time.sleep(1)
            lidar_points.PublishMessage(drone_name, "START")
        elif lidar_object["activate"] == False:
            try:
                lidar_object = LidarPointSession.objects.filter(
                    drone=Drone.objects.get(drone_name=drone_name),
                    operation=Operation.objects.get(operation_name=operation_name),
                )
                lidar_object.update(is_active=False, end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ))
                lidar_points.PublishMessage(drone_name, "STOP")
            except Exception as e:
                print(e)

    async def connect(self):
        logger.info(f'User {self.scope["user"]} connected to websocket index.')
        await self.accept()


    async def receive(self, text_data=None, bytes_data=None):
        try:
            json_data = json.loads(text_data)
            if(json_data["get_all_data"] == 1):
                await self.get_all_data_from_db(json_data["operation_name"])
            else:
                await self.get_video_frames_from_db(json_data["operation_name"])
        except Exception as e:
            print(e)


    async def get_all_data_from_db(self, operation_name):
        # print("get_data_from_db")
        data = {"drones": await self.get_all_drones_from_db(operation_name)}
        for drone in data["drones"]:
            drone.update({"telemetry": await self.get_drone_telemetry_from_db(drone["drone_name"])})
            drone["detected_objects"] =  await self.get_detection_frame_from_db(operation_name, drone["drone_name"])
            #drone.update({"detected_objects": await self.get_detection_frame_from_db(operation_name, drone["drone_name"])})
            drone.update({"detected_frame_url": await self.get_detection_frame_url_from_db(drone["drone_name"])})
            drone.update({"detection": await self.get_detection_from_db(drone["drone_name"])})
            drone.update({"weather": await self.get_drone_weather_from_db(drone["drone_name"])})
            drone.update({"video_frame_url": await self.get_video_frame_url_from_db(drone["drone_name"])})
            drone.update({"build_map_last_image": await self.get_latest_image_from_build_map_db(drone["drone_name"], operation_name)})
        data["devices"] = await self.get_all_devices_from_db(operation_name)
        for device in data["devices"]:
            device.update({"telemetry": await self.get_device_telemetry_from_db(device["name"])})
            device.update({"images": await self.get_latest_image_from_device_db(device["name"], operation_name)})
        data["baloras"] = await self.get_all_baloras_from_db(operation_name)
        for balora in data["baloras"]:
            balora.update({"telemetry": await self.get_balora_telemetry_from_db(balora["name"])})
        data["weather_station"] = await self.get_weather_station_from_db(operation_name)
        data["manually_set_objects"] = await self.get_manually_set_objects_from_db(operation_name)
        data["error_msg"] = ErrorMsg.get_message(operation_name)
        await self.send(json.dumps(data))


    async def get_video_frames_from_db(self, operation_name):
        data = {"drones": await self.get_all_drones_from_db(operation_name)}
        for drone in data["drones"]:
            drone.update({"video_frame_url": await self.get_video_frame_url_from_db(drone["drone_name"])})
            drone.update({"detected_frame_url": await self.get_detection_frame_url_from_db(drone["drone_name"])})
            drone["detected_objects"] =  await self.get_detection_frame_from_db(operation_name, drone["drone_name"])
            # SafeML
            drone["detected_frame_safeml_url"] = await self.get_detection_frame_safeml_from_db(operation_name, drone["drone_name"])
            drone["detected_frame_deepknowledge_url"] = await self.get_detection_frame_deepknowledge_from_db(operation_name, drone["drone_name"])
            drone.update({"detection": await self.get_detection_from_db(drone["drone_name"])})
        await self.send(json.dumps(data))


    async def disconnect(self, close_code):
        logger.info(f'User {self.scope["user"]} disconnected to websocket index.')


class ws_monitoring(AsyncWebsocketConsumer):
    @database_sync_to_async
    def getPlatformFromDB(self):
        return {k: v for k, v in SystemMonitoring.objects.filter().values().last().items() if k != "time"}

    @database_sync_to_async
    def getDroneFromDB(self, drone):
        return {k: v for k, v in ControlDevice.objects.filter(drone=Drone.objects.get(drone_name=drone)).values().last().items() if k != "time"}

    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data):
        try:
            try:
                drone_available = Drone.objects.get(drone_name=text_data)
            except Exception:
                drone_available = None
            if text_data == "platform":
                await self.send(json.dumps(await self.getPlatformFromDB()))
            elif drone_available != None:
                await self.send(json.dumps(await self.getDroneFromDB(text_data)))
            else:
                await self.send("")
        except Exception as e:
            print(e)
