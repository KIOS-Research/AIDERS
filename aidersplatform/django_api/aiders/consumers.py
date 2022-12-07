import json
import logging
import threading
import time

import pytz
from channels.generic.websocket import WebsocketConsumer
from django.conf import settings
from django.core.files.storage import default_storage
from django.forms import model_to_dict
from django.shortcuts import get_object_or_404
from logic.algorithms.lidar_point_cloud import lidar_points
from logic.algorithms.lora_station import lora_location_receiver
from logic.Constants import Constants
from requests import session
from torch import _sparse_sparse_matmul

from .models import *
from .views import MyThread

logger = logging.getLogger(__name__)


class Error(object):
    error_msg = {}

    def get_message(self, operation_name):
        if(operation_name in self.error_msg):
            if(self.error_msg[operation_name] == ''):
                return ''
            else:
                error_msg = self.error_msg[operation_name]
                self.set_message(operation_name, '')
                return error_msg
        else:
            return ''

    def set_message(self, operation_name, msg):
        self.error_msg[operation_name] = msg

    def set_message_and_error(self, logger, operation_name, msg):
        logger.error(msg)
        self.error_msg[operation_name] = msg


ErrorMsg = Error()


class ws_index(WebsocketConsumer):

    def get_all_drones_from_db(self, operation_name):
        drones = list(Drone.objects.filter(
            operation__operation_name=operation_name).values())
        for drone in drones:
            drone.pop('joined_at', None)
        return drones

    def get_drone_telemetry_from_db(self, drone_name):
        try:
            telemetry = Telemetry.objects.filter(
                drone__drone_name=drone_name).values().last()
            telemetry['received_at'] = str(telemetry['received_at'].astimezone(
                pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S'))
            return telemetry
        except Exception as e:
            return []

    def get_detection_frame_from_db(self, operation_name, drone_name):
        try:
            active_detection_session = DetectionSession.objects.get(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name)
            latest_frame = DetectionFrame.objects.filter(
                detection_session=active_detection_session).last()
            detected_objects = list(
                DetectedObject.objects.filter(frame=latest_frame).values())
            for objects in detected_objects:
                objects['detected_at'] = str(objects['detected_at'].astimezone(
                    pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S'))
            return detected_objects
        except Exception as e:
            return []

    def get_detection_frame_url_from_db(self, drone_name):
        try:
            active_detection_session = DetectionSession.objects.get(
                is_active=True, drone__drone_name=drone_name)
            return active_detection_session.latest_frame_url
        except Exception as e:
            return Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE

    def get_detection_from_db(self, drone_name):
        try:
            detection = Detection.objects.filter(
                drone__drone_name=drone_name).values().last()
            return detection
        except Exception as e:
            return []

    def update_detection_to_db(self, detection_object):
        try:
            from logic.algorithms.object_detection import aiders_tracker

            drone_name = detection_object['drone_name']
            detection_status = detection_object['detection_status']
            # import pdb; pdb.set_trace()
            qs = Detection.objects.filter(drone__drone_name=drone_name)
            detection = get_object_or_404(qs)
            drone = detection.drone
            detection.detection_status = detection_status
            if (detection_status == Detection.DetectionStatusChoices.DETECTION_WANT_TO_DISCONNECT):
                # import pdb;pdb.set_trace()
                tracker = aiders_tracker.Tracker.get(drone_name=drone_name)
                if tracker != None:
                    tracker.stop_tracker()
                    del tracker
            elif (detection_status == Detection.DetectionStatusChoices.DETECTION_WANT_TO_CONNECT):
                from logic.algorithms.object_detection.src.mot import \
                    DetectorType

                # import pdb; pdb.set_trace()
                net_ip = os.environ.get("NET_IP", "localhost")
                detection_type_str = detection_object['detection_type']
                rtmp_url = 'rtmp://{}:1935/live/{}'.format(net_ip, drone_name)
                user = User.objects.get(username=self.scope["user"])
                operation = Operation.objects.get(
                    operation_name=detection_object['operation_name'])
                tracker = aiders_tracker.Tracker(rtmp_url, display=False, drone=drone,
                                                 operation=operation, detection_type_str=detection_type_str,
                                                 user=user)
                tracker.start_tracker()
        except Exception as e:
            print(e)

    def get_weather_station_from_db(self, operation_name):
        try:
            weather_data = WeatherStation.objects.filter(
                operation__operation_name=operation_name).values().last()
            weather_data['current_time'] = str(weather_data['current_time'].astimezone(
                pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S '))
            return weather_data
        except:
            return ''

    def get_drone_weather_from_db(self, drone_name):
        try:
            weather_data = WeatherStation.objects.filter(
                drone__drone_name=drone_name).values().last()
            weather_data['current_time'] = str(weather_data['current_time'].astimezone(
                pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S '))
            return weather_data
        except:
            return ''

    def get_lora_meta(self):
        return {
            'isLoraReceiverAvailable': lora_location_receiver.getIfLoraReceiverAvailable(),
            'isLoraThreadActive': lora_location_receiver.getCurrentThreadActiveBoolean()
        }

    def get_lora_devices_from_db(self, operation_name):
        try:
            lora_data = {}
            list_of_lora = list(LoraTransmitter.objects.filter(
                operation=Operation.objects.get(operation_name=operation_name)))
            for lora in list_of_lora:
                lora_data[lora.tagName] = LoraTransmitterLocation.objects.filter(
                    loraTransmitter=lora).values().last()
                lora_data[lora.tagName]['received_at'] = str(lora_data[lora.tagName]['received_at'].astimezone(
                    pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S'))
            return lora_data
        except Exception as e:
            print(e)
            return ''

    def start_stop_lora_transmition(self, lora_info):
        op_name = lora_info['op_name']
        start = lora_info['start_lora_transmition']
        if (start):
            loraThread = MyThread(name='LoraThread{}'.format(op_name), target=lora_location_receiver.receiveLocations,
                                  args=(Constants.LORA_RECEIVER_DEVICE_SYMLINK_PATH, op_name))
            loraThread.start()
        else:
            for thread in threading.enumerate():
                if (thread.name == 'LoraThread{}'.format(op_name)):
                    thread.stop()

    def get_video_frame_url_from_db(self, drone_name):
        try:
            active_stream_session = LiveStreamSession.objects.get(
                is_active=True, drone__drone_name=drone_name)
            return active_stream_session.latest_frame_url
        except LiveStreamSession.DoesNotExist:
            return Constants.NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE

    def get_latest_image_from_build_map_db(self, drone_name, operation_name):
        if Drone.objects.get(drone_name=drone_name).build_map_activated == True:
            Session = BuildMapSession.objects.filter(operation=Operation.objects.get(
                operation_name=operation_name), drone=Drone.objects.get(drone_name=drone_name)).last()
            images = list(BuildMapImage.objects.filter(
                session=Session).values())
            for image in images:
                image['time'] = str(image['time'].astimezone(
                    pytz.timezone(settings.TIME_ZONE)).strftime('%H:%M:%S '))
                image['top_left'] = [float(image['top_left'].coords[0]), float(
                    image['top_left'].coords[1])]
                image['top_right'] = [float(image['top_right'].coords[0]), float(
                    image['top_right'].coords[1])]
                image['bottom_left'] = [float(image['bottom_left'].coords[0]), float(
                    image['bottom_left'].coords[1])]
                image['bottom_right'] = [float(image['bottom_right'].coords[0]), float(
                    image['bottom_right'].coords[1])]
                image['centre'] = [float(image['centre'].coords[0]), float(
                    image['centre'].coords[1])]
                image['altitude'] = float(image['altitude'])
                image['bearing'] = float(image['bearing'])
            return images
        else:
            return ""

    def update_lidar_to_db(self, lidar_object):
        operation_name = lidar_object['operation_name']
        drone_name = lidar_object['drone_name']
        if lidar_object['activate'] == True:
            lidar_session = LidarPointSession.objects.create(
                user=User.objects.get(username=self.scope["user"]),
                operation=Operation.objects.get(operation_name=operation_name),
                drone=Drone.objects.get(drone_name=drone_name),
                is_active=True,
                is_process=False,
            )
            publisherThread = MyThread(
                name='Lidar'+drone_name, target=lidar_points.listener, args=(drone_name, lidar_session))
            publisherThread.start()
            time.sleep(1)
            lidar_points.PublishMessage(drone_name, 'START')
        elif lidar_object['activate'] == False:
            try:
                lidar_object = LidarPointSession.objects.filter(drone=Drone.objects.get(
                    drone_name=drone_name), operation=Operation.objects.get(operation_name=operation_name))
                lidar_object.update(is_active=False, end_time=datetime.datetime.now(
                    tz=Constants.CYPRUS_TIMEZONE_OBJ))
                lidar_points.PublishMessage(drone_name, 'STOP')
            except Exception as e:
                print(e)

    def connect(self):
        logger.info('User {} connected to websocket index.'.format(
            self.scope["user"]))
        self.accept()

    def receive(self, text_data=None, bytes_data=None):
        operation_name = None
        try:
            operation_name = Operation.objects.get(operation_name=text_data)
        except Exception as e:
            operation_name = None
        if operation_name != None:
            data = {}
            operation_name = text_data
            data['drones'] = self.get_all_drones_from_db(operation_name)
            for drone in data['drones']:
                drone.update(
                    {'telemetry': self.get_drone_telemetry_from_db(drone['drone_name'])})
                drone.update({'detected_frame': self.get_detection_frame_from_db(
                    operation_name, drone['drone_name'])})
                drone.update(
                    {'detected_frame_url': self.get_detection_frame_url_from_db(drone['drone_name'])})
                drone.update(
                    {'detection': self.get_detection_from_db(drone['drone_name'])})
                drone.update(
                    {'weather': self.get_drone_weather_from_db(drone['drone_name'])})
                drone.update(
                    {'video_frame_url': self.get_video_frame_url_from_db(drone['drone_name'])})
                drone.update({'build_map_last_image': self.get_latest_image_from_build_map_db(
                    drone['drone_name'], operation_name)})
            data.update(
                {'weather_station': self.get_weather_station_from_db(operation_name)})
            data.update(
                {'lora_devices': self.get_lora_devices_from_db(operation_name)})
            data.update({'error_msg': ErrorMsg.get_message(operation_name)})
            self.send(json.dumps(data))

        else:
            request = json.loads(text_data)
            print(request)
            if 'detection' in request:
                self.update_detection_to_db(request['detection'])
            elif 'lidar' in request:
                print(request['lidar'])
                self.update_lidar_to_db(request['lidar'])

    def disconnect(self, close_code):
        logger.info('User {} disconnected to websocket index.'.format(
            self.scope["user"]))
        pass


class ws_monitoring(WebsocketConsumer):
    def getPlatformFromDB(self):
        data = SystemMonitoring.objects.filter().values().last()
        data.pop("time")
        return data

    def getDroneFromDB(self, drone):
        data = ControlDevice.objects.filter(
            drone=Drone.objects.get(drone_name=drone)).values().last()
        data.pop("received_at")
        return data

    def connect(self):
        self.accept()

    def receive(self, text_data=None, bytes_data=None):
        database = text_data
        try:
            drone_available = Drone.objects.get(drone_name=text_data)
        except:
            drone_available = None
        if database == 'platform':
            time_data = self.getPlatformFromDB()
            self.send(json.dumps(time_data))
        elif drone_available != None:
            time_data = self.getDroneFromDB(database)
            self.send(json.dumps(time_data))
        else:
            self.send('')

    def disconnect(self, close_code):
        pass
