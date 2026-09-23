import csv
import json
import logging
import os
import shutil
import threading
import zipfile
from datetime import datetime, timedelta
from decimal import Decimal
import pytz
import requests

from django import forms
from django.conf import settings
from django.contrib import messages
from django.contrib.auth import authenticate, get_user_model, login, logout
from django.contrib.auth.decorators import login_required
from django.contrib.auth.forms import AuthenticationForm
from django.contrib.auth.mixins import LoginRequiredMixin
from django.contrib.gis.geos import point
from django.core import serializers as core_serializers
from django.core.exceptions import PermissionDenied
from django.core.files.storage import default_storage
from django.forms.models import model_to_dict

from django.http import (
    FileResponse,
    Http404,
    HttpResponse,
    HttpResponseForbidden,
    HttpResponseNotFound,
    HttpResponseRedirect,
    JsonResponse,
)
from django.shortcuts import get_list_or_404, get_object_or_404, redirect, render
from django.urls import resolve, reverse, reverse_lazy
from django.utils import timezone
from django.utils.decorators import method_decorator
from django.views import View, generic
from django.views.decorators.csrf import csrf_exempt, csrf_protect
from guardian.shortcuts import assign_perm
from json2html import *
from logic import utils
from logic.algorithms.build_map import build_map_request_handler, img_georeference
from logic.algorithms.external_request import patho_request
from logic.algorithms.flying_report import flying_report
from logic.algorithms.mission import mission_request_handler
from logic.algorithms.operation_report.operation_report_generator import OperationReportGenerator
from aiders.utils.timeout_decorators import with_timeout
from logic.algorithms.safe_drones import calculations_safe_drones
from logic.Constants import Constants
from PIL import Image
from rest_framework import generics, permissions, status
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework.views import APIView

# from .factories import *
from django.contrib.gis.geos import Point
from .forms import *
from .httpRequests import (
    postDetectionStartToCv,
    postDetectionStopToCv,
    postRequestForLidarStartOrStop,
    postRequestForOpenWaterSamplingValve,
    startDroneLiveStreamCapture,
    postRequestForPilotNotification,
    postRequestForDeviceNotification,
)
from .models import DetectionSession, GroundVehicle, LiveStreamSession, ManuallySetObject, ManuallySetObjectLocation, Operation, StaticCamera
from .permissions import IsOwnerOrReadOnly
from .serializers import *

logger = logging.getLogger(__name__)

SENSOR_DATA_DURATION_HOURS = 0.17 #To initialize the duration to display previous data of SENSIRION

# Function for creating Thread instances with stop function and timer function


@login_required
def serve_media(request, path):
    response = HttpResponse()
    response['X-Accel-Redirect'] = '/protected-media/' + path
    del response['Content-Type']  # let nginx set it
    return response

class MyThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, *args, **kwargs):
        super(MyThread, self).__init__(*args, **kwargs)
        self._stop = threading.Event()
        self._time = 0

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def time(self, seconds):
        self._time = seconds

    def get_time(self):
        return self._time


# class DatabaseFiller(APIView):
#     """
#     A class that populates the database with dummy data.
#     It utilizes the Factory notion, using the Factory Boy library
#     Reference: https://factoryboy.readthedocs.io/en/stable/orms.html
#     """

#     def get(self, request):
#         UserFactory.create_batch(20)
#         OperationFactory.create_batch(20)
#         mission_points = MissionPointFactory.create_batch(10)
#         MissionFactory.create_batch(20, mission_points=tuple(mission_points))
#         mission = Mission.objects.all().first()
#         drones = DroneFactory.create_batch(20)
#         WeatherStationFactory.create_batch(50)
#         TelemetryFactory.create_batch(50)
#         LiveStreamSessionFactory.create_batch(20)
#         RawFrameFactory.create_batch(20)
#         DetectionFactory.create_batch(20)
#         DetectionSessionFactory.create_batch(50)
#         DetectionFrameFactory.create_batch(20)
#         DetectedObjectFactory.create_batch(20)
#         AlgorithmFactory.create_batch(20)
#         WaterSamplerFactory.create_batch(20)
#         ErrorMessageFactory.create_batch(20)
#         FrontEndUserInputFactory.create_batch(20)
#         LidarPointSessionFactory.create_batch(20)
#         LidarPointFactory.create_batch(20)
#         BuildMapImageFactory.create_batch(50)
#         BuildMapSessionFactory.create_batch(20)
#         ControlDeviceFactory.create_batch(20)
#         MissionLogFactory.create_batch(20)
#         return redirect("login")


# class OperationListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
#     """
#     List all operations or create new one. The get and create methods are inherited,
#     using the generics.ListCreateAPIView.
#     Tutorial Reference: https://www.django-rest-framework.org/tutorial/3-class-based-views/
#     """

#     queryset = Operation.objects.all()
#     serializer_class = OperationSerializer

#     """
#      Ensure that authenticated requests get read-write access, and unauthenticated requests get read-only access
#     """
#     permission_classes = [permissions.IsAuthenticatedOrReadOnly, IsOwnerOrReadOnly]

#     def perform_create(self, serializer):
#         """
#         Allows us to modify how the instance save is managed,
#         and handle any information that is implicit in the incoming request or requested URL.
#         """
#         serializer.save(operator=self.request.user)  # Operations are associated with the user that created them


class DroneListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    serializer_class = DroneSerializer

    def get_queryset(self):
        operation_name = self.kwargs.get("operation_name")
        return Drone.objects.filter(operation__operation_name=operation_name)

    def post(self, request, *args, **kwargs):
        return self.create(request, *args, **kwargs)


class DroneRetrieveAPIView(LoginRequiredMixin, generics.RetrieveUpdateDestroyAPIView):
    """
    Retrieve, update (patch) or delete a drone instance
    """

    queryset = Drone.objects.all()
    serializer_class = DroneSerializer
    lookup_field = "drone_name"

    def get_object(self):
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        obj = Drone.objects.get(drone_name=drone_name)
        if obj is None:
            raise Http404
        return obj

    def patch(self, request, *args, **kwargs):
        """
        Partially update the attributes of a drone.
        This is useful for example in case the drone is connected/disconnected from the platform, we update (patch)
        the "is_drone_active" field to true/false. OR we can update its DroneDetection field
        """
        operation_name = self.kwargs.get("operation_name")
        operation_obj = Operation.objects.filter(operator=request.user, active=True)
        drone_name = self.kwargs.get("drone_name")
        qs = Drone.objects.filter(name=drone_name, operation__operation_name=operation_name)
        obj = get_object_or_404(qs)
        serializer = DroneSerializer(obj, data=json.loads(request.body), partial=True)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data)


class DetectionRetrieveAPIView(LoginRequiredMixin, generics.RetrieveUpdateDestroyAPIView):
    """
    Retrieve, update (patch) or delete a detection drone instance
    """

    queryset = Drone.objects.all()
    serializer_class = DetectionDroneSerializer
    lookup_field = "drone_name"

    def patch(self, request, *args, **kwargs):
        """
        Partially update the attributes of a detection drone.
        This is useful when we just want to change the detection status of the drone
        """
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")
        qs = Detection.objects.filter(name=drone_name, operation__operation_name=operation_name)
        obj = get_object_or_404(qs)
        serializer = DetectionSerializer(obj, data=json.loads(request.body), partial=True)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data)

class DetectionStartOrStopAPIView(LoginRequiredMixin,generics.ListAPIView):
    def post(self, request, *args, **kwargs):
        if not User.checkIfUserAllowToExecudeCommands(request.user):
            return HttpResponseForbidden("You do not have permission to execute commands.")
        data = request.data
        detectionStatus = data.get("detectionStatus")
        userId = request.user.pk
        droneName = data.get("droneName")
        droneId = Drone.getDroneIdByName(droneName)
        operationName = data.get("operationName")
        operationId = Operation.getOperationIdByName(operationName)
        detectionType = data.get("detectionType")
        if detectionStatus == Detection.DetectionStatusChoices.DETECTION_WANT_TO_CONNECT:
            apiResponse = postDetectionStartToCv(userId, operationId, droneId, droneName, detectionType)
        elif detectionStatus == Detection.DetectionStatusChoices.DETECTION_WANT_TO_DISCONNECT:
            apiResponse = postDetectionStopToCv(operationId, droneId, droneName)
        else:
            # Handle other cases or provide an error response
            return HttpResponse("Invalid detectionStatus", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse(apiResponse, status=status.HTTP_200_OK)


class LidarStartOrStopAPIView(LoginRequiredMixin, generics.ListAPIView):
    def post(self, request, *args, **kwargs):
        data = request.data
        lidarCommand = data.get("command")
        droneName = data.get("droneName")
        droneId = Drone.getDroneIdByName(droneName)
        userId = request.user.pk
        operationName = data.get("operationName")
        operationId = Operation.getOperationIdByName(operationName)
        latestSession = LidarPointSession.getLatestActiveSessionByDroneId(droneId)
        droneType = Drone.objects.get(drone_name=droneName).type

        if lidarCommand == "START":
            if latestSession is not None:
                return JsonResponse(
                    {"message": "Lidar Session is already running.", "lidar_session_id": latestSession.id}, status=200
                )
            latestSession = LidarPointSession.objects.create(
                user_id=userId, operation_id=operationId, drone_id=droneId, is_active=True
            )
            postRequestForLidarStartOrStop(droneId, droneName, latestSession.id, droneType, lidarCommand)
            return JsonResponse(
                {"message": "Lidar Session is started.", "lidar_session_id": latestSession.id}, status=200
            )
        elif lidarCommand == "STOP":
            # print(latestSession is not None, flush=True)
            if latestSession is not None:
                deactivateSession = LidarPointSession.deactivateSession(latestSession)
                if deactivateSession:
                    postRequestForLidarStartOrStop(droneId, droneName, latestSession.id, droneType, lidarCommand)
                    return JsonResponse(
                        {"message": "Lidar Session is deactivated.", "lidar_session_id": latestSession.id}, status=200
                    )
            return JsonResponse(
                {"message": "There is no lidar session active."}, status=200
            )
        return JsonResponse({"message": "Lidar command Not Valid."}, status=400)

class MissionListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = Mission.objects.all()
    serializer_class = MissionSerializer

    def mission_save_to_db(self, dronePK, userPK, operationPK):
        serializer = MissionSerializer(data=self)
        if serializer.is_valid():
            createdMission = serializer.save()
            Drone.objects.filter(pk=dronePK).update(mission=createdMission.pk)
            logger.info(f"Mission with id {createdMission.pk} is created successfully.")
            MissionLoggerListCreateAPIView.mission_logger_save_to_db("START_MISSION", createdMission, userPK, operationPK, dronePK)
            return True
        else:
            msg = f"Mission is not valid and is not created. Error: {serializer.errors}."
            from .consumers import ErrorMsg

            ErrorMsg.set_message_and_error(logger, Drone.objects.get(pk=dronePK).operation.operation_name, msg)
            return False


class MissionLoggerListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = MissionLog.objects.all()
    serializer_class = MissionLoggerSerializer

    def mission_logger_save_to_db(self, mission, userPK, operationPK, dronePK):
        if Mission.objects.get(pk=mission.pk).mission_type == "SEARCH_AND_RESCUE_MISSION":
            algorithm = Algorithm.objects.filter(
                algorithm_name="CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM", user=userPK, operation=operationPK
            ).last()
            algorithmPK = algorithm.pk
        else:
            algorithmPK = None
        missionLoggerData = {
            "action": self,
            "mission": Mission.objects.get(pk=mission.pk).pk,
            "user": userPK,
            "operation": operationPK,
            "drone": dronePK,
            "algorithm": algorithmPK,
        }
        serializerMissionLogger = MissionLoggerSerializer(data=missionLoggerData)
        if serializerMissionLogger.is_valid():
            createdMissionLogger = serializerMissionLogger.save()
            logger.info("Mission Logger is saved successfully.")
        else:
            msg = f"Mission Logger is not valid. Error: {serializerMissionLogger.errors}."
            from .consumers import ErrorMsg

            ErrorMsg.set_message_and_error(logger, Drone.objects.get(pk=dronePK).operation.operation_name, msg)


class DeviceList(LoginRequiredMixin, generics.ListAPIView):
    queryset = OnlineSession.objects.all()

    def get(self, request, *args, **kwargs):
        devices = Device.objects.all()
        return render(request, "aiders/devices.html", {"devices": devices})


class MissionRetrieveAPIView(LoginRequiredMixin, generics.ListAPIView):
    success_url = reverse_lazy("home")

    def get(self, request, *args, **kwargs):
        replay_sessions = OnlineSession.objects.filter(operation__operation_name=self.kwargs.get("operation_name"))
        return render(request, "aiders/missions.html", {"replay_sessions": replay_sessions, "operation_name": self.kwargs.get("operation_name")})


class ReplayMissionOnlineAPIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        replaySession = OnlineSession.objects.get(id=self.kwargs.get("replay_session_id"))
        replayData = {
            "start_time": int(replaySession.start_time.timestamp()),
            "end_time": int(replaySession.end_time.timestamp()) if replaySession.end_time else int(timezone.now().timestamp()),
            "drones": [],
            "devices": [],
            "baloras": [],
            "time_series_data": [],
        }

        droneIdsList = Telemetry.getAllDronesOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        for droneId in droneIdsList:
            replayData["drones"].extend(Drone.getDroneFromIdToJsonFormat(droneId))
        listOfTelemetryObjects = Telemetry.getAllTelemetriesOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(Telemetry.convertListTelemetryToJsonFormat(listOfTelemetryObjects))
        # Drone
        for droneWithLiveData in replayData["drones"]:
            # List Live Frames
            listOfRawFrameObjects = RawFrame.getAllFramesOfDroneBetweenTwoTimes(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(RawFrame.convertListFramesToJsonFormat(listOfRawFrameObjects))
            # List Detection
            listOfDetectionFrameObjects = DetectionFrame.getAllDetectionFrameOfDroneBetweenTwoTimes(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(DetectionFrame.convertListDetectionFramesToJsonFormat(listOfDetectionFrameObjects))
            # List Detection
            listOfDetectionObjectObjects = DetectedObject.getAllDetectionObjectsOfDroneBetweenTwoTimes(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(DetectedObject.convertListDetectionObjectsToJsonFormat(listOfDetectionObjectObjects))
            # List Build Map
            listOfBuildMapObjects = BuildMapImage.getAllBuildMapImagesOfDroneBetweenTwoTimes(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(BuildMapImage.convertListBuildMapImagesToJsonFormat(listOfBuildMapObjects))
            # List Weather Drone
            listOfDroneWeatherObjects = WeatherStation.getAllWeatherDataOfDroneBetweenTwoTimes(
            droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(WeatherStation.convertListWeatherDataToJsonFormat(listOfDroneWeatherObjects))
            # List Missions
            listOfMissionObjects = MissionLog.getAllMissionLogOfDroneBetweenTwoTime(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(MissionLog.convertListMissionLogToJsonFormat(listOfMissionObjects))
            # List Error Messages
            listOfErrorMessageObjects = ErrorMessage.getAllErrorMessageOfDroneBetweenTwoTimes(
                droneWithLiveData["drone_name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(ErrorMessage.convertListErrorMessageToJsonFormat(listOfErrorMessageObjects))

        # Device
        deviceIdsList = DeviceTelemetry.getAllDevicesOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        for deviceId in deviceIdsList:
            replayData["devices"].extend(Device.getDeviceFromIdToJsonFormat(deviceId))
        listOfDeviceTelemetryObjects = DeviceTelemetry.getAllTelemetriesOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(DeviceTelemetry.convertListDeviceTelemetryToJsonFormat(listOfDeviceTelemetryObjects))
        for deviceWithLiveData in replayData["devices"]:
            listOfDeviceImageObjects = DeviceImage.getAllDeviceImagesOfDeviceBetweenTwoTimes(
                deviceWithLiveData["name"], replaySession.start_time, replaySession.end_time or timezone.now()
            )
            replayData["time_series_data"].extend(DeviceImage.convertListDeviceImagesToJsonFormat(listOfDeviceImageObjects))

        # Balora
        baloraIdsList = BaloraTelemetry.getAllBalorasOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        for baloraId in baloraIdsList:
            replayData["baloras"].extend(Balora.getBaloraFromIdToJsonFormat(baloraId))
        listOfBaloraTelemetryObjects = BaloraTelemetry.getAllTelemetriesOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(BaloraTelemetry.convertListBaloraTelemetryToJsonFormat(listOfBaloraTelemetryObjects))
        # List Weather Data
        listOfWeatherObjects = WeatherStation.getAllWeatherDataOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(WeatherStation.convertListWeatherDataToJsonFormat(listOfWeatherObjects))
        # List Algorithm Data
        listOfAlgorithmObjects = Algorithm.getAllAlgorithmOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(Algorithm.convertListAlgorithmToJsonFormat(listOfAlgorithmObjects))
        # List Error
        listOfErrorMessageObjects = ErrorMessage.getAllErrorMessageOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(ErrorMessage.convertListErrorMessageToJsonFormat(listOfErrorMessageObjects))

        listOfUserInputObjects = FrontEndUserInput.getUserInputDataOfOperationBetweenTwoTimes(
            self.kwargs.get("operation_name"), replaySession.start_time, replaySession.end_time or timezone.now()
        )
        replayData["time_series_data"].extend(FrontEndUserInput.convertListUserInputDataToJsonFormat(listOfUserInputObjects))

        replayData["time_series_data"] = sorted(replayData["time_series_data"], key=lambda x: x["time"])
        return render(
            request,
            "aiders/replay_mission.html",
            {
                "replay_data": replayData,
                "operation_name": self.kwargs.get("operation_name"),
                "operation": Operation.objects.get(operation_name=self.kwargs.get("operation_name")),
                "use_online_map": UserPreferences.objects.get(user=request.user).use_online_map,
            },
        )


class TelemetryListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = Telemetry.objects.all().order_by("-time")[:10]
    serializer_class = TelemetrySerializer


# class ControlDeviceDataAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
#     def control_device_save_data_to_db(self):
#         try:
#             ControlDevice.objects.create(
#                 drone=self["drone"],
#                 cpu_usage=self["cpu_usage"],
#                 cpu_core_usage=self["cpu_core_usage"],
#                 cpu_core_frequency=self["cpu_core_frequency"],
#                 cpu_temp=self["cpu_temp"],
#                 cpu_fan_RPM=self["cpu_fan_RPM"],
#                 gpu_usage=self["gpu_usage"],
#                 gpu_frequency=self["gpu_frequency"],
#                 gpu_temp=self["gpu_temp"],
#                 ram_usage=self["ram_usage"],
#                 swap_usage=self["swap_usage"],
#                 swap_cache=self["swap_cache"],
#                 emc_usage=self["emc_usage"],
#             )
#         except Exception as e:
#             logger.error(f'Control Device {self["drone"].drone_name} Serializer data are not valid. Error: {e}.')


class TelemetryRetrieveAPIView(LoginRequiredMixin, generics.RetrieveUpdateDestroyAPIView):
    # queryset = Telemetry.objects.all().select_related('drone')

    serializer_class = TelemetrySerializer

    def get_object(self):
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        """
        The following query set makes use of the  "Lookups that span relationships
        # lookups-that-span-relationships
        Reference: https://docs.djangoproject.com/en/1.11/topics/db/queries/
        """
        obj = Telemetry.objects.filter(drone__drone_name=drone_name).last()
        if obj is None:
            raise Http404
        self.check_object_permissions(self.request, obj)
        return obj

def postGetMissionPointsFromMissionId(request):
    if request.method == 'POST':
        data = json.loads(request.body)
        missionId = data.get("missionId")
        return JsonResponse({'data': Mission.getMissionDataById(missionId) }, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)

def postGetAllActiveSessionDetectionObjectsFromOperationId(request):
    if request.method == 'POST':
        data = json.loads(request.body)
        operationId = data.get("operationId")
        return JsonResponse({'data': {"detectionObjects": DetectedObject.getAllActiveDetectionObjectsByOperationId(operationId), "detectionDescriptions" : DetectedObjectDescription.getAllActiveDetectionDescriptionsByOperationId(operationId)} }, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)
    
def postGetAllDetectionInfoFromDroneIdAndSessionId(request):
    if request.method == 'POST':
        data = json.loads(request.body)
        droneId   = data.get("droneId")
        sessionId = data.get("sessionId")
        return JsonResponse({'data': {"detectionInfo": DetectionInfo.getAllDetectionInfoByDroneIdAndSessionId(droneId, sessionId)} }, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)


class UserList(LoginRequiredMixin, generics.ListAPIView):
    queryset = get_user_model().objects.all()
    serializer_class = UserSerializer

    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_users"):
            return HttpResponseForbidden("You do not have permission.")
        users = User.objects.exclude(username="AnonymousUser")
        return render(request, "aiders/users.html", {"users": users})


class DroneList(LoginRequiredMixin, generics.ListAPIView):

    # queryset = Drone.objects.all()
    # serializer_class = DroneSerializer

    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_operations"):
            return HttpResponseForbidden("You do not have permission.")
        drones = Drone.objects.all()
        return render(request, "aiders/drones.html", {"drones": drones})

    # def save_drone_to_db(self):
    #     serializer = DroneSerializer(data=self)
    #     if serializer.is_valid():
    #         drone = serializer.save()
    #         logger.info(f"Drone Serializer id {drone.pk} is saved.")
    #     else:
    #         logger.error(f"Drone Serializer data are not valid. Error: {serializer.errors}.")


class DeviceList(LoginRequiredMixin, generics.ListAPIView):
    # queryset = Device.objects.all()
    # serializer_class = DeviceSerializer

    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_operations"):
            return HttpResponseForbidden("You do not have permission.")        
        devices = Device.objects.all()
        return render(request, "aiders/devices.html", {"devices": devices})

    # def save_device_to_db(self):
    #     serializer = DeviceSerializer(data=self)
    #     if serializer.is_valid():
    #         device = serializer.save()
    #         logger.info(f"Device Serializer id {device.pk} is saved.")
    #     else:
    #         logger.error(f"Device Serializer data are not valid. Error: {serializer.errors}.")


class BaloraList(LoginRequiredMixin, generics.ListAPIView):
    # queryset = BaloraMaster.objects.all()
    # serializer_class = LoraSerializer

    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_operations"):
            return HttpResponseForbidden("You do not have permission.")           
        loras = BaloraMaster.objects.all()
        return render(request, "aiders/balora.html", {"loras": loras})

    # def save_lora_to_db(self):
    #     serializer = LoraSerializer(data=self)
    #     if serializer.is_valid():
    #         balora = serializer.save()
    #     else:
    #         logger.error(f"Balora Serializer data are not valid. Error: {serializer.errors}.")

    def save_lora_network_to_db(self, baloraMaster):
        return (
            Balora.objects.get(name=self)
            if Balora.objects.filter(name=self).exists()
            else Balora.objects.create(baloraMaster=baloraMaster, name=self)
        )

    def save_lora_telemetry_to_db(self):
        serializer = LoraTelemetrySerializer(data=self)
        if serializer.is_valid():
            loraTelemetry = serializer.save()
        else:
            logger.error(f"Balora Serializer data are not valid. Error: {serializer.errors}.")


class StaticCameraList(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_operations"):
            return HttpResponseForbidden("You do not have permission.")           
        static_cameras = StaticCamera.objects.all()
        return render(request, "aiders/static_cameras.html", {"static_cameras": static_cameras})


class GroundVehicleList(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.manage_operations"):
            return HttpResponseForbidden("You do not have permission.")
        ground_vehicles = GroundVehicle.objects.all()
        return render(request, "aiders/ground_vehicles.html", {"ground_vehicles": ground_vehicles})


@login_required
def static_camera_create_view(request):
    if not request.user.has_perm("aiders.manage_operations"):
        return HttpResponseForbidden("You do not have permission.")
        
    if request.method == "POST":
        form = StaticCameraForm(request.POST)
        if form.is_valid():
            form.save()
            # TODO: start stream rebroadcast if connected_with_platform is True
            # if form.cleaned_data['connected_with_platform']:
            #     startStreamRebroadcast(static_camera.name, static_camera.stream_url)            
            messages.success(request, "Static Camera created successfully.")
            return redirect('static_cameras_list')
    else:
        form = StaticCameraForm()
    
    return render(request, "aiders/static_camera_form.html", {"form": form, "title": "Add New Static Camera"})


@login_required
def static_camera_edit_view(request, camera_id):
    if not request.user.has_perm("aiders.manage_operations"):
        return HttpResponseForbidden("You do not have permission.")
    
    try:
        static_camera = StaticCamera.objects.get(id=camera_id)
    except StaticCamera.DoesNotExist:
        messages.error(request, "Static camera not found.")
        return redirect("static_cameras_list")
    
    if request.method == "POST":
        form = StaticCameraForm(request.POST, instance=static_camera)
        if form.is_valid():
            form.save()
            # TODO: start stream rebroadcast if connected_with_platform is True
            # if form.cleaned_data['connected_with_platform']:
            #     startStreamRebroadcast(static_camera.name, static_camera.stream_url)
            messages.success(request, f"Static camera '{form.cleaned_data['name']}' updated successfully!")
            return redirect("static_cameras_list")
        else:
            messages.error(request, "Please correct the errors below.")
    else:
        form = StaticCameraForm(instance=static_camera)
    
    return render(request, "aiders/static_camera_form.html", {"form": form, "title": "Edit Static Camera"})


class BaloraModifyOperationView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        lora_name = self.kwargs.get("lora_name")
        response = Operation.objects.filter(baloras_to_operate=BaloraMaster.objects.get(name=lora_name).pk)
        response = core_serializers.serialize("json", response)
        lora_data = BaloraMaster.objects.get(name=lora_name)
        response = json.loads(response)
        for data in response:
            if str(data["fields"]["operation_name"]) == str(lora_data.operation):
                data["fields"].update({"Selected": "Selected"})
        response = json.dumps(response)
        return HttpResponse(response)

    def post(self, request, *args, **kwargs):
        operation_name = request.POST["operation_name"]
        lora_name = self.kwargs.get("lora_name")
        balora = BaloraMaster.objects.get(name=lora_name)
        if operation_name == "None":
            balora.operation = None
            balora.save()
        else:
            try:
                balora.operation = Operation.objects.get(operation_name=operation_name)
                balora.save()
            except Exception:
                return HttpResponseNotFound("Operation not found", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse("lora_name", status=status.HTTP_202_ACCEPTED)


class UserDetail(LoginRequiredMixin, generics.RetrieveAPIView):
    queryset = get_user_model().objects.all()
    serializer_class = UserSerializer


class AlgorithmRetrieveView(LoginRequiredMixin, View):
    queryset = Algorithm.objects.all()
    serializer_class = AlgorithmSerializer

    def get(self, request, *args, **kwargs):
        attribute = self.kwargs.get("attr")
        """
        Retrieve the algorithm with the specified id
        but only the "input" or "output" attribute
        """
        if attribute not in ["input", "output"]:
            return Response(status=status.HTTP_400_BAD_REQUEST)
        pk = self.kwargs.get("pk")

        algorithm = get_object_or_404(Algorithm.objects.filter(pk=pk))
        serializer = AlgorithmSerializer(algorithm)
        # res = Response(serializer.data)
        # attr = res.data.get(attribute)
        # res.data = attr
        attr_json = serializer.data.get(attribute)
        attr_html_table = json2html.convert(json=attr_json)

        return render(request, "aiders/algorithm_info.html", {"attr_name": attribute, "attr_object_html_format": attr_html_table})
        # return serializer.data.get(attribute)
        # if (attribute == 'input'):
        #     serializer = AlgorithmSerializer(algorithm)
        #     res = Response(serializer.data)
        #     return res
        # elif (attribute == 'output'):

        # return Response(status=status.HTTP_404_NOT_FOUND)
        # qs = Algorithm.objects.filter(pk=pk).only('output').values()
        # obj = get_object_or_404(qs)
        # self.check_object_permissions(self.request, obj)
        # return obj
        # Get first object from all objects on Algorithm
        # obj = Algorithm.objects.all().first()
        # self.check_object_permissions(self.request, obj)
        # return obj

    def save_algorithm_to_db(self):
        serializer = AlgorithmSerializer(data=self)
        if serializer.is_valid():
            serializer.save()
            logger.info("Algorithm Serializer is saved.")
        else:
            logger.error(f"Algorithm Serializer data are not valid. Error: {serializer.errors}.")


class ManageOperationsView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        operations = Operation.objects.all()
        users = User.objects.all()
        # Get user's permissions for each operation
        user_view_operation_ids = []
        for op in operations:
            permission_name = f"aiders.view_operation_{op.id}"  # Dynamic permission name
            if request.user.has_perm(permission_name):
                user_view_operation_ids.append(op.id)

        user_manage_operation_ids = []
        for op in operations:
            permission_name = f"aiders.manage_operation_{op.id}"  # Dynamic permission name
            if request.user.has_perm(permission_name):
                user_manage_operation_ids.append(op.id)

        # get chat id for each operation
        for operation in operations:
            try:
                operation.chat_id = ChatRoom.objects.filter(operation_id=operation.id).last().id
                if not operation.chat_id:
                    operation.chat_id = "No chat ID available"

                # print(operation.chat_id, flush=True)
            except Exception as e:
                operation.chat_id = "No chat ID available"
                logger.error(f"Error retrieving chat ID for operation {operation.id}: {e}")

        return render(request, "aiders/manage_operations.html", {"operations": operations, "users": users, "user_view_operation_ids":user_view_operation_ids, "user_manage_operation_ids":user_manage_operation_ids})


# class JoinOperationView(LoginRequiredMixin,View):
#     def get(self, request, *args, **kwargs):
#         operation_id = self.kwargs.get("operation_id")
#         operation = Operation.objects.get(pk=operation_id)
#         return render(request, 'aiders/join_operation.html', {'operation': operation})


# class ManagePermissionsView(LoginRequiredMixin, generic.UpdateView):
#     def get(self, request, *args, **kwargs):
#         if not request.user.has_perm("aiders.manage_users"):
#             raise PermissionDenied("You do not have permission to read the permissions.")
#         users = User.objects.exclude(username="AnonymousUser")
#         for user in users:
#             self.add_user_perm(user)
#         operation_groups = ""
#         all_groups = Group.objects.all()
#         for group in all_groups:
#             if str(group.name).__contains__(" operation join"):
#                 operation_groups = operation_groups + (group.name).replace(" operation join", "") + ","
#                 print("operation_groups", operation_groups, flush=True)

#         return render(request, "aiders/manage_permissions.html", {"users": users, "all_groups": operation_groups})

    # def post(self, request, *args, **kwargs):
    #     if not request.user.has_perm("aiders.manage_users"):
    #         raise PermissionDenied("You do not have permission to change the permissions.")
    #     User.updateUserAllowExecuteCommandsById(request.POST.getlist("execute_commands"))
    #     # for user in User.objects.exclude(username="AnonymousUser"):
    #     #     User.update_permissions(user.id, "permission_edit_permissions", str(user.id) in request.POST.getlist("permission_edit_permissions"))
    #     #     User.update_permissions(user.id, "permission_create_operations", str(user.id) in request.POST.getlist("permission_create_operations"))
    #     users = User.objects.exclude(username="AnonymousUser")

    #     for user in users:
    #         self.add_user_perm(user)
    #     operation_groups = ""
    #     all_groups = Group.objects.all()
    #     for group in all_groups:
    #         if str(group.name).__contains__(" operation join"):
    #             operation_groups = operation_groups + (group.name).replace(" operation join", "") + ","
    #     return render(request, "aiders/manage_permissions.html", {"users": users, "all_groups": operation_groups}, status=status.HTTP_202_ACCEPTED)

    # def add_user_perm(self, user):
    #     user.permission_edit_permissions = user.has_perm("aiders.manage_users")
    #     user.permission_create_operations = user.has_perm("aiders.manage_operations")
    #     user.save()


# class ManageUserPermissionsView(LoginRequiredMixin, generic.UpdateView):
#     def post(self, request, *args, **kwargs):
#         if not request.user.has_perm("aiders.manage_users"):
#             raise PermissionDenied("You do not have permission to change the permissions.")
#         user_name = self.kwargs.get("user_name")
#         group_list = request.POST.get("selected")
#         group_list = group_list.split(",")
#         for group in Group.objects.all():
#             if str(group.name).__contains__(" operation join"):
#                 User.objects.filter(username=user_name)[0].groups.remove(group)
#         for group_name in group_list:
#             group_object = Group.objects.filter(name=f"{group_name} operation join").last()
#             User.objects.filter(username=user_name)[0].groups.add(group_object)
#         return HttpResponse(status=status.HTTP_200_OK)


def users_permissions_view(request):
    if not request.user.has_perm("aiders.manage_users"):
        raise PermissionDenied("You do not have permission to view the permissions.")
    # Retrieve all users and permissions
    users = User.objects.all().exclude(username="AnonymousUser")
    permissions = Permission.objects.filter(id__gt=264).exclude(codename__regex=r'\d$').order_by('id')

    # Pass the data to the template
    context = {
        'users': users,
        'permissions': permissions,
    }
    return render(request, 'aiders/users_permissions_view.html', context)


def deactivate_user_account(request, user_id):
    if not request.user.has_perm("aiders.manage_users"):
        raise PermissionDenied("You do not have permission to deactivate user accounts.")
    if int(user_id) == int(request.user.id):
        raise PermissionDenied("You cannot deactivate your own account.")
    else:
        user = get_object_or_404(User, pk=user_id)
        user.is_active = False
        user.save()
    return redirect('users')  # Redirect to the users permissions view after deactivation


def activate_user_account(request, user_id):
    if not request.user.has_perm("aiders.manage_users"):
        raise PermissionDenied("You do not have permission to deactivate user accounts.")
    if int(user_id) == int(request.user.id):
        raise PermissionDenied("You cannot activate your own account.")
    else:   
        user = get_object_or_404(User, pk=user_id)
        user.is_active = True
        user.save()
    return redirect('users')  # Redirect to the users permissions view after activation



def index(request):
    """
    Triggered when the main page of the web app is loaded on browser
    :param request:
    """
    context = {"auth_form": AuthenticationForm}
    if request.user.is_authenticated:

        userQuery = User.objects.filter(pk=request.user.id)
        user = get_object_or_404(userQuery)


        if joined_op_obj := user.joined_operation:
            if request.method == "POST":
                previous_page = resolve(request.POST.get("next", "/")).func.view_class
                if previous_page == AlgorithmListView:
                    """
                    Check if we got here after user selected to show results for some algorithms (That is, if we got here from aiders/algorithms.html)
                    If this is the case, save the results to the request session and then redirect again to this page
                    This is because if we don't redirect, the "POST" request will persist.
                    Reference: https://stackoverflow.com/a/49178154/15290071
                    """
                    algorithm_result_ids = request.POST.getlist("checkedAlgoResultIDs")
                    request.session["checkedAlgoResultIDs"] = algorithm_result_ids
                    return HttpResponseRedirect(reverse("home"))
            elif request.method == "GET":
                disasterEpicenterGPS=Operation.getDisasterEpicenterGPSByOperationId(joined_op_obj.id)
                booGPS=Operation.getBooGPSByOperationId(joined_op_obj.id)
                context = {
                    "operation": joined_op_obj,
                    "net_ip": os.environ.get("NET_IP"),
                    "nginx_port": os.environ.get("NGINX_PORT"),
                    "video_and_cv_remote": os.environ.get("VIDEO_AND_CV_REMOTE", 0),
                    "legacy_lsc": os.environ.get("LEGACY_LSC", 0),
                    "mtx_user": os.environ.get("MTX_USER", ""),
                    "mtx_pass": os.environ.get("MTX_PASS", "")}
                if disasterEpicenterGPS is not None:
                    context["disaster_epicenter_latitude"] = disasterEpicenterGPS['latitude']
                    context["disaster_epicenter_longitude"] = disasterEpicenterGPS['longitude']
                if booGPS is not None:
                    context["boo_latitude"] = booGPS['latitude']
                    context["boo_longitude"] = booGPS['longitude']
                user_wants_to_load_algorithm_results_on_map = request.session.get("checkedAlgoResultIDs") != None
                if user_wants_to_load_algorithm_results_on_map:
                    algorithm_result_ids = request.session.get("checkedAlgoResultIDs")
                    try:
                        qs = Algorithm.objects.filter(pk__in=algorithm_result_ids)
                        algorithm_results = get_list_or_404(qs)
                        algorithm_results = core_serializers.serialize("json", algorithm_results, fields=("pk", "algorithm_name", "output"))
                        context["algorithm_results"] = algorithm_results
                        del request.session["checkedAlgoResultIDs"]
                    except Exception:
                        context.pop("algorithm_results", None)
                else:
                    context.pop("algorithm_results", None)

        else:
            context = {"join_operation_form": JoinOperationForm}

        

        use_online_map = UserPreferences.objects.get(user=request.user).use_online_map
        # context = {'auth_form': AuthenticationForm,'use_online_map':use_online_map}

        # retrieve key for interacting with APIs
        from rest_framework.authtoken.models import Token
        try:
            token = Token.objects.get(user=user)
        except Token.DoesNotExist:
            token = Token.objects.create(user=user)
        context["token"] = token.key

        context["use_online_map"] = use_online_map
        return render(request, "aiders/platform.html", context)

    # return render(request, "aiders/login.html", context)
    return redirect("login")


class DroneModifyOperationView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        drone_name = self.kwargs.get("drone_name")
        response = Operation.objects.filter(drones_to_operate=Drone.objects.get(drone_name=drone_name).pk)
        response = core_serializers.serialize("json", response)
        drone_data = Drone.objects.get(drone_name=drone_name)
        response = json.loads(response)
        for data in response:
            if str(data["fields"]["operation_name"]) == str(drone_data.operation):
                data["fields"].update({"Selected": "Selected"})
        response = json.dumps(response)
        return HttpResponse(response)

    def post(self, request, *args, **kwargs):
        operation_name = request.POST["operation_name"]
        drone_name = self.kwargs.get("drone_name")
        drone = Drone.objects.get(drone_name=drone_name)
        if operation_name == "None":
            drone.operation = None
            drone.save()
        else:
            try:
                drone.operation = Operation.objects.get(operation_name=operation_name)
                drone.save()
            except Exception:
                return HttpResponseNotFound("Operation not found", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse(drone_name, status=status.HTTP_202_ACCEPTED)


class DeviceModifyOperationView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        name = self.kwargs.get("device_name")
        response = Operation.objects.filter(devices_to_operate=Device.objects.get(name=name).pk)
        response = core_serializers.serialize("json", response)
        device_data = Device.objects.get(name=name)
        response = json.loads(response)
        for data in response:
            if str(data["fields"]["operation_name"]) == str(device_data.operation):
                data["fields"].update({"Selected": "Selected"})
        response = json.dumps(response)
        return HttpResponse(response)

    def post(self, request, *args, **kwargs):
        operation_name = request.POST["operation_name"]
        device_name = self.kwargs.get("device_name")
        device = Device.objects.get(name=device_name)
        if operation_name == "None":
            device.operation = None
            device.save()
        else:
            try:
                device.operation = Operation.objects.get(operation_name=operation_name)
                device.save()
            except Exception:
                return HttpResponseNotFound("Operation not found", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse(device_name, status=status.HTTP_202_ACCEPTED)


class DeviceNewSessionView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        device = Device.objects.get(name=self.kwargs.get("device_name"))
        DeviceSession.objects.filter(is_active=True, device=device).update(
            is_active=False, end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ)
        )
        DeviceSession.objects.create(
            user=request.user,
            operation=device.operation,
            device=device,
            is_active=True,
            folder_path=Constants.DEVICE_IMAGE_DIR_PREFIX + self.kwargs.get("device_name") + "_",
        )
        return HttpResponse(status=status.HTTP_202_ACCEPTED)


class DeviceStopSessionView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        DeviceSession.objects.filter(is_active=True, device=Device.objects.get(name=self.kwargs.get("device_name"))).update(
            is_active=False, end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ)
        )
        return HttpResponse(status=status.HTTP_202_ACCEPTED)

def getActiveDeviceSessionImagesByDeviceId(request, *args, **kwargs):
    if request.method == "POST":
        data = json.loads(request.body)
        deviceId = data.get("deviceId")
        latestReceivedImageId = data.get("latestReceivedImageId")
        maxShownImages = data.get("maxShownImages")
        latestSession = DeviceSession.objects.filter(
            is_active=True,
            device_id=deviceId,
        ).last()
        if not latestSession:
            return JsonResponse({"message": "There is no active Device."}, status=400)
        listDeviceImages = list(DeviceImage.objects.filter(device__id=deviceId, session__id=latestSession.id, id__gt=latestReceivedImageId).order_by('-time')[:maxShownImages].values())
        # Format timestamp
        for image in listDeviceImages:
            image['time'] = image['time'].strftime('%H:%M:%S')
        return JsonResponse({"data": listDeviceImages}, status=200)
    else:
        return JsonResponse({"message": "Invalid request method. Only POST requests are accepted."}, status=400)


class BuildMapStartOrStopSession(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        if not User.checkIfUserAllowToExecudeCommands(request.user):
            return HttpResponseForbidden("You do not have permission to execute commands.")
        operationName = self.kwargs.get("operation_name")

        postData = json.loads(request.body)
        print(postData, flush=True)
        droneName = postData.get("drone_name")
        activateBuildMap = postData.get("start_build_map_boolean")
        interval = postData.get("interval")

        drone = Drone.objects.get(drone_name=droneName)
        if activateBuildMap == True:
            build_map_request_handler.PostRequestForBuildMapStartOrStop(droneName, "START", interval, drone.connection_type)
            latestActiveBuildMapSession = BuildMapSession.getLatestActiveSessionIdByDroneId(drone.id)
            drone.build_map_activated = True
            drone.save()
            if latestActiveBuildMapSession:
                print("Already active build map session", flush=True)
                BuildMapSessionId = latestActiveBuildMapSession.id
            else:
                print("Active build map session", flush=True)
                BuildMapSessionId = BuildMapSession.createBuildMapSessionByUserIdOperationNameDrone(User.objects.get(id = request.user.id), operationName, drone)
            logger.info("User sending build map request Start for drone {}.".format(droneName))
            return JsonResponse({'data': BuildMapSessionId}, status=200)
        else:
            drone.build_map_activated = False
            drone.save()
            BuildMapSession.objects.filter(
                operation=Operation.objects.get(operation_name=operationName), drone=drone, is_active=True
            ).update(end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ), is_active=False)
            # BuildMapSessionId = BuildMapSession.deactivateBuildMapSessionByOperationNameDrone(operationName, drone)
            build_map_request_handler.PostRequestForBuildMapStartOrStop(droneName, "STOP", interval, drone.connection_type)
            logger.info("User sending build map request Stop for drone {}.".format(droneName))
            print("STOP build map session", flush=True)
            return JsonResponse({'data': 1}, status=200)

def buildMapGetLatestImages(request, *args, **kwargs):
    if request.method == 'POST':
        data = json.loads(request.body)
        droneId = data.get("droneId")
        latestReceivedImageId = data.get("latestReceivedImageId")
        latestBuildMapSession = BuildMapSession.getLatestActiveSessionIdByDroneId(droneId)

        if not latestBuildMapSession:
            return JsonResponse({'message': 'There is no active Build Map Session.'}, status=400)

        listBuildMapImages = list(BuildMapImage.objects.filter(session__id=latestBuildMapSession.id, id__gt=latestReceivedImageId).order_by('-time').values())
        for image in listBuildMapImages:
            image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            image["top_left"] = [float(image["top_left"].coords[0]), float(image["top_left"].coords[1])]
            image["top_right"] = [float(image["top_right"].coords[0]), float(image["top_right"].coords[1])]
            image["bottom_left"] = [float(image["bottom_left"].coords[0]), float(image["bottom_left"].coords[1])]
            image["bottom_right"] = [float(image["bottom_right"].coords[0]), float(image["bottom_right"].coords[1])]
            image["centre"] = [float(image["centre"].coords[0]), float(image["centre"].coords[1])]
        return JsonResponse({'data': listBuildMapImages}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)

def buildMapGetLatestImagesBySessionId(request, *args, **kwargs):
    if request.method == 'POST':
        data = json.loads(request.body)
        
        buildMapSessionId = data.get("buildMapSessionId")
        listBuildMapImages = list(BuildMapImage.objects.filter(session__id=buildMapSessionId).order_by('-time').values())
        for image in listBuildMapImages:
            image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            image["top_left"] = [float(image["top_left"].coords[0]), float(image["top_left"].coords[1])]
            image["top_right"] = [float(image["top_right"].coords[0]), float(image["top_right"].coords[1])]
            image["bottom_left"] = [float(image["bottom_left"].coords[0]), float(image["bottom_left"].coords[1])]
            image["bottom_right"] = [float(image["bottom_right"].coords[0]), float(image["bottom_right"].coords[1])]
            image["centre"] = [float(image["centre"].coords[0]), float(image["centre"].coords[1])]
        return JsonResponse({'data': listBuildMapImages}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)


class LidarPointsAPIView(LoginRequiredMixin, generic.UpdateView):
    def save_point_in_db(self, drone_name, lidar_session):
        if lidar_session.is_active != True:
            return
        telemetry = Telemetry.objects.filter(drone__drone_name=drone_name).last()
        batch_list = []
        for data_record in self:
            data_record["telemetry"] = telemetry
            data_record["lidar_point_session"] = lidar_session
            batch_list.append(LidarPoint(**data_record))
        LidarPoint.objects.bulk_create(batch_list)


@csrf_exempt
def BuildMapImageView(request):

    if request.method != "POST":
        return
    # print(request.POST, flush=True)
    img_file = request.FILES.get("image_file")
    img_name = request.POST.get("image_name")
    drone_name = request.POST.get("drone_name")
    drone_bearing = float(request.POST.get("bearing"))
    drone_alt = float(request.POST.get("alt"))
    drone_lat = float(request.POST.get("lat"))
    drone_lon = float(request.POST.get("lon"))
    extra_data = False
    try:
        d_roll = float(request.POST.get("d_roll"))
        d_pitch = float(request.POST.get("d_pitch"))
        d_yaw = float(request.POST.get("d_yaw"))
        g_roll = float(request.POST.get("g_roll"))
        g_pitch = float(request.POST.get("g_pitch"))
        g_yaw = float(request.POST.get("g_yaw"))
        extra_data = True
    except Exception:
        extra_data = False

    drone_instance = Drone.objects.get(drone_name=drone_name)
    destinations = img_georeference.calcPoints(
        drone_lat, drone_lon, drone_bearing, drone_alt, img_name, drone_instance.model, drone_instance.camera_model
    )
    try:
        if drone_instance.is_connected_with_platform and drone_instance.build_map_activated:
            Session = BuildMapSession.objects.filter(drone=Drone.objects.get(drone_name=drone_name)).last()
            Image.open(img_file)
            file_name = default_storage.save(os.path.join(Session.folder_path, img_file.name), img_file)
            if extra_data:
                image = BuildMapImage.objects.create(
                    path=f"{Session.folder_path}/{img_name}",
                    top_left=Point(destinations[2].longitude, destinations[2].latitude),
                    top_right=Point(destinations[0].longitude, destinations[0].latitude),
                    bottom_left=Point(destinations[1].longitude, destinations[1].latitude),
                    bottom_right=Point(destinations[3].longitude, destinations[3].latitude),
                    centre=Point(drone_lon, drone_lat),
                    altitude=Decimal(drone_alt),
                    bearing=Decimal(drone_bearing),
                    d_roll=d_roll,
                    d_pitch=d_pitch,
                    d_yaw=d_yaw,
                    g_roll=g_roll,
                    g_pitch=g_pitch,
                    g_yaw=g_yaw,
                    session=Session,
                )
            else:
                image = BuildMapImage.objects.create(
                    path=f"{Session.folder_path}/{img_name}",
                    top_left=Point(destinations[2].longitude, destinations[2].latitude),
                    top_right=Point(destinations[0].longitude, destinations[0].latitude),
                    bottom_left=Point(destinations[1].longitude, destinations[1].latitude),
                    bottom_right=Point(destinations[3].longitude, destinations[3].latitude),
                    centre=Point(drone_lon, drone_lat),
                    altitude=Decimal(drone_alt),
                    bearing=Decimal(drone_bearing),
                    d_roll=0,
                    d_pitch=0,
                    d_yaw=0,
                    g_roll=0,
                    g_pitch=0,
                    g_yaw=0,
                    session=Session,
                )
            logger.info(f"Saved Image Successfully for Build Map Session {Session.id}.")
            return HttpResponse({"status:success"}, status=status.HTTP_200_OK)
    except Exception as e:
        print(e)
        return HttpResponse({"status:failed"}, status=status.HTTP_400_BAD_REQUEST)


@csrf_exempt
def DataImageView(request):
    if request.method == "POST":
        max = 15
        current = 1
        drone_name = "mavic2a"
        if current == 1:
            BuildMapAdvanceSessionInstance = BuildMapAdvanceSession.objects.create(
                drone=Drone.objects.get(drone_name=drone_name),
                folder_path=Constants.BUILD_MAP_ADVANCE_DIR_PREFIX + drone_name + "_",
            )
        else:
            BuildMapAdvanceSessionInstance = BuildMapAdvanceSession.objects.filter(drone=Drone.objects.get(drone_name=drone_name))
        img_file = request.FILES.get("image_file")
        temp_image = Image.open(img_file)
        temp_image.save(default_storage.path(os.path.join(BuildMapAdvanceSessionInstance.folder_path, img_file.name)))
        BuildMapAdvanceImage.objects.create(
            path=f"{BuildMapAdvanceSessionInstance.folder_path}/{img_file.name}",
            session=BuildMapAdvanceSessionInstance,
        )
    return HttpResponse(status=status.HTTP_200_OK)


@csrf_exempt
def DeviceImageView(request):
    if request.method == "POST":
        try:
            print(request.POST, flush=True)
            device_name = request.POST.get("deviceName")
            session = DeviceSession.objects.filter(
                is_active=True,
                device=Device.objects.get(name=device_name),
            ).last()

            if not os.path.exists(default_storage.path(session.folder_path)):
                os.mkdir(default_storage.path(session.folder_path))               

            img_file = request.FILES.get("image_file")
            img_name = request.POST.get("img_name")
            # device_latitude = float(request.POST.get("latitude"))
            # device_longitude = float(request.POST.get("longitude"))

            # get latest device telemetry from the database instead
            device_telemetry = DeviceTelemetry.objects.filter(device__name=device_name).last()
            device_latitude = device_telemetry.latitude
            device_longitude = device_telemetry.longitude

            temp_image = Image.open(img_file)
            temp_image.save(default_storage.path(os.path.join(session.folder_path, img_file.name)))
            DeviceImage.objects.create(
                path=os.path.join(session.folder_path, img_file.name),
                device=Device.objects.get(name=device_name),
                latitude=device_latitude,
                longitude=device_longitude,
                session=session,
            )
        except Exception as e:
            print("Error:", e)
            return HttpResponse(status=status.HTTP_500_INTERNAL_SERVER_ERROR)
        return HttpResponse(status=status.HTTP_200_OK)


class BuildMapLoadAPIView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        operation = Operation.objects.get(operation_name=self.kwargs["operation_name"])
        inactiveSessionsWithImageCount = BuildMapSession.getInactiveSessionAndNumberOfImagesByOperationId(operation.id)
        result = []
        for session in inactiveSessionsWithImageCount:
            result.append({
                'sessionId': session.id,
                'droneName': session.drone.drone_name,
                'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                'endTime': session.end_time.strftime('%Y-%m-%d %H:%M:%S'),
                'count': session.image_count
            })
        return HttpResponse(json.dumps(result))

    def post(self, request, *args, **kwargs):
        try:
            build_map_id = json.loads(request.body.decode("utf-8"))["build_map_id"]
        except Exception:
            return HttpResponse(status=status.HTTP_400_BAD_REQUEST)
        map_build = list(BuildMapImage.objects.filter(session_id=build_map_id).values())
        for data in map_build:
            self.buildMapFixData(data)
        json_string = json.dumps(map_build)
        return HttpResponse(json_string, status=status.HTTP_201_CREATED)

    def buildMapFixData(self, data):
        data["time"] = str(data["time"])
        data["top_left"] = [float(data["top_left"].coords[0]), float(data["top_left"].coords[1])]
        data["top_right"] = [float(data["top_right"].coords[0]), float(data["top_right"].coords[1])]
        data["bottom_left"] = [float(data["bottom_left"].coords[0]), float(data["bottom_left"].coords[1])]
        data["bottom_right"] = [float(data["bottom_right"].coords[0]), float(data["bottom_right"].coords[1])]
        data["centre"] = [float(data["centre"].coords[0]), float(data["centre"].coords[1])]
        data["altitude"] = float(data["altitude"])
        data["bearing"] = float(data["bearing"])


class FirePredictionCreateAPIView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        for jsonPostData in request:
            try:
                PostData = json.loads(jsonPostData)
                if PostData["user"]:
                    operation = Operation.objects.get(operation_name=self.kwargs["operation_name"])
                    operationPK = operation.pk
                    user = User.objects.get(username=PostData["user"])
                    userPK = user.pk
                    algorithmName = "FIRE_PROPAGATION_ALGORITHM"
                    canBeLoadedOnMap = True
                    input = PostData
                    del input["user"]
                    try:
                        output = utils.handleAlgorithmExecution(operationPK, input, canBeLoadedOnMap, algorithmName, userPK)
                    except Exception as e:
                        return HttpResponse(status=status.HTTP_400_BAD_REQUEST)
                    response = "[" + str(output) + "]"
                    return HttpResponse(response, status=status.HTTP_201_CREATED)
            except Exception:
                pass
        raise Http404


# legacy login
def login_view(request):
    if request.method == "GET":
        redirect_to = request.GET.get("next")
        if request.user.is_authenticated:
            if redirect_to != None:
                return HttpResponseRedirect(redirect_to)
            return HttpResponseRedirect(reverse("manage_operations"))
        return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": redirect_to})
    if request.method == "POST":
        username = request.POST["username"]
        password = request.POST["password"]
        redirect_to = request.POST["next"]
        user = authenticate(request, username=username, password=password)
        if user is not None:
            if user.is_active:
                if request.META.get("HTTP_X_FORWARDED_FOR"):
                    ip = request.META.get("HTTP_X_FORWARDED_FOR")
                else:
                    ip = request.META.get("REMOTE_ADDR")

                from user_agents import parse

                user_agent = parse(request.META.get("HTTP_USER_AGENT"))
                """
                When user logs in, save a few data that concern their machine
                """
                terminal = Terminal(
                    ip_address=ip,
                    user=user,
                    os=user_agent.os.family,
                    device=user_agent.device.family,
                    logged_in=True,
                    browser=user_agent.browser.family,
                )
                terminal.save()

                # generate key for interacting with APIs
                from rest_framework.authtoken.models import Token
                Token.objects.filter(user=user).delete()
                token = Token.objects.create(user=user)

                if not UserPreferences.objects.filter(user=user).exists():
                    UserPreferences.objects.create(use_online_map=True, user=user)

                login(request, user, backend="django.contrib.auth.backends.ModelBackend")

                if redirect_to != "None":
                    return HttpResponseRedirect(redirect_to)
                return redirect("manage_operations")
        else:
            messages.error(request, "Wrong username or password!")
            return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": redirect_to})


# legacy logout
def logout_view(request):
    from rest_framework.authtoken.models import Token
    Token.objects.filter(user=request.user).delete()
  
    logout(request)
    # Redirect to a success page
    return redirect("login")



# login for keycloak
def custom_login(request):
    print("****************************** custom_login **********************************", flush=True)
    redirect_to = request.GET.get("next")
    if request.user.is_authenticated:
        if redirect_to != None:
            return HttpResponseRedirect(redirect_to)
        return HttpResponseRedirect(reverse("manage_operations"))
    # return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": redirect_to})
    return redirect("/accounts/oidc/keycloak/login/") # redirect to keycloak login page



# logout for keycloak
def custom_logout(request):
    from rest_framework.authtoken.models import Token
    Token.objects.filter(user=request.user).delete()    
    # from allauth.socialaccount.models import SocialToken
    # id_token = None
    # try:
    #     token = SocialToken.objects.filter(account__user=request.user, account__provider='keycloak').last()
    #     if token and 'id_token' in token.extra_data:
    #         print("ID Token found:", token.extra_data['id_token'], flush=True)
    #         id_token = token.extra_data['id_token']
    #     else:
    #         print("ID Token not found in token.extra_data", flush=True)
    # except Exception as e:
    #     print("Error retrieving id_token:", e, flush=True)

    logout(request) # django logout

    # keycloak_logout_url = (
    #     "http://" + os.environ.get("NET_IP") + ":8083/realms/master/protocol/openid-connect/logout"
    #     f"?post_logout_redirect_uri=http://" + os.environ.get("NET_IP") + ":8888/"
    # )
    # if id_token:
    #     keycloak_logout_url += f"&id_token_hint={id_token}"
    
    keycloak_logout_url = "http://" + os.environ.get("NET_IP") + ":" + os.environ.get("KEYCLOAK_PORT") + "/realms/master/protocol/openid-connect/logout"    
    return redirect(keycloak_logout_url)


# # check if a token exists in the database
# def validate_token(request):
#     from rest_framework.authtoken.models import Token
#     # Get token from query parameters
#     token_key = request.GET.get('token')
#     if not token_key:
#         return JsonResponse({"error": "Token parameter is required."})

#     # Check if token exists
#     try:
#         token = Token.objects.get(key=token_key)
#     except Token.DoesNotExist:
#         return JsonResponse({'valid': False})

#     # If token exists, return valid
#     return JsonResponse({'valid': True})


def new_operation_form_view(request):
    if not request.user.has_perm("aiders.manage_operations"):
        raise PermissionDenied("You do not have permission to create the operation.")
    if request.method == "POST":
        form = NewOperationFormForm(request.POST)
        if form.is_valid():
            return new_operation_form_save(request)
    else:
        form = NewOperationFormForm()
    users = User.objects.all()
    drones = Drone.objects.all()
    devices = Device.objects.all()
    baloras = BaloraMaster.objects.all()
    return render(
        request,
        "aiders/operation_new_form.html",
        {
            "form": form,
            "users": users,
            "drones": drones,
            "devices": devices,
            "baloras": baloras,
        },
    )


def new_operation_form_save(request):
    if request.POST.get("disaster_epicenter_latitude") == '':
        disasterEpicenterLatitudeValue = None
    else:
        disasterEpicenterLatitudeValue = request.POST.get("disaster_epicenter_latitude")
    if request.POST.get("disaster_epicenter_longitude") == '':
        disasterEpicenterLongitudeValue = None
    else:
        disasterEpicenterLongitudeValue = request.POST.get("disaster_epicenter_longitude")

    operation_instance = Operation.objects.create(
        operation_name=request.POST.get("operation_name"),
        location=request.POST.get("location"),
        description=request.POST.get("description"),
        operator=request.user,
        disaster_epicenter_latitude = disasterEpicenterLatitudeValue,
        disaster_epicenter_longitude = disasterEpicenterLongitudeValue,
    )
    # Save Operation Drones
    drone_allow_list = Drone.objects.none()
    for drone_id in request.POST.getlist("drones_allow"):
        drone_allow_list = drone_allow_list | Drone.objects.filter(pk=drone_id)
        if request.POST.getlist("drone_operation") == ["True"] and (
            Drone.objects.get(pk=drone_id).operation is None or Drone.objects.get(pk=drone_id).is_connected_with_platform == False
        ):
            drone_instance = Drone.objects.get(pk=drone_id)
            drone_instance.operation = operation_instance
            drone_instance.save()

        operation_instance.drones_to_operate.set(drone_allow_list)
    # Save Operation Devices
    device_allow_list = Device.objects.none()
    for device_id in request.POST.getlist("devices_allow"):
        device_allow_list = device_allow_list | Device.objects.filter(pk=device_id)
        if request.POST.getlist("device_operation") == ["True"] and (
            Device.objects.get(pk=device_id).operation is None or Device.objects.get(pk=device_id).is_connected_with_platform == False
        ):
            device_instance = Device.objects.get(pk=device_id)
            device_instance.operation = operation_instance
            device_instance.save()
        operation_instance.devices_to_operate.set(device_allow_list)
    # Save Operation Balora
    lora_allow_list = BaloraMaster.objects.none()
    for lora_id in request.POST.getlist("baloras_allow"):
        lora_allow_list = lora_allow_list | BaloraMaster.objects.filter(pk=lora_id)
        if request.POST.getlist("balora_operation") == ["True"] and (
            BaloraMaster.objects.get(pk=lora_id).operation is None or BaloraMaster.objects.get(pk=lora_id).is_connected_with_platform == False
        ):
            lora_instance = BaloraMaster.objects.get(pk=lora_id)
            lora_instance.operation = operation_instance
            lora_instance.save()
        operation_instance.baloras_to_operate.set(lora_allow_list)

    # Create join and manage permissions for the this operation
    ct = ContentType.objects.get_for_model(Operation)
    view_permission = Permission.objects.create(codename=f"view_operation_{operation_instance.id}", name=f"view operation {operation_instance.operation_name}", content_type=ct)
    manage_permission = Permission.objects.create(codename=f"manage_operation_{operation_instance.id}", name=f"manage operation {operation_instance.operation_name}", content_type=ct)
    request.user.user_permissions.add(view_permission)   # give creator access
    request.user.user_permissions.add(manage_permission) # give creator access
    # Add the permissions to the users that are allowed to view/manage the operation
    for user_id in request.POST.getlist("users_allow"):
        current_user = User.objects.get(pk=user_id)
        current_user.user_permissions.add(view_permission)
        if current_user.has_perm("aiders.manage_operations"):
            current_user.user_permissions.add(manage_permission)

    # group_join_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation join") # ?
    # group_edit_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation edit") # ?
    # assign_perm("join_operation", group_join_operation, operation_instance)
    # assign_perm("edit_operation", group_edit_operation, operation_instance)

    # for user_id in request.POST.getlist("users_allow"):
    #     User.objects.filter(pk=user_id)[0].groups.add(group_join_operation)

    # Get or create chat room for this operation
    room, created = ChatRoom.get_or_create_for_operation(operation_instance)
    
    # Add user as member if not already
    member, member_created = ChatRoomMember.objects.get_or_create(
        room=room,
        user=request.user,
        defaults={'is_active': True}
    )
        
    logger.info(f"Operation with id {operation_instance.pk} is created successfully.")
    return redirect("manage_operations")


def edit_operation_form_view(request, operation_name):
    op = Operation.objects.get(operation_name=operation_name)
    if not request.user.has_perm("aiders.manage_operations") or not request.user.has_perm(f"aiders.manage_operation_{op.id}"):
        raise PermissionDenied("You do not have permission to edit the operation.")
    if request.method == "POST":
        if operation_name == request.POST.get("operation_name"):
            return edit_operation_form_save(operation_name, request)
    else:
        operation_instance = Operation.objects.get(operation_name=operation_name)

    users_all = []
    users_allow = []
    for user in User.objects.all():
        if user.username != "AnonymousUser":
            if user.has_perm(f"aiders.view_operation_{operation_instance.id}"):
                users_allow.append(user)
            else:
                users_all.append(user)

    operation_drones_dict = model_to_dict(operation_instance)
    drones_all = set(list(Drone.objects.all())) ^ set(operation_drones_dict["drones_to_operate"])
    drones_allow = set(list(Drone.objects.all())) & set(operation_drones_dict["drones_to_operate"])

    operation_devices_dict = model_to_dict(operation_instance)
    devices_all = set(list(Device.objects.all())) ^ set(operation_devices_dict["devices_to_operate"])
    devices_allow = set(list(Device.objects.all())) & set(operation_devices_dict["devices_to_operate"])

    operation_baloras_dict = model_to_dict(operation_instance)
    baloras_all = set(list(BaloraMaster.objects.all())) ^ set(operation_baloras_dict["baloras_to_operate"])
    baloras_allow = set(list(BaloraMaster.objects.all())) & set(operation_baloras_dict["baloras_to_operate"])
    return render(
        request,
        "aiders/operation_edit_form.html",
        {
            "operation_name": operation_name,
            "operation": operation_instance,
            "users_all": users_all,
            "users_allow": users_allow,
            "drones_all": drones_all,
            "drones_allow": drones_allow,
            "devices_all": devices_all,
            "devices_allow": devices_allow,
            "baloras_all": baloras_all,
            "baloras_allow": baloras_allow,
        },
    )


def edit_operation_form_save(operation_name, request):
    operation_instance = Operation.objects.get(operation_name=operation_name)
    operation_instance.location = request.POST.get("location")
    operation_instance.description = request.POST.get("description")

    # handle base of operations coordinates
    if request.POST.get("boo_latitude") == '':
        booLatitudeValue = None
    else:
        booLatitudeValue = request.POST.get("boo_latitude")
    if request.POST.get("boo_longitude") == '':
        booLongitudeValue = None
    else:
        booLongitudeValue = request.POST.get("boo_longitude")
    operation_instance.boo_latitude = booLatitudeValue
    operation_instance.boo_longitude = booLongitudeValue

    print(operation_instance.boo_latitude, operation_instance.boo_longitude, flush=True)

    # handle disaster epicenter coordinates
    if request.POST.get("disaster_epicenter_latitude") == '':
        disasterEpicenterLatitudeValue = None
    else:
        disasterEpicenterLatitudeValue = request.POST.get("disaster_epicenter_latitude")
    if request.POST.get("disaster_epicenter_longitude") == '':
        disasterEpicenterLongitudeValue = None
    else:
        disasterEpicenterLongitudeValue = request.POST.get("disaster_epicenter_longitude")
    operation_instance.disaster_epicenter_latitude = disasterEpicenterLatitudeValue
    operation_instance.disaster_epicenter_longitude = disasterEpicenterLongitudeValue

    operation_instance.save()
    # Group.objects.get(name=f"{operation_instance.operation_name} operation join").delete()
    # group_join_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation join")
    # assign_perm("join_operation", group_join_operation, operation_instance)

    # for user_id in request.POST.getlist("users_allow"):
    #     User.objects.filter(pk=user_id)[0].groups.add(group_join_operation)

    # Get join and manage permissions for the this operation
    view_permission = Permission.objects.get(codename=f"view_operation_{operation_instance.id}")
    manage_permission = Permission.objects.get(codename=f"manage_operation_{operation_instance.id}")
    # Remove the users that already had the permissions but were removed from the operation
    for user_id in request.POST.getlist("users_all"):
        current_user = User.objects.get(pk=user_id)
        current_user.user_permissions.remove(view_permission)
        current_user.user_permissions.remove(manage_permission)
    request.user.user_permissions.add(view_permission)   # give creator access
    request.user.user_permissions.add(manage_permission) # give creator access        
    # Add the permissions to the users that are allowed to view/manage the operation
    for user_id in request.POST.getlist("users_allow"):
        current_user = User.objects.get(pk=user_id)
        current_user.user_permissions.add(view_permission)
        if current_user.has_perm("aiders.manage_operations"):
            current_user.user_permissions.add(manage_permission)

    # Save Operation Drones
    drone_allow_list = Drone.objects.none()
    for drone_id in request.POST.getlist("drones_allow"):
        drone_allow_list = drone_allow_list | Drone.objects.filter(pk=drone_id)
        if request.POST.getlist("drone_operation") == ["True"] and (
            Drone.objects.get(pk=drone_id).operation is None or Drone.objects.get(pk=drone_id).is_connected_with_platform == False
        ):
            drone_instance = Drone.objects.get(pk=drone_id)
            drone_instance.operation = operation_instance
            drone_instance.save()
    operation_instance.drones_to_operate.set(drone_allow_list)
    for drone_id in request.POST.getlist("drones_all"):
        drone_instance = Drone.objects.get(pk=drone_id)
        drone_instance.operation = None
        drone_instance.save()

    # Save Operation Devices
    device_allow_list = Device.objects.none()
    for device_id in request.POST.getlist("devices_allow"):
        device_allow_list = device_allow_list | Device.objects.filter(pk=device_id)
        if request.POST.getlist("device_operation") == ["True"] and (
            Device.objects.get(pk=device_id).operation is None or Device.objects.get(pk=device_id).is_connected_with_platform == False
        ):
            device_instance = Device.objects.get(pk=device_id)
            device_instance.operation = operation_instance
            device_instance.save()
    operation_instance.devices_to_operate.set(device_allow_list)
    for device_id in request.POST.getlist("devices_all"):
        device_instance = Device.objects.get(pk=device_id)
        device_instance.operation = None
        device_instance.save()

    # Save Operation Baloras
    balora_allow_list = BaloraMaster.objects.none()
    for balora_id in request.POST.getlist("baloras_allow"):
        balora_allow_list = balora_allow_list | BaloraMaster.objects.filter(pk=balora_id)
        if request.POST.getlist("balora_operation") == ["True"] and (
            BaloraMaster.objects.get(pk=balora_id).operation is None or BaloraMaster.objects.get(pk=balora_id).is_connected_with_platform == False
        ):
            balora_instance = BaloraMaster.objects.get(pk=balora_id)
            balora_instance.operation = operation_instance
            balora_instance.save()
    operation_instance.baloras_to_operate.set(balora_allow_list)
    for balora_id in request.POST.getlist("baloras_all"):
        balora_instance = BaloraMaster.objects.get(pk=balora_id)
        balora_instance.operation = None
        balora_instance.save()

    logger.info(f"Operation with id {operation_instance.pk} is modified successfully.")
    return redirect("manage_operations")


def about_view(request):
    """About Us page view - accessible to all users"""
    return render(request, "aiders/about.html")

class ExecuteAlgorithmAPIView(LoginRequiredMixin, APIView):
    def post(self, request, *args, **kwargs):
        if not User.checkIfUserAllowToExecudeCommands(request.user):
            return HttpResponseForbidden("You do not have permission to execute commands.")
        return Response(
            utils.handleAlgorithmExecution(
                Operation.objects.get(operation_name=kwargs["operation_name"]).pk,
                request.data["input"],
                request.data["canBeLoadedOnMap"],
                request.data["algorithmName"],
                request.user.pk,
            )
        )


# class ExecuteMissionAPIView(LoginRequiredMixin, APIView):
class ExecuteMissionAPIView(APIView):
    # def get(self, request, *args, **kwargs):
    #     operation_name = kwargs["operation_name"]
    #     drone_name = kwargs["drone_name"]
    #     user = request.user
    #     operation = Operation.objects.get(operation_name=operation_name)
    #     drone = Drone.objects.get(drone_name=drone_name)

    #     mission_log = MissionLog.objects.filter(action="START_MISSION", user=user.pk, drone=drone, operation=operation).last()
    #     return Response(mission_log.mission.mission_type)


    def post(self, request, *args, **kwargs):
        # if not User.checkIfUserAllowToExecudeCommands(request.user):
        #     return HttpResponseForbidden("You do not have permission to execute commands.")
        
        actionDetails = request.data
        # TODO: check if user is logged in with django
        # TODO: ELSE
        # TODO: check bearer token against Keycloak
        # TODO: figure out a way to assign the mission to a user_id if missions comes from Kafka
        user_id = 2 # TODO: get the user's PK from the request
        # TODO: check if user has permission to issue commands

        operation_name = kwargs["operation_name"]
        drone_name = kwargs["drone_name"]
        # user_name = request.user.username
        operation = Operation.objects.get(operation_name=operation_name)

        action = actionDetails["action"]
        grid = actionDetails["grid"]
        captureAndStoreImages = actionDetails["captureAndStoreImages"]
        missionPath = actionDetails["mission_points"]
        missionSpeed = actionDetails["mission_speeds"]
        missionGimbal = actionDetails["mission_gimbal"]
        missionRepeat = int(actionDetails["mission_repeat"])
        # for index in range(len(missionPath)):
        #     if missionPath != missionPath[index][2]:
        #         missionPath[index][2] = missionPath[index][2][0]
        #     else:
        #         missionPath[index][2] = missionPath[index][2][index]

        dronePK = Drone.objects.get(drone_name=drone_name).pk
        try:
            missionType = actionDetails["mission_type"]
        except Exception:
            missionType = None
        mission_request_handler.publishMissionToRos(
            operation.pk,
            missionType,
            drone_name,
            grid,
            captureAndStoreImages,
            missionPath,
            missionSpeed,
            missionGimbal,
            missionRepeat,
            action,
            user_id, 
            dronePK,
        )
        # elif missionType == Mission.SEARCH_AND_RESCUE_MISSION:
        #     utils.handleAlgorithmExecution(operation.pk, input, canBeLoadedOnMap, userPK, algorithmName)
        # pass
        return Response(status=status.HTTP_200_OK)


class AlgorithmListView(LoginRequiredMixin, generic.ListView):
    model = Algorithm
    # fields = ('__all__')
    template_name = "aiders/algorithms.html"
    queryset = Algorithm.objects.all()
    success_url = reverse_lazy("home")

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))

        if not self.request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to join the operation.")

        # User has to join the operation in order to view the operation's algorithms
        User.objects.filter(pk=self.request.user.id).update(joined_operation=operation)

        context = super(AlgorithmListView, self).get_context_data(**kwargs)
        context["algorithm_results"] = operation.algorithm_set.all()
        context["operation_name"] = self.kwargs.get("operation_name")
        # Create any data and add it to the context
        return context


@login_required
@csrf_protect
def stop_operation_view(request, operation_name):
    op = Operation.objects.get(operation_name=operation_name)
    if not request.user.has_perm("aiders.manage_operations") or not request.user.has_perm(f"aiders.manage_operation_{op.id}"):
        raise PermissionDenied("You do not have permission to manage operations.")    
    if request.method == "GET":
        opQuery = Operation.objects.filter(operation_name=operation_name)

        if opQuery.exists():
            operation = get_object_or_404(opQuery)

            if operation.active:
                operation.active = False
                operation.save()
                return redirect("manage_operations")
    return redirect("manage_operations")


@login_required
@csrf_protect
def leave_operation_view(request):
    if request.method == "GET":
        get_user_model().objects.filter(pk=request.user.id).update(joined_operation=None)
        return redirect("manage_operations")
        # if (userQuery.exists()):
        #     get_object_or_404(userQuery).update(joined_operation=None)
        #     user.joined_operation = None
        #     user.save()
        # return redirect('home')


@login_required
@csrf_protect
def join_operation_view(request, operation_name):
    op = Operation.objects.get(operation_name=operation_name)
    if not request.user.has_perm(f"aiders.view_operation_{op.id}"):
        raise PermissionDenied("You do not have permission to join the operation.")
    if request.method == "POST":
        opQuery = Operation.objects.filter(operation_name=operation_name)
        if opQuery.exists():
            operation = get_object_or_404(opQuery)
            # if operation.active:
            User.objects.filter(pk=request.user.id).update(joined_operation=operation)
            # get_object_or_404(user_query)
            return redirect("home")
            # else:
            #     raise Http404("Operation Not Found")
        else:
            raise Http404("Operation Not Found")
    return JsonResponse({"success": False})


@csrf_protect
def register_request(request):
    if request.method == "POST":
        form = NewUserForm(request.POST)
        if form.is_valid():
            user = form.save()
            UserPreferences.initalizeUserPreferences(user)
            login(request, user, backend="django.contrib.auth.backends.ModelBackend")
            return redirect("manage_operations")
    else:
        form = NewUserForm()
    return render(request=request, template_name="aiders/register.html", context={"register_form": form})


class DetectionAPIOperations:
    @staticmethod
    def create_detection_session_on_db(user, operation, drone):
        return DetectionSession.objects.create(user=user, operation=operation, drone=drone)

    @staticmethod
    def save_frame_to_db(frame_file, detection_session):
        return DetectionFrame.objects.create(
            frame=frame_file,
            detection_session=detection_session,
        )

    @staticmethod
    def update_detection_status_on_db(drone, detection_status, detection_type_str):
        qs = Detection.objects.filter(drone__drone_name=drone.drone_name).update(
            detection_status=detection_status, detection_type_str=detection_type_str
        )

    @staticmethod
    def update_detection_session_end_time(detection_session):
        end_time = datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ)
        DetectionSession.objects.filter(pk=detection_session.id).update(end_time=end_time, is_active=False)

    @staticmethod
    def update_latest_frame(detection_session, latest_frame_url):
        DetectionSession.objects.filter(pk=detection_session.id).update(latest_frame_url=latest_frame_url)

    @staticmethod
    def save_detected_object_to_db(detection_session, detectedObj, frame):
        DetectedObject.objects.create(
            track_id=detectedObj.trk_id,
            label=detectedObj.label,
            lat=detectedObj.lat,
            lon=detectedObj.lon,
            detection_session=detection_session,
            distance_from_drone=detectedObj.distFromDrone,
            frame=frame,
        )


class LiveStreamAPIOperations(LoginRequiredMixin, generics.RetrieveAPIView):
    # def get(self, request, *args, **kwargs):
    #     operation_name=self.kwargs.get('operation_name')
    #     drone_name = self.kwargs.get('drone_name')

    @staticmethod
    def create_live_stream_session_on_db(drone):
        return LiveStreamSession.objects.create(drone=drone)

    @staticmethod
    def save_raw_frame_to_db(frame_file, drone_name, live_stream_session):
        return RawFrame.objects.create(
            frame=frame_file,
            drone=Drone.objects.get(drone_name=drone_name),
            live_stream_session=live_stream_session,
        )

    @staticmethod
    def update_latest_raw_frame(live_stream_session, latest_frame_url):
        LiveStreamSession.objects.filter(pk=live_stream_session.id).update(latest_frame_url=latest_frame_url)


@api_view(["GET"])
def objects_detected_on_last_frame_api_view(request, operation_name, drone_name):
    if request.method == "GET":
        try:
            active_detection_session = DetectionSession.objects.filter(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name
            )
            active_detection_session = DetectionSession.objects.get(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name
            )
            # Get the last frame object for the active detection session
            latest_frame = DetectionFrame.objects.filter(detection_session=active_detection_session).last()
            # Get the detected objects that appear on the last frame
            detected_objects = DetectedObject.objects.filter(frame=latest_frame)

        except DetectionSession.DoesNotExist:
            return Response({"error": Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE})
        if detected_objects == None:
            return Response({"error": "No objects detected on last frame"})
        serializer = DetectedObjectSerializer(detected_objects, many=True)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


@api_view(["GET"])
def last_detection_frame_api_view(request, operation_name, drone_name):
    if request.method == "GET":
        try:
            active_detection_session = DetectionSession.objects.get(is_active=True, drone__drone_name=drone_name)
        except DetectionSession.DoesNotExist:
            return Response({"latest_frame_url": Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE})
        serializer = DetectionSessionSerializer(active_detection_session)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


@api_view(["GET"])
def last_raw_frame_api_view(request, operation_name, drone_name):
    if request.method == "GET":
        try:
            active_detection_session = LiveStreamSession.objects.get(is_active=True, drone__drone_name=drone_name)
        except LiveStreamSession.DoesNotExist:
            return Response({"latest_frame_url": Constants.NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE})
        serializer = LiveStreamSessionSerializer(active_detection_session)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


# @api_view(["GET"])
# def detection_types_api_view(request, operation_name):
#     if request.method == "GET":
#         from logic.algorithms.object_detection.src.models.label import get_labels_all

#         return Response({"detection_types": list(get_labels_all())})

#     return Response(status=status.HTTP_400_BAD_REQUEST)


@api_view(["GET"])
def live_stream_status_api_view(request, operation_name, drone_name):
    if request.method == "GET":
        liveStreamSession = LiveStreamSession.objects.get(drone__drone_name=drone_name)
        if liveStreamSession.is_active:
            return Response({"is_live_stream_active": True})
        else:
            return Response({"is_live_stream_active": False})
    return Response(status=status.HTTP_400_BAD_REQUEST)


class WeatherStationAPIView(LoginRequiredMixin, generics.RetrieveAPIView):
    queryset = WeatherStation.objects.all()
    serializer_class = WeatherStationSerializer

    def addWeatherStationDataToDB(data, object_name):
        object_name = object_name.replace("~", " ")
        try:
            operation_name = Operation.objects.get(operation_name=object_name)
            WeatherStation.objects.create(
                wind_speed=data.speed,
                wind_direction=data.direction,
                temperature=data.temperature,
                pressure=data.pressure,
                humidity=data.humidity,
                heading=data.heading,
                operation=Operation.objects.get(operation_name=operation_name),
                drone=None,
            )
        except Operation.DoesNotExist:
            operation_name = None
        try:
            drone_name = Drone.objects.get(drone_name=object_name)
            WeatherStation.objects.create(
                wind_speed=data.speed,
                wind_direction=data.direction,
                temperature=data.temperature,
                pressure=data.pressure,
                humidity=data.humidity,
                heading=data.heading,
                operation=None,
                drone=Drone.objects.get(drone_name=drone_name),
            )
        except Drone.DoesNotExist:
            drone_name = None


def system_monitoring_save_to_db(
    cpu_usage,
    cpu_core_usage,
    cpu_temp,
    gpu_usage,
    gpu_memory,
    gpu_temp,
    ram_usage,
    swap_memory_usage,
    temp,
    mb_new_sent,
    mb_new_received,
    mb_new_total,
    disk_read,
    disk_write,
    battery_percentage,
):
    SystemMonitoring.objects.create(
        cpu_usage=cpu_usage,
        cpu_core_usage=cpu_core_usage,
        cpu_temp=cpu_temp,
        gpu_usage=gpu_usage,
        gpu_memory=gpu_memory,
        gpu_temp=gpu_temp,
        ram_usage=ram_usage,
        swap_memory_usage=swap_memory_usage,
        temp=temp,
        upload_speed=mb_new_sent,
        download_speed=mb_new_received,
        total_network=mb_new_total,
        disk_read=disk_read,
        disk_write=disk_write,
        battery_percentage=battery_percentage,
    )


class buildMapSessionsAPIView(LoginRequiredMixin, generic.ListView):
    model = BuildMapSession

    template_name = "aiders/build_map_session.html"
    queryset = BuildMapSession.objects.all()

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))

        if not self.request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to join the operation.")

        context = super(buildMapSessionsAPIView, self).get_context_data(**kwargs)
        context["MapSession_results"] = list(operation.buildmapsession_set.all())
        index = 0
        urlList = []
        list_non_zero_images = list(BuildMapImage.objects.filter().values("session").annotate(n=models.Count("pk")))
        while index < len(context["MapSession_results"]):
            element = context["MapSession_results"][index]
            save = False
            for session_non_zero_images in list_non_zero_images:
                if session_non_zero_images["session"] == context["MapSession_results"][index].id:
                    context["MapSession_results"][index].images = session_non_zero_images["n"]
                    save = True
            if save == False:
                context["MapSession_results"].remove(element)
            else:
                urlList.append(
                    self.request.build_absolute_uri(reverse("build_map_session_share", args=[self.kwargs.get("operation_name"), element.id]))
                )
                index += 1
        context["operation_name"] = self.kwargs.get("operation_name")
        context["urls"] = urlList
        return context


class buildMapSessionsShareAPIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        self.kwargs.get("pk")
        buildMapSessionObject = BuildMapSession.objects.get(pk=self.kwargs.get("pk"))
        fileList = []
        with open("buildMapSession.csv", "w") as csvFile:
            fileWriter = csv.writer(csvFile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
            fileWriter.writerow([f.name for f in BuildMapSession._meta.get_fields()])
            dataList = []
            for key in [f.name for f in BuildMapSession._meta.get_fields()]:
                try:
                    dataList.append(getattr(buildMapSessionObject, key))
                except:
                    dataList.append("")
            fileWriter.writerow(dataList)

        with open("buildMapImages.csv", "w") as csvFile2:
            fileWriter = csv.writer(csvFile2, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
            fileWriter.writerow([f.name for f in BuildMapImage._meta.get_fields()])
            for data in BuildMapImage.objects.filter(session=self.kwargs.get("pk")):
                dataList = []
                for key in [f.name for f in BuildMapImage._meta.get_fields()]:
                    try:
                        if isinstance(getattr(data, key), point.Point):
                            dataList.append(str(getattr(data, key).coords[0]) + " " + str(getattr(data, key).coords[1]))
                        else:
                            dataList.append(getattr(data, key))
                    except:
                        dataList.append("")
                fileWriter.writerow(dataList)

        try:
            if not os.path.exists(default_storage.path("") + "/temp/"):
                os.makedirs(default_storage.path("") + "/temp/")
            else:
                shutil.rmtree(default_storage.path("") + "/temp/")
                os.makedirs(default_storage.path("") + "/temp/")
            shutil.move("buildMapSession.csv", default_storage.path("") + "/temp/buildMapSession.csv")
            shutil.move("buildMapImages.csv", default_storage.path("") + "/temp/buildMapImages.csv")
            os.mkdir(default_storage.path("") + "/temp/" + BuildMapImage.objects.filter(session=self.kwargs.get("pk")).last().path.split("/")[0])
            for data in BuildMapImage.objects.filter(session=self.kwargs.get("pk")):
                shutil.copyfile(default_storage.path(data.path), default_storage.path("") + "/temp/" + data.path)
        except Exception as e:
            pass

        try:
            zip_file = zipfile.ZipFile(default_storage.path("build_map_session_share.zip"), "w")
            for root, dirs, files in os.walk(default_storage.path("temp")):
                for f in files:
                    zip_file.write(os.path.join(root, f), f)
            zip_file.close()
            zip_file = open(default_storage.path("build_map_session_share.zip"), "rb")
            return FileResponse(zip_file)
        except Exception as e:
            return HttpResponse(status=status.HTTP_404_NOT_FOUND)


class waterCollectionActivatedAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        droneName = request.POST.get("drone_id")
        apiResponse=postRequestForOpenWaterSamplingValve(droneName)
        return HttpResponse(apiResponse, status=status.HTTP_200_OK)


# class ballisticActivatedAPIView(LoginRequiredMixin, View):
#     def post(self, request, *args, **kwargs):
#         drone_name = request.POST.get("drone_id")
#         operation_name = kwargs.get("operation_name")
#         if Drone.objects.get(drone_name=drone_name).ballistic_available:
#             try:
#                 # ballistic.publish_message(drone_name, 1)
#                 Ballistic.objects.create(
#                     drone=Drone.objects.get(drone_name=drone_name),
#                     operation=Operation.objects.get(operation_name=operation_name),
#                     user=User.objects.get(pk=request.user.pk),
#                     telemetry=Telemetry.objects.filter(drone=Drone.objects.get(drone_name=drone_name)).last(),
#                 )
#                 logger.info(f"Ballistic activated for drone {drone_name}.")
#                 return HttpResponse("Sending message to drone.", status=status.HTTP_200_OK)
#             except Exception as e:
#                 logger.error(f"Ballistic encounter an error for drone {drone_name}. Error: {e}")
#         return HttpResponse(
#             f"Ballistic encounter an error for drone {drone_name}.",
#             status=status.HTTP_200_OK,
#         )


class BaloraPM25APIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        return JsonResponse(
            list(
                BaloraTelemetry.objects.filter(
                    operation__operation_name=kwargs.get("operation_name"), baloraMaster__is_connected_with_platform=True,
                    time__gte = timezone.now() - timedelta(hours=SENSOR_DATA_DURATION_HOURS) 
                ).values("pm25", "latitude", "longitude")
            ),
            safe=False,
        )
    
class BaloraPM1APIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        return JsonResponse(
            list(
                BaloraTelemetry.objects.filter(
                    operation__operation_name=kwargs.get("operation_name"), baloraMaster__is_connected_with_platform=True,
                    time__gte = timezone.now() - timedelta(hours=SENSOR_DATA_DURATION_HOURS)  
                ).values("pm1", "latitude", "longitude")
            ),
            safe=False,
        )
class BaloraNOxAPIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        return JsonResponse(
            list(
                BaloraTelemetry.objects.filter(
                    operation__operation_name=kwargs.get("operation_name"), baloraMaster__is_connected_with_platform=True,
                    time__gte = timezone.now() - timedelta(hours=SENSOR_DATA_DURATION_HOURS) 
                ).values("nox", "latitude", "longitude")
            ),
            safe=False,
        )
    
class BaloraVOCAPIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        return JsonResponse(
            list(
                BaloraTelemetry.objects.filter(
                    operation__operation_name=kwargs.get("operation_name"), baloraMaster__is_connected_with_platform=True,
                    time__gte = timezone.now() - timedelta(hours=SENSOR_DATA_DURATION_HOURS)  
                ).values("voc", "latitude", "longitude")
            ),
            safe=False,
        )
    
# class BaloraPM25AverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("pm25", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]
#             batch_avg = sum(item["pm25"] for item in batch) / len(batch)  # Calculate average for the batch

#             # Update each item's pm25 to only contain the average
#             for item in batch:
#                 item["pm25"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)
    
# class BaloraPM1AverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("pm1", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]
#             batch_avg = sum(item["pm1"] for item in batch) / len(batch)  # Calculate average for the batch

#             # Update each item's pm1 to only contain the average
#             for item in batch:
#                 item["pm1"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)
    
# class BaloraNOxAverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("nox", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]
            
#             # Replace None with 0 in the 'nox' field
#             for item in batch:
#                 item["nox"] = item["nox"] or 0  # Use 0 if 'nox' is None
            
#             # Calculate the average for the batch
#             batch_avg = sum(item["nox"] for item in batch) / len(batch)

#             # Update each item's nox to only contain the average
#             for item in batch:
#                 item["nox"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)

# class BaloraVOCAverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("voc", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]

#             # Replace None with 0 for the "voc" values
#             valid_values = [item["voc"] if item["voc"] is not None else 0 for item in batch]

#             # Calculate average for the batch
#             batch_avg = sum(valid_values) / len(batch)

#             # Update each item's voc to only contain the average
#             for item in batch:
#                 item["voc"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)


# class BaloraTempAverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("temp", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]
#             batch_avg = sum(item["temp"] for item in batch) / len(batch)  # Calculate average for the batch

#             # Update each item's temp to only contain the average
#             for item in batch:
#                 item["temp"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)
    
# class BaloraHumidityAverageAPIView(LoginRequiredMixin, View):
#     def get(self, request, *args, **kwargs):
#         # Fetch all telemetry records for the specified operation
#         raw_values = list(
#             BaloraTelemetry.objects.filter(
#                 operation__operation_name=kwargs.get("operation_name"),
#                 baloraMaster__is_connected_with_platform=True
#             )
#             .values("humidity", "latitude", "longitude")
#             .order_by("id")  # Ensure records are ordered correctly
#         )

#         # Process data in batches of 10
#         batch_size = 10
#         for i in range(0, len(raw_values), batch_size):
#             batch = raw_values[i:i + batch_size]
#             batch_avg = sum(item["humidity"] for item in batch) / len(batch)  # Calculate average for the batch

#             # Update each item's humidity to only contain the average
#             for item in batch:
#                 item["humidity"] = batch_avg

#         # Return the response
#         return JsonResponse(raw_values, safe=False)


class rangeFinderAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        drone_name = request.POST.get("drone_id")
        start_stop = request.POST.get("start_stop")
        operation_name = kwargs.get("operation_name")
        if Drone.objects.get(drone_name=drone_name).camera_model:
            try:
                range_detection.buildMapPublisherSingleMessage(drone_name, start_stop)
                logger.info(f"Range Finder activated for drone {drone_name}.")
                return HttpResponse("Sending message to drone.", status=status.HTTP_200_OK)
            except Exception as e:
                logger.error(f"Range Finder encounter an error for drone {drone_name}. Error: {e}")
        return HttpResponse("Range Finder an error for drone {}.".format(drone_name), status=status.HTTP_200_OK)


class frontEndUserInputAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        element = request.POST.get("elementId")
        value = request.POST.get("active")
        operation_name = kwargs.get("operation_name")
        try:
            FrontEndUserInput.objects.create(
                operation=Operation.objects.get(operation_name=operation_name), element_name=element, value=value, user=request.user
            )
            return HttpResponse("Action Saved Successful.", status=status.HTTP_200_OK)
        except Exception as e:
            logger.error(e)
        return HttpResponse("Action Not Saved Successful.", status=status.HTTP_200_OK)


class SystemMonitoringView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        if request.user.is_superuser:
            return render(request, "aiders/monitoring_platform.html", {})
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)


class ControlDevicesMonitoringView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        if request.user.is_superuser:
            drones = Drone.objects.all()
            return render(
                request,
                "aiders/monitoring_control_devices.html",
                {
                    "drones": drones,
                    "available_drones": [
                        Drone.objects.get(id=drones_temp["drone"])
                        for drones_temp in list(ControlDevice.objects.filter().values("drone").annotate(n=models.Count("pk")))
                    ],
                },
            )
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)


class ControlDeviceMonitoringView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        if request.user.is_superuser:
            drone_name = kwargs.get("control_device")
            available_drones = list(ControlDevice.objects.filter().values("drone").annotate(n=models.Count("pk")))
            temp = [drones_temp["drone"] for drones_temp in available_drones]
            available_drones = temp
            if Drone.objects.get(drone_name=drone_name).id not in available_drones:
                return HttpResponse(status=status.HTTP_404_NOT_FOUND)
            return render(request, "aiders/monitoring_control_device.html", {"drone_name": drone_name})
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)
    


def getLidarSessionOfPoints(request, *args, **kwargs):
    if request.method == 'GET':
        operation = Operation.objects.get(operation_name=kwargs["operation_name"])
        inactiveSessionsWithLidarPointsCount = LidarPointSession.getNoActiveSessionAndNumberOfLidarPointsByOperationId(operation.id)
        result = []
        for session in inactiveSessionsWithLidarPointsCount:
            result.append({
                'sessionId': session.id,
                'droneName': session.drone.drone_name,
                'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                'endTime': session.end_time.strftime('%Y-%m-%d %H:%M:%S'),
                'count': session.lidar_point_count
            })
        return HttpResponse(json.dumps(result))

def getLidarPointsBySessionId(request, *args, **kwargs):
    if request.method == 'POST':
        data = json.loads(request.body)
        lidarPointSessionId = data.get("lidar_point_session_id")
        latestPointId = data.get("latest_point_id")
        numberOfPoints = data.get("number_of_points")
        lidarList =LidarPoint.getLidarPointsBySessionIdLatestIdAndLimit(lidarPointSessionId, latestPointId, numberOfPoints)
        if(latestPointId == 0):
            return JsonResponse({"data": lidarList, "coordinates":LidarPoint.getOriginCoordinatesLonLatBySessionId(lidarPointSessionId)})
        return JsonResponse({"data": lidarList})

def getLidarSessionOfPointsThatAreNotProcessed(request, *args, **kwargs):
    if request.method == 'GET':
        operation = Operation.objects.get(operation_name=kwargs["operation_name"])
        inactiveSessionsWithLidarPointsCount = LidarPointSession.getUnprocessedSessionAndNumberOfLidarPointsByOperationId(operation.id)
        result = []
        for session in inactiveSessionsWithLidarPointsCount:
            result.append({
                'sessionId': session.id,
                'droneName': session.drone.drone_name,
                'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                'endTime': session.end_time.strftime('%Y-%m-%d %H:%M:%S'),
                'count': session.lidar_point_count
            })
        return HttpResponse(json.dumps(result))

def processLidarSessionPointsBySessionId(request, *args, **kwargs):
    from .httpRequests import postRequestForLidarProcess
    if request.method == 'POST':
        data = json.loads(request.body)
        result=postRequestForLidarProcess(data.get("sessionId"))
        print(result, flush=True)
        return JsonResponse(result)


def getLidarSessionWithProcessMesh(request, *args, **kwargs):
    if request.method == 'GET':
        operation = Operation.objects.get(operation_name=kwargs["operation_name"])
        inactiveSessionsWithLidarPointsCount = LidarPointSession.getProcessedSessionAndNumberOfLidarPointsByOperationId(operation.id)
        result = []
        for session in inactiveSessionsWithLidarPointsCount:
            result.append({
                'sessionId': session.id,
                'droneName': session.drone.drone_name,
                'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                'endTime': session.end_time.strftime('%Y-%m-%d %H:%M:%S'),
                'count': session.lidar_point_count
            })
        return HttpResponse(json.dumps(result))
 
def getLidarMeshDataNeededForVisualizationBySessionId(request, *args, **kwargs):
    if request.method == 'POST':
        data = json.loads(request.body)
        lidarPointSessionId = data.get("sessionId")
        lidarMeshData = LidarPoint.getLidarMeshDataNeedForVisualizationBySessionId(lidarPointSessionId)
        return JsonResponse({"data": lidarMeshData})

class FlyingReportAPIView(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        AvailableDroneList = list(Drone.objects.filter(operation__operation_name=operation_name).values())
        listDrones = [
            {
                "drone_name": drone["drone_name"],
                "latitude": Telemetry.objects.filter(drone__drone_name=drone["drone_name"]).last().lat,
                "longitude": Telemetry.objects.filter(drone__drone_name=drone["drone_name"]).last().lon,
            }
            for drone in AvailableDroneList
        ]
        return render(
            request,
            "aiders/flying_report.html",
            {"list_of_drones": listDrones, "available_drones": json.dumps(listDrones), "operation_name": operation_name, "form": FlyingReportForm()},
        )

    def post(self, request, *args, **kwargs):
        user = request.user.username
        if request.POST.get("form_selection") != "custom":
            drone = request.POST.get("form_selection")
        else:
            drone = "Unknown"
        operation_name = self.kwargs.get("operation_name")
        form = FlyingReportForm(request.POST)
        if form.is_valid():
            return self.valid_form(request, user, drone, operation_name)
        operation_name = self.kwargs.get("operation_name")
        AvailableDroneList = list(Drone.objects.filter(operation__operation_name=operation_name).values())
        listDrones = [
            {
                "drone_name": drone["drone_name"],
                "latitude": Telemetry.objects.filter(drone__drone_name=drone["drone_name"]).last().lat,
                "longitude": Telemetry.objects.filter(drone__drone_name=drone["drone_name"]).last().lon,
            }
            for drone in AvailableDroneList
        ]
        return render(
            request,
            "aiders/flying_report.html",
            {"list_of_drones": listDrones, "available_drones": json.dumps(listDrones), "operation_name": operation_name, "form": form},
        )

    def valid_form(self, request, user, drone, operation_name):
        latitude = request.POST.get("latitude")
        longitude = request.POST.get("longitude")
        altitude = request.POST.get("altitude")
        radius = request.POST.get("radius")
        buffer_altitude = request.POST.get("buffer_altitude")
        buffer_radius = request.POST.get("buffer_radius")
        start_date = request.POST.get("start_date_time")
        end_date = request.POST.get("end_date_time")
        start_date = datetime.datetime.strptime(start_date, "%Y-%m-%dT%H:%M")
        end_date = datetime.datetime.strptime(end_date, "%Y-%m-%dT%H:%M")
        path = f"daily_fly_notams/notams{len(FlyingReport.objects.all())}.pdf"
        flying_report.main(
            user, drone, operation_name, latitude, longitude, altitude, radius, buffer_altitude, buffer_radius, start_date, end_date, path
        )
        try:
            drone = Drone.objects.get(drone_name=drone)
        except Drone.DoesNotExist:
            drone = None

        FlyingReport.objects.create(
            user=request.user,
            drone=drone,
            operation=Operation.objects.get(operation_name=operation_name),
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            radius=radius,
            buffer_altitude=buffer_altitude,
            buffer_radius=buffer_radius,
            start_date_time=start_date,
            end_date_time=end_date,
            file_path=path,
        )
        response = open(default_storage.path(path), "rb")
        return FileResponse(response)


class FlyingReportTableAPIView(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        fly_reports = FlyingReport.objects.filter(operation=Operation.objects.get(operation_name=operation_name))
        return render(request, "aiders/flying_reports.html", {"flying_reports": fly_reports, "operation_name": operation_name})



#########################################
# OPERATION REPORT
#########################################

class OperationReportAPIView(LoginRequiredMixin, View):
    """
    Generate comprehensive operation reports with flight paths and statistics
    """
    def get(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        operation = get_object_or_404(Operation, operation_name=operation_name)
        
        # Check permissions
        if not request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to view this operation.")
        
        form = OperationReportForm(initial=self.default_date_range(operation))
        return render(
            request,
            "aiders/operation_report.html",
            {
                "form": form,
                "operation_name": operation_name,
                "operation": operation
            }
        )

    @staticmethod
    def default_date_range(operation):
        """Pre-fill the date pickers with the operation's own time range"""
        return {
            "start_date": timezone.localtime(operation.created_at),
            "end_date": timezone.localtime(operation.ended_at) if operation.ended_at else timezone.localtime(),
        }

    @method_decorator(with_timeout(600))  # 10 minutes timeout
    def post(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        operation = get_object_or_404(Operation, operation_name=operation_name)
        
        # Check permissions
        if not request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to view this operation.")
        
        form = OperationReportForm(request.POST)
        if form.is_valid():
            return self.valid_form(request, operation, operation_name, form)
        
        return render(
            request,
            "aiders/operation_report.html",
            {
                "form": form,
                "operation_name": operation_name,
                "operation": operation
            }
        )

    def valid_form(self, request, operation, operation_name, form):
        try:
            # Get form data
            start_date = form.cleaned_data.get('start_date')
            end_date = form.cleaned_data.get('end_date')
            include_flight_paths = form.cleaned_data.get('include_flight_paths')
            include_statistics = form.cleaned_data.get('include_statistics')
            enhanced_maps = form.cleaned_data.get('enhanced_maps', True)
            
            # Create report generator
            generator = OperationReportGenerator()
            
            # Generate the report
            pdf_buffer = generator.generate_operation_report(
                operation=operation,
                start_date=start_date,
                end_date=end_date,
                include_flight_paths=include_flight_paths,
                include_statistics=include_statistics,
                enhanced_maps=enhanced_maps
            )
            
            # Create the response
            response = HttpResponse(pdf_buffer.getvalue(), content_type='application/pdf')
            response['Content-Disposition'] = f'attachment; filename="operation_report_{operation_name}_{timezone.now().strftime("%Y%m%d_%H%M%S")}.pdf"'
            
            return response
            
        except Exception as e:
            logger.error(f"Error generating operation report: {str(e)}")
            messages.error(request, f"Error generating report: {str(e)}")
            
            return render(
                request,
                "aiders/operation_report.html",
                {
                    "form": form,
                    "operation_name": operation_name,
                    "operation": operation
                }
            )



class ExternalAPI(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        if all(thread.name != "external_api_" + self.kwargs.get("operation_name") for thread in threading.enumerate()):
            t = threading.Thread(
                name="external_api_" + self.kwargs.get("operation_name"), target=patho_request.main(self.kwargs.get("operation_name"))
            )
            t.start()
        return HttpResponse(status=status.HTTP_200_OK)


def settings_view(request):
    if request.user.is_authenticated:
        if request.method == "GET":
            use_online_map = UserPreferences.objects.get(user=request.user).use_online_map
            return render(request, "aiders/settings.html", {"use_online_map": use_online_map})
        elif request.method == "POST":
            selectedVal = request.POST.get("map_mode_dropdown")
            use_online_map = selectedVal == Constants.ONLINE_MAP_MODE
        UserPreferences.objects.filter(user=request.user).update(use_online_map=use_online_map)
        return render(request, "aiders/settings.html", {"use_online_map": use_online_map})



def safeDronesStart(request):
    calculations_safe_drones.start()
    return HttpResponse(status=status.HTTP_200_OK)

def safeDronesStop(request):
    print(datetime.datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES STOP REQUESTED", flush=True)
    calculations_safe_drones.stop()
    return HttpResponse(status=status.HTTP_200_OK)


# a page that lists all live stream sessions for a specific operation
class LiveStreamSessionListView(LoginRequiredMixin, generic.ListView):
    model = LiveStreamSession
    template_name = "aiders/live_stream_sessions.html"
    context_object_name = "live_stream_sessions"
    success_url = reverse_lazy("home")

    def get_queryset(self):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        if not self.request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to view the operation.")
        
        # Get all drones for this operation, then get all their live stream sessions
        drones = operation.drone_set.all()
        return LiveStreamSession.objects.filter(drone__in=drones).order_by('-start_time')

    def get_context_data(self, **kwargs):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        # User has to join the operation in order to view the operation's live stream sessions
        User.objects.filter(pk=self.request.user.id).update(joined_operation=operation)

        context = super(LiveStreamSessionListView, self).get_context_data(**kwargs)
        
        # Add duration calculation for each session
        sessions_with_duration = []
        for session in context['live_stream_sessions']:
            session_data = {
                'session': session,
                'duration_str': None
            }
            
            if session.end_time:
                duration = session.end_time - session.start_time
                total_seconds = int(duration.total_seconds())
                
                # Calculate hours, minutes, seconds
                hours, remainder = divmod(total_seconds, 3600)
                minutes, seconds = divmod(remainder, 60)
                
                # Format duration string
                if hours > 0:
                    if minutes > 0:
                        session_data['duration_str'] = f"{hours}h {minutes}m"
                    else:
                        session_data['duration_str'] = f"{hours}h"
                elif minutes > 0:
                    if seconds > 0:
                        session_data['duration_str'] = f"{minutes}m {seconds}s"
                    else:
                        session_data['duration_str'] = f"{minutes}m"
                else:
                    session_data['duration_str'] = f"{seconds}s"
            
            sessions_with_duration.append(session_data)
        
        context['sessions_with_duration'] = sessions_with_duration
        context["operation_name"] = self.kwargs.get("operation_name")
        return context
    


# a page that lists all detection sessions for a specific operation
class DetectionSessionListView(LoginRequiredMixin, generic.ListView):
    model = DetectionSession
    template_name = "aiders/detection_sessions.html"
    context_object_name = "detection_sessions"
    success_url = reverse_lazy("home")

    def get_queryset(self):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        if not self.request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to view the operation.")
        
        # Get all drones for this operation, then get all their live stream sessions
        drones = operation.drone_set.all()
        return DetectionSession.objects.filter(drone__in=drones).order_by('-start_time')

    def get_context_data(self, **kwargs):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        # User has to join the operation in order to view the operation's live stream sessions
        User.objects.filter(pk=self.request.user.id).update(joined_operation=operation)

        context = super(DetectionSessionListView, self).get_context_data(**kwargs)
        
        # Add duration calculation for each session
        sessions_with_duration = []
        for session in context['detection_sessions']:
            session_data = {
                'session': session,
                'duration_str': None
            }
            
            if session.end_time:
                duration = session.end_time - session.start_time
                total_seconds = int(duration.total_seconds())
                
                # Calculate hours, minutes, seconds
                hours, remainder = divmod(total_seconds, 3600)
                minutes, seconds = divmod(remainder, 60)
                
                # Format duration string
                if hours > 0:
                    if minutes > 0:
                        session_data['duration_str'] = f"{hours}h {minutes}m"
                    else:
                        session_data['duration_str'] = f"{hours}h"
                elif minutes > 0:
                    if seconds > 0:
                        session_data['duration_str'] = f"{minutes}m {seconds}s"
                    else:
                        session_data['duration_str'] = f"{minutes}m"
                else:
                    session_data['duration_str'] = f"{seconds}s"
            
            sessions_with_duration.append(session_data)
        
        context['sessions_with_duration'] = sessions_with_duration
        context["operation_name"] = self.kwargs.get("operation_name")
        return context


class DeviceSessionListView(LoginRequiredMixin, generic.ListView):
    model = DeviceSession
    template_name = "aiders/device_sessions.html"
    context_object_name = "device_sessions"
    success_url = reverse_lazy("home")

    def get_queryset(self):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))

        if not self.request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            raise PermissionDenied("You do not have permission to view the operation.")

        devices = Device.objects.filter(operation_id=operation.id)
        sessions = DeviceSession.objects.filter(device__in=devices).order_by('-start_time')
        # Only include sessions that have at least one image
        return [s for s in sessions if DeviceImage.objects.filter(session_id=s.id).exists()]

    def get_context_data(self, **kwargs):
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))

        User.objects.filter(pk=self.request.user.id).update(joined_operation=operation)

        context = super(DeviceSessionListView, self).get_context_data(**kwargs)

        sessions_with_details = []
        for session in context['device_sessions']:
            frame_count = DeviceImage.objects.filter(session_id=session.id).count()
            duration_str = None
            if session.end_time:
                delta = session.end_time - session.start_time
                total_seconds = int(delta.total_seconds())
                hours, remainder = divmod(total_seconds, 3600)
                minutes, seconds = divmod(remainder, 60)
                if hours > 0:
                    duration_str = f"{hours}h {minutes}m" if minutes > 0 else f"{hours}h"
                elif minutes > 0:
                    duration_str = f"{minutes}m {seconds}s" if seconds > 0 else f"{minutes}m"
                else:
                    duration_str = f"{seconds}s"
            sessions_with_details.append({
                'session': session,
                'duration_str': duration_str,
                'frame_count': frame_count,
            })

        context['sessions_with_details'] = sessions_with_details
        context["operation_name"] = self.kwargs.get("operation_name")
        return context


##############################################################
######################### MAVLINK ############################
##############################################################


class MavlinkForm(forms.Form):
    name = forms.CharField(max_length=255, help_text="A unique identifier for the UAV", widget=forms.TextInput(attrs={'size': '14', 'class': 'form-control'}))
    model = forms.CharField(max_length=255, widget=forms.TextInput(attrs={'size': '14', 'class': 'form-control'}))
    configuration = forms.ChoiceField(choices=[("MULTICOPTER", "MULTICOPTER"), ("VTOL", "VTOL"), ("FIXED-WING", "FIXED-WING")], widget=forms.Select(attrs={'class': 'form-control native-select'}))
    ip = forms.CharField(max_length=255, required=False, label="IP Address", widget=forms.TextInput(attrs={'size': '11', 'class': 'form-control'}))
    port = forms.CharField(max_length=255, help_text="Default MAVLink port is 14550", widget=forms.TextInput(attrs={'size': '5', 'class': 'form-control'}))
    live_stream_url = forms.CharField(max_length=255, help_text="RTMP or RTSP", widget=forms.TextInput(attrs={'size': '30', 'class': 'form-control'}))
    # dropdown list for connection type
    connection_type = forms.ChoiceField(choices=[("MAVPROXY", "MAVPROXY"), ("WEBSOCKETS", "WEBSOCKETS (custom onboard server)")], widget=forms.Select(attrs={'class': 'form-control native-select'}))
    # operation = forms.CharField(widget=forms.Textarea)


def mavlinkAddFormView(request):
    if request.method == 'POST':
        form = MavlinkForm(request.POST)
        if form.is_valid():
            # Retrieve data from the form
            name = form.cleaned_data['name']
            model = form.cleaned_data['model']
            type = "MAVLINK"
            connection_type = form.cleaned_data['connection_type']
            configuration = form.cleaned_data['configuration']
            ip = form.cleaned_data['ip']
            port = form.cleaned_data['port']
            live_stream_url = form.cleaned_data['live_stream_url']
            drone = Drone(drone_name=name, model=model, type=type, connection_type=connection_type, configuration=configuration, ip=ip, port=port, live_stream_url=live_stream_url, camera_model="no_cam", is_connected_with_platform=False, time=datetime.datetime.now())
            drone.save()

            # retrieve the latest active operation and add the drone to it
            operation = Operation.objects.filter(active=True).last() 
            operation.drones_to_operate.add(drone)
            drone.operation = operation
            drone.save()

            return redirect('drones_list')  
    else:
        form = MavlinkForm()

    return render(request, 'aiders/mavlink-add.html', {'form': form})


def mavlinkEditFormView(request, pk):
    drone = Drone.objects.get(id=pk)
    if request.method == 'POST':
        form = MavlinkForm(request.POST)
        if form.is_valid():
            # Retrieve data from the form
            drone.drone_name = form.cleaned_data['name']
            drone.model = form.cleaned_data['model']
            drone.ip = form.cleaned_data['ip']
            drone.port = form.cleaned_data['port']
            drone.connection_type = form.cleaned_data['connection_type']
            drone.configuration = form.cleaned_data['configuration']
            drone.live_stream_url = form.cleaned_data['live_stream_url']
            drone.save()

            return redirect('drones_list')
    else:
        # populate the form with the existing data
        form = MavlinkForm(
            initial={
                'name': drone.drone_name,
                'model': drone.model,
                'ip': drone.ip,
                'port': drone.port,
                'live_stream_url': drone.live_stream_url,
                'connection_type': drone.connection_type,
                'configuration': drone.configuration,
            })

    return render(request,
    'aiders/mavlink-edit.html',
    {'form': form, 'drone': drone})


def mavlinkManageView(request, pk):
    drone = Drone.objects.get(id=pk)
    return render(request, 'aiders/mavlink-manage.html', {'drone': drone})


def mavlinkCheckConnection(request, pk):
    if request.method == 'POST':
        drone = Drone.objects.get(id=pk)
        return JsonResponse({'connected': drone.is_connected_with_platform}, status=200)
    return JsonResponse({'error': "not supported"}, status=500)


def mavlinkGetLogs(request, operation_id, last_log_id=0):
    if int(last_log_id) == 0:
        logs = MavlinkLog.objects.select_related('drone').filter(operation_id=operation_id).order_by('-id')[:20]
    else:
        logs = MavlinkLog.objects.select_related('drone').filter(id__gt=last_log_id, operation_id=operation_id)

    logs_list = list(logs.values('id', 'message', 'time', 'drone__drone_name'))
    if int(last_log_id) == 0:
        logs_list.reverse()
    return JsonResponse(logs_list, safe=False)


#####################
# MAVLINK API CALLS #
#####################


def mavlinkConnect(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "id": data.get('id'),
            "name": data.get('name'),
            "ip": data.get('ip'),
            "port": data.get('port'),
            "model": data.get('model'),
            "operationId": data.get('operationId'),
            "protocol": data.get('protocol'),
            "library": data.get('library'),
        }
        # print("Connecting to UAV using", data.get('library'), flush=True)

        result=postRequestForMavlink("connectToUav", payload)
        return HttpResponse(result)
    
def mavlinkDisconnect(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('name'),
        }
        result=postRequestForMavlink("disconnectFromUav", payload)
        return HttpResponse(result)
    
def mavlinkTakeoff(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
            "alt": data.get('altitude'),
        }
        result=postRequestForMavlink("takeoff", payload)
        return HttpResponse(result)

def mavlinkLand(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
        }
        result=postRequestForMavlink("land", payload)
        return HttpResponse(result)
    
def mavlinkReturnHome(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
        }        
        result=postRequestForMavlink("returnHome", payload)
        return HttpResponse(result)    

def mavlinkTransition(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
            "mode": data.get('mode'),
        }        
        result=postRequestForMavlink("transition", payload)
        return HttpResponse(result)    

def mavlinkSetSpeed(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
            "speed": data.get('speed'),
        }        
        result=postRequestForMavlink("setSpeed", payload)
        return HttpResponse(result)    

def mavlinkArm(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
        }        
        result=postRequestForMavlink("arm", payload)
        return HttpResponse(result)  
    
def mavlinkDisarm(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
        }        
        result=postRequestForMavlink("disarm", payload)
        return HttpResponse(result)  

def mavlinkKill(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('droneName'),
        }        
        result=postRequestForMavlink("kill", payload)
        return HttpResponse(result)



##################################
###### OPERATION COVERAGE ########
##################################



def operation_coverage_points(request, operation_name):
    operation = Operation.objects.filter(operation_name=operation_name).last()

    print(request.body, flush=True)
    requestData = json.loads(request.body)

    # get from and to datetimes
    fromDatetimeStr = f"{requestData['fromDate']} {requestData['fromTime']}"
    fromDatetimeObj = datetime.datetime.strptime(fromDatetimeStr, '%Y-%m-%d %H:%M')    
    toDatetimeStr = f"{requestData['toDate']} {requestData['toTime']}"
    toDatetimeObj = datetime.datetime.strptime(toDatetimeStr, '%Y-%m-%d %H:%M')    

    dronePolygons = []
    droneData = []
    devicePolygons = []
    deviceData = []
    baloraPolygons = []
    baloraData = []
    droneAllTelemetryPoints = []
    deviceAllTelemetryPoints = []
    baloraAllTelemetryPoints = []

    # drones
    if(requestData['getDrones'] == True):
        if(requestData['getPoints'] == True):
            droneAllTelemetryPoints = list(Telemetry.objects.filter(operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('lat', 'lon'))
        
        drones = Drone.objects.all()
        for drone in drones:
                
            try:
                # Get telemetry with FOV coordinates
                droneTelemetry = list(Telemetry.objects.filter(
                    drone_id=drone.id, 
                    # operation_id=operation.id, # TODO: uncomment when correct operation_id is added to Telemetry
                    time__gte=fromDatetimeObj, 
                    time__lt=toDatetimeObj,
                    fov_coordinates__isnull=False
                )[0:100000].values('lat', 'lon', 'secondsOn', 'time', 'fov_coordinates'))
                
                print(f"Drone {drone.drone_name}: Found {len(droneTelemetry)} telemetry records with FOV data", flush=True)
                
                previousSecondsOn = 0
                currentSecondsOn = 0
                # break down drone's telemetry into sessions based on the secondsOn field
                droneTelemetrySessions = []
                sessionTelemetry = []
                for t in droneTelemetry:
                    if(t['lat'] != 0 and t['lon'] != 0 and t['fov_coordinates']):
                        currentSecondsOn = t['secondsOn']
                        if abs(currentSecondsOn - previousSecondsOn) > 29:
                            if(len(sessionTelemetry)) > 0:
                                droneTelemetrySessions.append(sessionTelemetry)
                            sessionTelemetry = []
                        sessionTelemetry.append(t)
                        previousSecondsOn = currentSecondsOn

                if(len(sessionTelemetry)) > 0:
                    droneTelemetrySessions.append(sessionTelemetry) # append the last session

                # Create coverage polygons directly from telemetry flight paths with FOV-based width
                all_telemetry_with_fov = []
                
                for session in droneTelemetrySessions:
                    if len(session) == 0:
                        continue
                    
                    # Get telemetry points with FOV data
                    session_telemetry = []
                    for telemetry in session:
                        try:
                            # Parse FOV coordinates to calculate coverage width
                            fov_coords = json.loads(telemetry['fov_coordinates'])
                            if fov_coords and len(fov_coords) >= 3:
                                # Calculate FOV width (distance between furthest points)
                                fov_points = [[coord[1], coord[0]] for coord in fov_coords if len(coord) >= 2]  # [lon, lat]
                                
                                if len(fov_points) >= 2:
                                    # Calculate maximum distance between FOV points as coverage width
                                    max_dist = 0
                                    for i in range(len(fov_points)):
                                        for j in range(i + 1, len(fov_points)):
                                            dist = ((fov_points[i][0] - fov_points[j][0])**2 + (fov_points[i][1] - fov_points[j][1])**2)**0.5
                                            max_dist = max(max_dist, dist)
                                    
                                    # Store telemetry point with calculated width
                                    session_telemetry.append({
                                        'lon': telemetry['lon'],
                                        'lat': telemetry['lat'],
                                        'fov_width': max_dist / 2,  # Half width for each side
                                        'time': telemetry['time']
                                    })
                        except (json.JSONDecodeError, TypeError, IndexError, KeyError) as e:
                            print(f"Error processing telemetry for drone {drone.drone_name}: {e}", flush=True)
                    
                    if len(session_telemetry) > 1:
                        all_telemetry_with_fov.extend(session_telemetry)
                
                # Create flight path corridors from telemetry
                if len(all_telemetry_with_fov) >= 2:
                    try:
                        def create_flight_path_corridor(telemetry_points, sample_distance=0.0001):
                            """Create a corridor polygon following the flight path with FOV-based width"""
                            if len(telemetry_points) < 2:
                                return []
                            
                            # Sample telemetry points to avoid too dense coverage
                            sampled_points = []
                            last_point = None
                            
                            for point in telemetry_points:
                                if last_point is None:
                                    sampled_points.append(point)
                                    last_point = point
                                else:
                                    # Calculate distance from last sampled point
                                    dist = ((point['lon'] - last_point['lon'])**2 + (point['lat'] - last_point['lat'])**2)**0.5
                                    if dist >= sample_distance:  # Sample every ~11 meters
                                        sampled_points.append(point)
                                        last_point = point
                            
                            # Ensure we have the last point
                            if sampled_points[-1] != telemetry_points[-1]:
                                sampled_points.append(telemetry_points[-1])
                            
                            if len(sampled_points) < 2:
                                return []
                            
                            print(f"Creating corridor from {len(sampled_points)} telemetry points", flush=True)
                            
                            # Create left and right boundaries of the corridor
                            left_boundary = []
                            right_boundary = []
                            
                            for i, point in enumerate(sampled_points):
                                # Calculate direction vector for this segment
                                if i == 0:
                                    # First point: use direction to next point
                                    next_point = sampled_points[i + 1]
                                    direction = [next_point['lon'] - point['lon'], next_point['lat'] - point['lat']]
                                elif i == len(sampled_points) - 1:
                                    # Last point: use direction from previous point
                                    prev_point = sampled_points[i - 1]
                                    direction = [point['lon'] - prev_point['lon'], point['lat'] - prev_point['lat']]
                                else:
                                    # Middle point: average direction from previous and to next
                                    prev_point = sampled_points[i - 1]
                                    next_point = sampled_points[i + 1]
                                    dir1 = [point['lon'] - prev_point['lon'], point['lat'] - prev_point['lat']]
                                    dir2 = [next_point['lon'] - point['lon'], next_point['lat'] - point['lat']]
                                    direction = [(dir1[0] + dir2[0]) / 2, (dir1[1] + dir2[1]) / 2]
                                
                                # Normalize direction vector
                                length = (direction[0]**2 + direction[1]**2)**0.5
                                if length > 0:
                                    direction = [direction[0] / length, direction[1] / length]
                                else:
                                    direction = [1, 0]  # Default direction
                                
                                # Calculate perpendicular vector (for corridor width)
                                perp_vector = [-direction[1], direction[0]]
                                
                                # Use FOV width for corridor width
                                width = point['fov_width']
                                
                                # Create left and right points
                                left_point = [
                                    point['lon'] + perp_vector[0] * width,
                                    point['lat'] + perp_vector[1] * width
                                ]
                                right_point = [
                                    point['lon'] - perp_vector[0] * width,
                                    point['lat'] - perp_vector[1] * width
                                ]
                                
                                left_boundary.append(left_point)
                                right_boundary.append(right_point)
                            
                            # Create corridor polygon: left boundary + reversed right boundary
                            corridor_points = left_boundary + right_boundary[::-1]
                            
                            # Close the polygon
                            if len(corridor_points) > 0 and corridor_points[0] != corridor_points[-1]:
                                corridor_points.append(corridor_points[0])
                            
                            return corridor_points
                        
                        # Group telemetry points by session/proximity for separate corridors
                        def group_telemetry_by_proximity(telemetry_points, max_gap=0.002):
                            """Group telemetry points into continuous flight segments"""
                            if not telemetry_points:
                                return []
                            
                            groups = []
                            current_group = [telemetry_points[0]]
                            
                            for i in range(1, len(telemetry_points)):
                                current_point = telemetry_points[i]
                                last_point = current_group[-1]
                                
                                # Calculate distance from last point in current group
                                dist = ((current_point['lon'] - last_point['lon'])**2 + (current_point['lat'] - last_point['lat'])**2)**0.5
                                
                                if dist <= max_gap:  # Within ~222 meters
                                    current_group.append(current_point)
                                else:
                                    # Start new group
                                    if len(current_group) >= 2:
                                        groups.append(current_group)
                                    current_group = [current_point]
                            
                            # Add the last group
                            if len(current_group) >= 2:
                                groups.append(current_group)
                            
                            return groups
                        
                        # Sort telemetry by time to maintain flight order
                        all_telemetry_with_fov.sort(key=lambda x: x['time'])
                        
                        # Group into flight segments
                        telemetry_groups = group_telemetry_by_proximity(all_telemetry_with_fov, max_gap=0.002)
                        
                        print(f"Created {len(telemetry_groups)} flight path segments for {drone.drone_name}", flush=True)
                        
                        # Create corridor polygon for each flight segment
                        for group_idx, telemetry_group in enumerate(telemetry_groups):
                            if len(telemetry_group) >= 2:
                                corridor_coords = create_flight_path_corridor(telemetry_group)
                                
                                if len(corridor_coords) >= 4:  # Valid polygon
                                    coverage_polygon = json.dumps(corridor_coords)
                                    dronePolygons.append(coverage_polygon)
                                    segment_name = f"{drone.drone_name} Path {group_idx + 1}" if len(telemetry_groups) > 1 else f"{drone.drone_name} Coverage"
                                    droneData.append([segment_name, telemetry_group[0]['time']])
                                    
                                    avg_width = sum(p['fov_width'] for p in telemetry_group) / len(telemetry_group)
                                    print(f"Created flight corridor {group_idx + 1} for {drone.drone_name}: {len(telemetry_group)} points, avg width {avg_width:.6f}°", flush=True)
                        
                        if len(coverage_polygons) > 0:
                            for poly_idx, polygon_coords in enumerate(coverage_polygons):
                                coverage_polygon = json.dumps(polygon_coords)
                                dronePolygons.append(coverage_polygon)
                                area_name = f"{drone.drone_name} Coverage {poly_idx + 1}" if len(coverage_polygons) > 1 else f"{drone.drone_name} Coverage"
                                droneData.append([area_name, droneTelemetrySessions[0][0]['time']])
                                
                                # Calculate rough area for logging
                                polygon_area = 0
                                if len(polygon_coords) >= 4:
                                    # Simple area calculation for debugging
                                    for i in range(len(polygon_coords) - 1):
                                        polygon_area += (polygon_coords[i][0] * polygon_coords[i+1][1] - polygon_coords[i+1][0] * polygon_coords[i][1])
                                    polygon_area = abs(polygon_area) / 2
                                
                                print(f"Created accurate coverage area {poly_idx + 1} for {drone.drone_name}: {len(polygon_coords)} points, ~{polygon_area:.8f} deg² area", flush=True)
                        else:
                            # Fallback to union of all FOV polygons
                            print(f"Grid-based coverage failed for {drone.drone_name}, using FOV union approach", flush=True)
                            
                            # Simple approach: create convex hull of all FOV polygon points
                            all_fov_points = []
                            for fov_polygon in all_fov_polygons:
                                all_fov_points.extend(fov_polygon[:-1])  # Exclude closing point
                            
                            if len(all_fov_points) >= 3:
                                def convex_hull(points):
                                    def cross_product(o, a, b):
                                        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
                                    
                                    points = sorted(set(tuple(p) for p in points))
                                    if len(points) <= 1:
                                        return points
                                    
                                    # Build lower hull
                                    lower = []
                                    for p in points:
                                        while len(lower) >= 2 and cross_product(lower[-2], lower[-1], p) <= 0:
                                            lower.pop()
                                        lower.append(p)
                                    
                                    # Build upper hull
                                    upper = []
                                    for p in reversed(points):
                                        while len(upper) >= 2 and cross_product(upper[-2], upper[-1], p) <= 0:
                                            upper.pop()
                                        upper.append(p)
                                    
                                    return lower[:-1] + upper[:-1]
                                
                                hull_points = convex_hull(all_fov_points)
                                hull_coords = [[p[0], p[1]] for p in hull_points]
                                
                                if len(hull_coords) > 0 and hull_coords[0] != hull_coords[-1]:
                                    hull_coords.append(hull_coords[0])
                                
                                if len(hull_coords) >= 4:
                                    coverage_polygon = json.dumps(hull_coords)
                                    dronePolygons.append(coverage_polygon)
                                    droneData.append([f"{drone.drone_name} Coverage", droneTelemetrySessions[0][0]['time']])
                                    print(f"Created fallback coverage area for {drone.drone_name} with {len(hull_coords)} points", flush=True)
                    
                    except Exception as e:
                        # Final fallback: create a simple bounding box
                        print(f"Error creating grid-based coverage for {drone.drone_name}, using bounding box: {e}", flush=True)
                        
                        # Find bounding box of all FOV points
                        all_fov_points = [point for polygon in all_fov_polygons for point in polygon[:-1]]
                        if all_fov_points:
                            min_lon = min(point[0] for point in all_fov_points)
                            max_lon = max(point[0] for point in all_fov_points)
                            min_lat = min(point[1] for point in all_fov_points)
                            max_lat = max(point[1] for point in all_fov_points)
                            
                            # Create bounding box polygon
                            bbox_polygon = [
                                [min_lon, min_lat],
                                [max_lon, min_lat],
                                [max_lon, max_lat],
                                [min_lon, max_lat],
                                [min_lon, min_lat]  # Close the polygon
                            ]
                            
                            coverage_polygon = json.dumps(bbox_polygon)
                            dronePolygons.append(coverage_polygon)
                            droneData.append([f"{drone.drone_name} Coverage Area", droneTelemetrySessions[0][0]['time']])
                            print(f"Created bounding box coverage for {drone.drone_name}", flush=True)
                        droneData.append([f"{drone.drone_name} Coverage Area", droneTelemetrySessions[0][0]['time']])
                        print(f"Created bounding box coverage for {drone.drone_name}", flush=True)

            except Exception as e:
                print(e)

    #devices
    if(requestData['getDevices'] == True):
        if(requestData['getPoints'] == True):
            deviceAllTelemetryPoints = list(DeviceTelemetry.objects.filter(operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('latitude', 'longitude'))
        devices = Device.objects.all()
        for device in devices:
            try:
                deviceTelemetry = list(DeviceTelemetry.objects.filter(device_id=device.id, operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('latitude', 'longitude', 'secondsOn', 'time'))
                previousSecondsOn = 0
                currentSecondsOn = 0
                # break down device's telemetry into sessions based on the secondsOn field
                deviceTelemetrySessions = []
                sessionPoints = []
                for t in deviceTelemetry:
                    if(t['latitude'] != 0 and t['longitude'] != 0):
                        currentSecondsOn = t['secondsOn']
                        if abs(currentSecondsOn - previousSecondsOn) > 29:
                            if(len(sessionPoints)) > 0:
                                deviceTelemetrySessions.append(sessionPoints)
                            sessionPoints = []

                        sessionPoints.append([t['latitude'], t['longitude'], t['time']])
                        previousSecondsOn = currentSecondsOn

                if(len(sessionPoints)) > 0:
                    deviceTelemetrySessions.append(sessionPoints) # append the last session

                # # loop telemetry sessions and create polygons
                # for s in deviceTelemetrySessions:
                #     deviceTelemetryTuple = [(d[0], d[1]) for d in s]
                #     hull = ConvexHull(deviceTelemetryTuple)
                #     devicePolygon = json.dumps([hull.points[i].tolist() for i in hull.vertices])
                #     devicePolygons.append(devicePolygon)
                #     deviceData.append([device.name, s[0][2]])
                
                # loop telemetry sessions and create polygons
                for s in deviceTelemetrySessions:
                    deviceTelemetryTuple = [(d[0], d[1]) for d in s]
                    try:
                        shaper = Alpha_Shaper(deviceTelemetryTuple)
                        alpha_shape = shaper.get_shape(alpha=5)
                        shapeMapping = mapping(alpha_shape)
                        for i in range(len(shapeMapping['coordinates'])):
                            devicePolygon = json.dumps(shapeMapping['coordinates'][i])
                            devicePolygons.append(devicePolygon)
                            deviceData.append([device.name, s[0][2]])
                    except Exception as e:
                        print(e, flush=True)                  

            except Exception as e:
                print(e)

    # baloras
    if(requestData['getBaloras'] == True):
        if(requestData['getPoints'] == True):
            baloraAllTelemetryPoints = list(BaloraTelemetry.objects.filter(operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('latitude', 'longitude'))
        baloras = Balora.objects.all()
        for balora in baloras:
            try:
                baloraTelemetry = list(BaloraTelemetry.objects.filter(balora_id=balora.id, operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('latitude', 'longitude', 'secondsOn', 'time'))
                previousSecondsOn = 0
                currentSecondsOn = 0
                # break down balora's telemetry into sessions based on the secondsOn field
                baloraTelemetrySessions = []
                sessionPoints = []
                for t in baloraTelemetry:
                    if(t['latitude'] != 0 and t['longitude'] != 0):
                        currentSecondsOn = t['secondsOn']
                        if abs(currentSecondsOn - previousSecondsOn) > 29:
                            if(len(sessionPoints)) > 0:
                                baloraTelemetrySessions.append(sessionPoints)
                            sessionPoints = []

                        sessionPoints.append([t['latitude'], t['longitude'], t['time']])
                        previousSecondsOn = currentSecondsOn

                if(len(sessionPoints)) > 0:
                    baloraTelemetrySessions.append(sessionPoints) # append the last session

                # # loop telemetry sessions and create polygons
                # for s in baloraTelemetrySessions:
                #     baloraTelemetryTuple = [(d[0], d[1]) for d in s]
                #     hull = ConvexHull(baloraTelemetryTuple)
                #     baloraPolygon = json.dumps([hull.points[i].tolist() for i in hull.vertices])
                #     baloraPolygons.append(baloraPolygon)
                #     baloraData.append([balora.name, s[0][2]])

                # loop telemetry sessions and create polygons
                for s in baloraTelemetrySessions:
                    baloraTelemetryTuple = [(d[0], d[1]) for d in s]
                    try:
                        shaper = Alpha_Shaper(baloraTelemetryTuple)
                        alpha_shape = shaper.get_shape(alpha=5)
                        shapeMapping = mapping(alpha_shape)
                        for i in range(len(shapeMapping['coordinates'])):
                            baloraPolygon = json.dumps(shapeMapping['coordinates'][i])
                            baloraPolygons.append(baloraPolygon)
                            baloraData.append([balora.name, s[0][2]])
                    except Exception as e:
                        print(e, flush=True)                


            except Exception as e:
                print(e)

    response_data = {
        'dronePolygons': dronePolygons,
        'droneData': droneData,
        'devicePolygons': devicePolygons,
        'deviceData': deviceData,
        'baloraPolygons': baloraPolygons,
        'baloraData': baloraData,
        'droneAllTelemetryPoints': json.dumps(droneAllTelemetryPoints),
        'deviceAllTelemetryPoints': json.dumps(deviceAllTelemetryPoints),
        'baloraAllTelemetryPoints': json.dumps(baloraAllTelemetryPoints),
    }

    return JsonResponse(response_data, safe=False)



####################################
######### SESSION REPLAYS ##########
####################################


def getAvailableDroneSessions(request, *args, **kwargs):
    if request.method == 'GET':
        operation = Operation.objects.get(operation_name=kwargs["operation_name"])
        stream_type = kwargs["stream_type"]
        print(stream_type, flush=True)
        result = []
        drones = Drone.objects.filter(operation_id=operation.id)
        for drone in drones:

            if stream_type == "raw":
                sessions = LiveStreamSession.objects.filter(drone_id=drone.id)
                for session in sessions:
                    frame_count = RawFrame.objects.filter(live_stream_session_id=session.id).count()
                    if frame_count == 0:
                        continue
                    session_end = session.end_time.strftime('%Y-%m-%d %H:%M:%S') if session.end_time else ""
                    result.append({
                        'sessionId': session.id,
                        'droneName': session.drone.drone_name,
                        'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                        'endTime': session_end,
                        'count': frame_count
                    })
            else:
                sessions = DetectionSession.objects.filter(drone_id=drone.id)
                for session in sessions:
                    frame_count = DetectionFrame.objects.filter(detection_session_id=session.id).count()
                    if frame_count == 0:
                        continue
                    session_end = session.end_time.strftime('%Y-%m-%d %H:%M:%S') if session.end_time else ""
                    result.append({
                        'sessionId': session.id,
                        'droneName': session.drone.drone_name,
                        'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                        'endTime': session_end,
                        'count': frame_count
                    })

        return HttpResponse(json.dumps(result))


def drone_session_replay(request, stream_type, session_id):
    if not request.user.is_authenticated:
        return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": "/home"})

    print(stream_type, flush=True) 

    if stream_type == "raw":
        session = LiveStreamSession.objects.get(id=session_id)
        frames = list(RawFrame.objects.filter(live_stream_session_id=session_id).values('frame', 'time'))
    else:
        session = DetectionSession.objects.get(id=session_id)
        frames = list(DetectionFrame.objects.filter(detection_session_id=session_id).values('frame', 'time'))

    drone_id = session.drone_id
    drone = Drone.objects.get(id=drone_id)
     
    sessionStart = frames[0]["time"]
    sessionEnd = frames[-1]["time"]

    telemetry = list(Telemetry.objects.filter(drone_id=drone_id, time__gte=sessionStart, time__lt=sessionEnd).values('time', 'lat', 'lon', 'heading', 'alt', 'velocity', 'battery_percentage', 'gimbal_angle', 'drone_state'))
    monitoring_data = list(ControlDevice.objects.filter(drone_id=drone_id, time__gte=sessionStart, time__lt=sessionEnd).values('time', 'cpu_usage', 'cpu_temp'))

    has_monitoring_data = 0
    if len(monitoring_data) > 0:
        has_monitoring_data=1

    # retrieve build map images
    build_map_session = list(BuildMapSession.objects.filter(drone_id=drone_id, start_time__gte=sessionStart, end_time__lt=sessionEnd).order_by('-start_time').values())
    if(len(build_map_session) == 0):
        build_map_images = []
    else:
        build_map_images = list(BuildMapImage.objects.filter(session_id=build_map_session[0]["id"]).order_by('time').values("path", "top_left", "top_right", "bottom_left", "bottom_right", "centre", "time"))
        for image in build_map_images:
            image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S "))
            image["top_left"] = [float(image["top_left"].coords[0]), float(image["top_left"].coords[1])]
            image["top_right"] = [float(image["top_right"].coords[0]), float(image["top_right"].coords[1])]
            image["bottom_left"] = [float(image["bottom_left"].coords[0]), float(image["bottom_left"].coords[1])]
            image["bottom_right"] = [float(image["bottom_right"].coords[0]), float(image["bottom_right"].coords[1])]
            image["centre"] = [float(image["centre"].coords[0]), float(image["centre"].coords[1])]    

    # Sync the frames with telemetry
    frames_with_telemetry = []
    for frame in frames:
        # Add the telemetry data to the frame
        closest_telemetry = find_closest_telemetry(frame['time'], telemetry)
        frame.update(closest_telemetry)
        if has_monitoring_data:
            closest_monitor_data = find_closest_telemetry(frame['time'], monitoring_data)
            frame['cpu_usage'] = closest_monitor_data['cpu_usage']
            frame['cpu_temp'] = closest_monitor_data['cpu_temp']
        frame['time'] = frame['time'].strftime('%Y-%m-%dT%H:%M:%S.%fZ')
        frames_with_telemetry.append(frame)


    distance = round(get_total_path_distance(telemetry, 'lat', 'lon'), 2)
    max_altitude = round(max(telemetry, key=lambda x: x['alt'])['alt'], 2)
    max_velocity = round(max(telemetry, key=lambda x: x['velocity'])['velocity'], 2)
    battery_used = round(telemetry[0]['battery_percentage'] - telemetry[-1]['battery_percentage'], 2)

    if has_monitoring_data:
        max_cpu_usage = round(max(monitoring_data, key=lambda x: x['cpu_usage'])['cpu_usage'], 2)
        max_cpu_temp = round(max(monitoring_data, key=lambda x: x['cpu_temp'])['cpu_temp'], 2)
    else:
        max_cpu_usage = 0
        max_cpu_temp = 0

    time_difference = sessionEnd - sessionStart
    time_difference_seconds = int(time_difference.total_seconds())

    # calculate the duration of the session
    hours, remainder = divmod(time_difference_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    duration = "{:02}:{:02}:{:02}".format(hours, minutes, seconds)

    # calculate battery % per minute
    battery_used_per_minute = round(battery_used / (time_difference_seconds / 60), 2)

    session_data = {
        'drone': drone,
        'session_id': session_id,
        'stream_type': stream_type,
        'distance': distance,
        'duration': duration,
        'max_altitude': max_altitude,
        'max_velocity': max_velocity,
        'battery_used': battery_used,
        'battery_used_per_minute': battery_used_per_minute,
        'has_monitoring_data': has_monitoring_data,
        'max_cpu_usage': max_cpu_usage,
        'max_cpu_temp': max_cpu_temp,
        'build_map_images': json.dumps(build_map_images),
    }

    return render(request, 'aiders/drone_session_replay.html', {'frames_with_telemetry': frames_with_telemetry, 'session_data': session_data})


def drone_video_session_replay(request, session_id):
    """
    Video-based session replay using MP4 recordings instead of individual frames
    """
    if not request.user.is_authenticated:
        return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": "/home"})

    try:
        session = LiveStreamSession.objects.get(id=session_id)
    except LiveStreamSession.DoesNotExist:
        return Http404("Session not found")

    # Check if session has a recording URL
    if not session.recording_url:
        return render(request, 'aiders/error.html', {
            'error_message': 'No video recording available for this session'
        })

    drone = session.drone
    session_start = session.start_time
    session_end = session.end_time or timezone.now()

    # Get telemetry data for the session duration
    telemetry = list(Telemetry.objects.filter(
        drone=drone, 
        time__gte=session_start, 
        time__lte=session_end
    ).values('time', 'lat', 'lon', 'heading', 'alt', 'velocity', 'battery_percentage', 'gimbal_angle', 'drone_state'))

    # Get monitoring data if available
    monitoring_data = list(ControlDevice.objects.filter(
        drone=drone, 
        time__gte=session_start, 
        time__lte=session_end
    ).values('time', 'cpu_usage', 'cpu_temp'))

    has_monitoring_data = len(monitoring_data) > 0

    # Get build map images for the session
    build_map_session = list(BuildMapSession.objects.filter(
        drone=drone, 
        start_time__gte=session_start, 
        end_time__lte=session_end
    ).order_by('-start_time').values())

    build_map_images = []
    if build_map_session:
        build_map_images = list(BuildMapImage.objects.filter(
            session_id=build_map_session[0]["id"]
        ).order_by('time').values("path", "top_left", "top_right", "bottom_left", "bottom_right", "centre", "time"))
        
        for image in build_map_images:
            image["time"] = str(image["time"].astimezone(pytz.timezone(settings.TIME_ZONE)).strftime("%H:%M:%S"))
            image["top_left"] = [float(image["top_left"].coords[0]), float(image["top_left"].coords[1])]
            image["top_right"] = [float(image["top_right"].coords[0]), float(image["top_right"].coords[1])]
            image["bottom_left"] = [float(image["bottom_left"].coords[0]), float(image["bottom_left"].coords[1])]
            image["bottom_right"] = [float(image["bottom_right"].coords[0]), float(image["bottom_right"].coords[1])]
            image["centre"] = [float(image["centre"].coords[0]), float(image["centre"].coords[1])]

    # Convert telemetry timestamps to seconds from session start for video sync
    telemetry_for_video = []
    try:
        for telem in telemetry:
            time_offset = (telem['time'] - session_start).total_seconds()
            telem_data = {
                'video_time': float(time_offset),
                'time_str': telem['time'].strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
                'lat': float(telem['lat']) if telem['lat'] is not None else 0.0,
                'lon': float(telem['lon']) if telem['lon'] is not None else 0.0,
                'heading': float(telem['heading']) if telem['heading'] is not None else 0.0,
                'alt': float(telem['alt']) if telem['alt'] is not None else 0.0,
                'velocity': float(telem['velocity']) if telem['velocity'] is not None else 0.0,
                'battery_percentage': float(telem['battery_percentage']) if telem['battery_percentage'] is not None else 0.0,
                'gimbal_angle': float(telem['gimbal_angle']) if telem['gimbal_angle'] is not None else 0.0,
                'drone_state': str(telem['drone_state']) if telem['drone_state'] is not None else '',
            }
            telemetry_for_video.append(telem_data)
    except Exception as e:
        print(f"Error processing telemetry data: {e}")
        telemetry_for_video = []

    # Convert monitoring data timestamps for video sync
    monitoring_for_video = []
    try:
        if has_monitoring_data:
            for monitor in monitoring_data:
                time_offset = (monitor['time'] - session_start).total_seconds()
                monitor_data = {
                    'video_time': float(time_offset),
                    'time_str': monitor['time'].strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
                    'cpu_usage': float(monitor['cpu_usage']) if monitor['cpu_usage'] is not None else 0.0,
                    'cpu_temp': float(monitor['cpu_temp']) if monitor['cpu_temp'] is not None else 0.0,
                }
                monitoring_for_video.append(monitor_data)
    except Exception as e:
        print(f"Error processing monitoring data: {e}")
        monitoring_for_video = []

    # Calculate session statistics
    distance = max_altitude = max_velocity = battery_used = 0
    try:
        if telemetry and len(telemetry) > 0:
            distance = round(get_total_path_distance(telemetry, 'lat', 'lon'), 2)
            max_altitude = round(max(telemetry, key=lambda x: x.get('alt', 0) or 0)['alt'] or 0, 2)
            max_velocity = round(max(telemetry, key=lambda x: x.get('velocity', 0) or 0)['velocity'] or 0, 2)
            first_battery = telemetry[0].get('battery_percentage', 0) or 0
            last_battery = telemetry[-1].get('battery_percentage', 0) or 0
            battery_used = round(first_battery - last_battery, 2)
    except Exception as e:
        print(f"Error calculating telemetry statistics: {e}")

    max_cpu_usage = max_cpu_temp = 0
    try:
        if has_monitoring_data and monitoring_data and len(monitoring_data) > 0:
            max_cpu_usage = round(max(monitoring_data, key=lambda x: x.get('cpu_usage', 0) or 0)['cpu_usage'] or 0, 2)
            max_cpu_temp = round(max(monitoring_data, key=lambda x: x.get('cpu_temp', 0) or 0)['cpu_temp'] or 0, 2)
    except Exception as e:
        print(f"Error calculating monitoring statistics: {e}")

    # Calculate session duration
    time_difference = session_end - session_start
    time_difference_seconds = int(time_difference.total_seconds())
    hours, remainder = divmod(time_difference_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    duration = "{:02}:{:02}:{:02}".format(hours, minutes, seconds)

    # Calculate battery usage per minute
    battery_used_per_minute = round(battery_used / max(time_difference_seconds / 60, 1), 2)

    # Create a single JSON object with all data for JavaScript
    session_data_for_js = {
        'session_data': {
            'session_id': session_id,
            'video_url': session.recording_url or '',
            'distance': float(distance) if distance else 0.0,
            'duration': str(duration),
            'max_altitude': float(max_altitude) if max_altitude else 0.0,
            'max_velocity': float(max_velocity) if max_velocity else 0.0,
            'battery_used': float(battery_used) if battery_used else 0.0,
            'battery_used_per_minute': float(battery_used_per_minute) if battery_used_per_minute else 0.0,
            'has_monitoring_data': bool(has_monitoring_data),
            'max_cpu_usage': float(max_cpu_usage) if max_cpu_usage else 0.0,
            'max_cpu_temp': float(max_cpu_temp) if max_cpu_temp else 0.0,
            'session_start': session_start.strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
            'session_duration_seconds': int(time_difference_seconds),
        },
        'telemetry_data': telemetry_for_video,
        'monitoring_data': monitoring_for_video,
        'build_map_images': build_map_images or []
    }
    
    # Debug: print the JSON to make sure it's valid
    try:
        json_string = json.dumps(session_data_for_js)
        print(f"JSON data length: {len(json_string)}")
        print(f"Sample JSON: {json_string[:200]}...")
    except Exception as e:
        print(f"Error serializing JSON: {e}")
        # Fallback to empty data
        session_data_for_js = {
            'session_data': {},
            'telemetry_data': [],
            'monitoring_data': [],
            'build_map_images': []
        }

    # Data for template (includes Django objects)
    session_data = {
        'drone': drone,
        'session': session,
        'session_id': session_id,
        'video_url': session.recording_url or '',
        'distance': distance or 0,
        'duration': duration,
        'max_altitude': max_altitude or 0,
        'max_velocity': max_velocity or 0,
        'battery_used': battery_used or 0,
        'battery_used_per_minute': battery_used_per_minute or 0,
        'has_monitoring_data': has_monitoring_data,
        'max_cpu_usage': max_cpu_usage or 0,
        'max_cpu_temp': max_cpu_temp or 0,
        'build_map_images': json.dumps(build_map_images) if build_map_images else '[]',
        'session_start': session_start.strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
        'session_duration_seconds': time_difference_seconds,
        # Add JSON strings for JavaScript
        'telemetry_data_json': json.dumps(telemetry_for_video) if telemetry_for_video else '[]',
        'monitoring_data_json': json.dumps(monitoring_for_video) if monitoring_for_video else '[]',
    }

    return render(request, 'aiders/drone_video_session_replay.html', {
        'session_data': session_data,
        'session_data_json': json.dumps(session_data_for_js)
    })


############################
#### KMZ SAVE DATA VIEW ####
############################


def save_kmz_results(request, operation_name):

    operation = Operation.objects.filter(operation_name=operation_name).last()

    requestData = json.loads(request.body)
    
    print(requestData.get('geoJson'), flush=True)
    algorithmObj = {
        "algorithm_name":Algorithm.KMZ_DATA_ALGORITHM,
        "output": json.loads(requestData.get('geoJson')),
        "title": requestData.get('title'),
        "input":{'test_data':'test_value'},
        'user': get_user_model().objects.all().first().id,
        'operation':operation.id,
        'canBeLoadedOnMap':True
    }
    AlgorithmRetrieveView.save_algorithm_to_db(algorithmObj)

    return JsonResponse({'message': 'KMZ data saved successfully'}, status=200)




# user defined area
def save_user_defined_area(request):
    if request.method == "POST":
        try:
            body = json.loads(request.body)
            user = request.user 
            operation_id = body.get("operation_id")
            input_data = body.get("input", {}) 
            output_data = body.get("output", {}) 
            algorithm_name = body.get("algorithm_name", "USER_DEFINED_AREA")

            alg = Algorithm.objects.create(
                algorithm_name=algorithm_name,
                title=body.get("title"),
                input=input_data,
                output=output_data,
                canBeLoadedOnMap=True,
                operation_id=operation_id,
                user=user,
                time=timezone.now()
            )

            return JsonResponse({"status": "success", "pk": alg.pk}, status=200)
        except Exception as e:
            return JsonResponse({"status": "error", "message": str(e)}, status=400)

    return JsonResponse({"status": "error", "message": "Only POST allowed"}, status=405)

# send pilot notifications
def sendPilotNotification(request, *args, **kwargs):
    if request.method == 'POST':
        from .httpRequests import postRequestForPilotNotification
        data = json.loads(request.body)
        print(data, flush=True)

        selected_drones = data.get("selectedDrones")

        # loop selected drones and send POST request to wsi for each drone
        for drone_name in selected_drones:
            sent = postRequestForPilotNotification(
                _droneName=drone_name,
                _message=data.get("message"),
                _sender=data.get("sender")
            )

            if sent:
                # create PilotNotification object in the database
                notification = PilotNotification.objects.create(
                    sender=request.user.username,
                    receiver=drone_name,
                    message=data.get('message'),
                    incoming=False,
                    operation_id=request.user.joined_operation_id,
                )

        return JsonResponse({'message': 'Notification sent successfully'}, status=200)


# get pilot notifications
def getPilotNotifications(request, operation_id, last_notification_id=0):
    if int(last_notification_id) == 0:
        notifications = PilotNotification.objects.filter(operation_id=operation_id).order_by('-id')[:20]
    else:
        notifications = PilotNotification.objects.filter(id__gt=last_notification_id, operation_id=operation_id)

    notifications_list = list(notifications.values('id', 'message', 'sender', 'receiver', 'incoming', 'timestamp'))
    if int(last_notification_id) == 0:
        notifications_list.reverse()
    return JsonResponse(notifications_list, safe=False)




# DEVICES


# send device notifications
def sendDeviceNotification(request, *args, **kwargs):
    if request.method == 'POST':
        from .httpRequests import postRequestForDeviceNotification
        data = json.loads(request.body)
        print(data, flush=True)

        selected_devices = data.get("selectedDevices")

        # loop selected devices and send POST request to wsi for each device
        for device_name in selected_devices:
            sent = postRequestForDeviceNotification(
                _deviceName=device_name,
                _message=data.get("message"),
                _sender=data.get("sender")
            )

            if sent:
                # create DeviceNotification object in the database
                notification = DeviceNotification.objects.create(
                    sender=request.user.username,
                    receiver=device_name,
                    message=data.get('message'),
                    incoming=False,
                    operation_id=request.user.joined_operation_id,
                )

        return JsonResponse({'message': 'Notification sent successfully'}, status=200)


# get device notifications
def getDeviceNotifications(request, operation_id, last_notification_id=0):
    if int(last_notification_id) == 0:
        notifications = DeviceNotification.objects.filter(operation_id=operation_id).order_by('-id')[:20]
    else:
        notifications = DeviceNotification.objects.filter(id__gt=last_notification_id, operation_id=operation_id)

    notifications_list = list(notifications.values('id', 'message', 'sender', 'receiver', 'incoming', 'timestamp'))
    if int(last_notification_id) == 0:
        notifications_list.reverse()
    return JsonResponse(notifications_list, safe=False)


def getAvailableDeviceSessions(request, *args, **kwargs):
    if request.method == 'GET':
        operation = Operation.objects.get(operation_name=kwargs["operation_name"])

        result = []
        devices = Device.objects.filter(operation_id=operation.id)
        for device in devices:


            sessions = DeviceSession.objects.filter(device_id=device.id)
            for session in sessions:
                frame_count = DeviceImage.objects.filter(session_id=session.id).count()
                if frame_count == 0:
                    continue
                session_end = session.end_time.strftime('%Y-%m-%d %H:%M:%S') if session.end_time else ""
                result.append({
                    'sessionId': session.id,
                    'name': session.device.name,
                    'startTime': session.start_time.strftime('%Y-%m-%d %H:%M:%S'),
                    'endTime': session_end,
                    'count': frame_count
                })

        return HttpResponse(json.dumps(result))
    

def device_session_replay(request, session_id):
    if not request.user.is_authenticated:
        return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": "/home"})

    session = DeviceSession.objects.get(id=session_id)
    frames = list(DeviceImage.objects.filter(session_id=session_id).values())

    device_id = session.device_id
    device = Device.objects.get(id=device_id)
     
    sessionStart = frames[0]["time"]
    sessionEnd = frames[-1]["time"]

    for frame in frames:
        frame['time'] = frame['time'].strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    distance = round(get_total_path_distance(frames, 'latitude', 'longitude'), 2)

    time_difference = sessionEnd - sessionStart
    time_difference_seconds = int(time_difference.total_seconds())

    # calculate the duration of the session
    hours, remainder = divmod(time_difference_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    duration = "{:02}:{:02}:{:02}".format(hours, minutes, seconds)

    session_data = {
        'device': device,
        'session_id': session_id,
        'distance': distance,
        'duration': duration,
    }

    return render(request, 'aiders/device_session_replay.html', {'frames': frames, 'session_data': session_data})



def find_closest_telemetry(time, telemetry):
    # Initialize the closest telemetry and minimum difference with the first telemetry data
    closest_telemetry = telemetry[0]
    min_diff = abs(time - telemetry[0]['time'])

    # Iterate over the telemetry data
    for data in telemetry:
        diff = abs(time - data['time'])
        # If the current difference is less than the minimum difference, update the closest telemetry and minimum difference
        if diff < min_diff:
            closest_telemetry = data
            min_diff = diff

    return closest_telemetry


def get_total_path_distance(telemetry_data, latKey, lonKey):
    from haversine import haversine
    total = 0
    for i in range(1, len(telemetry_data)):
        lat1, lon1 = telemetry_data[i-1][latKey], telemetry_data[i-1][lonKey]
        lat2, lon2 = telemetry_data[i][latKey], telemetry_data[i][lonKey]
        total += haversine([lat1, lon1], [lat2, lon2])
    return total


#############################################################################################################
#  Safe drones

def safeDronesResults(request):
    calculations_safe_drones.start()

    # print(datetime.datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES RESULTS REQUESTED", flush=True)

    
    from django.db.models import OuterRef, Subquery

    # Subquery to get the latest DateTime for each drone
    latest_datetimes = SafeDroneResults.objects.filter(
        drone=OuterRef('drone')
    ).order_by('-DateTime').values('DateTime')[:1]

    # Retrieve the last entry for each drone with the drone's name
    last_entries = SafeDroneResults.objects.filter(
        DateTime=Subquery(latest_datetimes)
    ).annotate(drone_name=F('drone__drone_name'))

    # Serialize the queryset to JSON
    data = list(last_entries.values())

    # Return the JSON response
    return JsonResponse(data, safe=False)



def postUpdateDetectionObjectDescriptionById(request, *args, **kwargs):
    if request.method == 'POST':
        data = json.loads(request.body)
        trackId = data.get("track_id")
        sessionId = data.get("detection_session_id")
        isSuspicious = data.get("is_suspicious")
        shouldFollow = data.get("should_follow", False)
        description = data.get("description")
        user = request.user
        DetectedObjectDescription.updateDescriptionByTrackIdAndSessionId(sessionId, trackId, description, isSuspicious, shouldFollow, user)
        return JsonResponse({'data':{'message':'Updated'}}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)



def getLatestManuallySetObject(request, *args, **kwargs):
    if request.method == 'POST':
        mso_data = {}
        operation_name = kwargs.get("operation_name")
        list_of_mso = list(ManuallySetObject.objects.filter(operation=Operation.objects.get(operation_name=operation_name)))
        for mso in list_of_mso:

            mso_data[mso.id] = {}
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

        return JsonResponse({'data': mso_data}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)

@method_decorator(csrf_exempt, name='dispatch')
class ManuallySetObjectAddAPIView(LoginRequiredMixin, View):

    def post(self, request, *args, **kwargs):

        return_data = {}

        operation_name = self.kwargs.get("operation_name")

        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        user = request.user

        # TODO: check if user has permission in this operation and if is allowed to add objects 

        jdata = json.loads(request.body)
        jdata["operation"] = operation.id
        #jdata["created_by"] = user.id

        location_data = {}
        location_data["lon"] = jdata["lon"] 
        location_data["lat"] = jdata["lat"]
        #location_data["received_by"] = user.id

        serializer = ManuallySetObjectSerializer(data=jdata)

        #logger.info( jdata )
        #logger.info( location_data )

        if serializer.is_valid() : 
            ManuallySetObject = serializer.save(created_by = request.user)

        else: 
            return HttpResponse(json.dumps( serializer.errors ), 
                                        content_type='application/json',
                                        status=status.HTTP_400_BAD_REQUEST )

        return_data["ManuallySetObject"] = ManuallySetObjectSerializer(ManuallySetObject).data

        #logger.info(ManuallySetObject)
        location_data["manually_set_object"] =  ManuallySetObject.id

        jdata["manually_set_object"] =  ManuallySetObject.id

        # description_data["manually_set_object"] =  ManuallySetObject.id

        ######################
        # location 

        location_serializer = ManuallySetObjectLocationSerializer(data=location_data)

        if location_serializer.is_valid() :
            ManuallySetObjectLocation = location_serializer.save(received_by=request.user)
    
            return_data["ManuallySetObjectLocation"] = ManuallySetObjectLocationSerializer(ManuallySetObjectLocation).data
        else:
            return HttpResponse(json.dumps( location_serializer.errors ), 
                                            content_type='application/json',
                                            status=status.HTTP_400_BAD_REQUEST )

        ######################
        # description 

        description_serializer = ManuallySetObjectDescriptionSerializer( data=jdata )

        if description_serializer.is_valid() : 
            ManuallySetObjectDescription_object = description_serializer.save(updated_by=request.user)

        else: 
            return HttpResponse(json.dumps( description_serializer.errors ), 
                                content_type='application/json',
                                status=status.HTTP_400_BAD_REQUEST )

        return_data["ManuallySetObjectDescription"] = ManuallySetObjectDescriptionSerializer(ManuallySetObjectDescription_object).data

        return HttpResponse(    json.dumps( return_data ) , 
                                content_type='application/json' , 
                                status=status.HTTP_200_OK )

@method_decorator(csrf_exempt, name='dispatch')
class ManuallySetObjectLocationAddAPIView(LoginRequiredMixin, View):

    def post(self, request, *args, **kwargs):

        user = request.user
        
        jdata = json.loads(request.body)

        jdata["received_by"] = user.id

        serializer = ManuallySetObjectLocationSerializer(data=jdata)

        if serializer.is_valid():

            ManuallySetObjectLocation = serializer.save(received_by=request.user)

            #logger.info( ManuallySetObjectLocation )

            ManuallySetObjectLocation_data = ManuallySetObjectLocationSerializer(ManuallySetObjectLocation).data

            return HttpResponse( json.dumps({ "ManuallySetObjectLocation" :  ManuallySetObjectLocation_data  }) , 
                                 content_type='application/json',
                                 status=status.HTTP_201_CREATED 
                                )

        return HttpResponse(    json.dumps( serializer.errors ), 
                                content_type='application/json', 
                                status=status.HTTP_400_BAD_REQUEST)

@method_decorator(csrf_exempt, name='dispatch')
class ManuallySetObjectUpdateAPIView(LoginRequiredMixin, View):

    def post(self, request, *args, **kwargs):
        
        return_data = {}

        # GET the Object 

        objid = self.kwargs.get("id")

        ManuallySetObject_Object = ManuallySetObject.objects.get(id=self.kwargs.get("id"))
        
        jdata = json.loads(request.body)

        ##################################################

        operation_name = self.kwargs.get("operation_name")

        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        user = request.user

        # TODO: check if user has permission in this operation and if is allowed to add objects 

        jdata = json.loads(request.body)
        jdata["operation"] = operation.id
        jdata["created_by"] = user.id

        ##################################################
        # Get Location object 

        ManuallySetObjectLocation_Object = ManuallySetObjectLocation.objects.filter( manually_set_object = ManuallySetObject_Object ).last()

        # check if coords are different
        # OR check a condition to change coords 
        # ... then create new location object 
        if jdata["lon"] != str(ManuallySetObjectLocation_Object.lon) or jdata["lat"] != str(ManuallySetObjectLocation_Object.lat) : 

            location_data = {}
            location_data["lon"] = jdata["lon"] 
            location_data["lat"] = jdata["lat"]
            location_data["received_by"] = request.user
            location_data["manually_set_object"] =  ManuallySetObject_Object.id

            location_serializer = ManuallySetObjectLocationSerializer(data=location_data)

            if location_serializer.is_valid() :
                ManuallySetObjectLocation_object = location_serializer.save(received_by=request.user)
            else: 
                return HttpResponse(json.dumps( location_serializer.errors ), 
                                                content_type='application/json',
                                                status=status.HTTP_400_BAD_REQUEST)

            ManuallySetObjectLocation_data = ManuallySetObjectLocationSerializer(ManuallySetObjectLocation_object).data

            return_data["ManuallySetObjectLocation"] = ManuallySetObjectLocation_data

        ############################
        # save new description oject 

        description_serializer = ManuallySetObjectDescriptionSerializer( data=jdata )

        if description_serializer.is_valid() : 
            ManuallySetObjectDescription_object = description_serializer.save(updated_by=request.user)
            #logger.info(ManuallySetObject_Updated)
        else: 
            return HttpResponse(json.dumps( description_serializer.errors ), 
                                content_type='application/json',
                                status=status.HTTP_400_BAD_REQUEST )

        return_data["ManuallySetObjectDescription"] = ManuallySetObjectDescriptionSerializer(ManuallySetObjectDescription_object).data

        return HttpResponse(    json.dumps( return_data ) , 
                                content_type='application/json' , 
                                status=status.HTTP_200_OK )



# Crisis Classification

def crisisClassificationUpdate(request, id):
    if request.user.is_authenticated:
        if request.method == 'POST':
            data = json.loads(request.body)
            description = data.get("description")
            resolved = data.get("resolved")
            false_alarm = data.get("false_alarm")
            user = request.user

            print("Updating crisis classification with ID:", id, flush=True)
            crisis_classification = CrisisClassification.objects.get(id=id)
            if crisis_classification:
                crisis_classification.description = description
                crisis_classification.resolved = resolved
                crisis_classification.false_alarm = false_alarm
                crisis_classification.updated_by = user
                crisis_classification.save()
                return JsonResponse({'message': 'Crisis classification updated successfully'}, status=200)
            else:
                return JsonResponse({'message': 'Crisis classification not found'}, status=404)
        else:
            return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)
        


# Live Stream Capture

def liveStreamCaptureStart(request, pk):
    drone = Drone.objects.get(id=pk)
    if request.method == 'GET':
        droneName = drone.drone_name
        droneId = pk
        apiResponse = startDroneLiveStreamCapture(droneId, droneName)
        return redirect("drones_list")


# MTX Live Stream Events (hook callbacks)

@method_decorator(csrf_exempt, name='dispatch')
def mtx_stream_started(request, drone_name):
    if request.method == 'POST':
        # find the recording file on the hard drive
        import time
        import requests

        # timestampDifference is the difference between the last recording start time and the request time
        # if the recording has not started yet, the timestampDifference will be negative
        # if the recording has started, the timestampDifference will be positive
        # while the timestampDifference is negative, wait for the recording to start
        requestUnixTimestamp = int(time.time()) # now
        recordingUnixTimestamp = 0
        timestampDifference = -1
        retries = 0
        recordingStarted = False
        while not recordingStarted and retries < 10:
            retries += 1
            resp = requests.get(f"http://{os.environ.get('NET_IP')}:9997/v3/recordings/get/live/{drone_name}")
            resp.raise_for_status()
            data = resp.json()
            print(data, flush=True)

            segments = data.get("segments", [])
            lastRecording = segments[-1] if segments else None
            # convert datetime to unix timestamp
            if lastRecording:
                start_time = lastRecording.get("start")
                if start_time:
                    dt_start_time = datetime.datetime.strptime(start_time, "%Y-%m-%dT%H:%M:%SZ")
                    recordingUnixTimestamp = int(dt_start_time.timestamp())
                    print(f"Start time found in the last recording: {start_time} (Unix timestamp: {recordingUnixTimestamp})", flush=True)
                    # calculate the difference
                    timestampDifference = recordingUnixTimestamp - requestUnixTimestamp
                    print(f" ------------- Timestamp difference: {timestampDifference} seconds", flush=True)
                else:
                    print("No start time found in the last recording", flush=True)
            else:
                print("No segments found in the response", flush=True)
            time.sleep(3)  # wait before checking again

        print(f"Final timestamp difference: {timestampDifference} seconds", flush=True)
        if timestampDifference >= -4:
            recordingStarted = True
            print(f"Recording started for drone: {drone_name} at {recordingUnixTimestamp}", flush=True)
            recordingFile = f"/media/live/{drone_name}-{recordingUnixTimestamp}.mp4"
            print(f"Recording filename: {recordingFile}", flush=True)

            # deactivate any live stream sessions
            try:
                live_stream_sessions = LiveStreamSession.objects.filter(drone__drone_name=drone_name, is_active=True)
                for session in live_stream_sessions:
                    session.end_time = datetime.datetime.now(pytz.timezone(settings.TIME_ZONE))
                    session.is_active = False
                    session.save()
                    print(f"Deactivated live stream session for drone {drone_name}.", flush=True)
            except LiveStreamSession.DoesNotExist:
                print(f"No active live stream session found for drone {drone_name}.", flush=True)

            # create a new live stream session
            sessionStartTime = datetime.datetime.fromtimestamp(recordingUnixTimestamp, pytz.UTC)
            print(f"session for drone {drone_name} starting at {sessionStartTime}.", flush=True)
            new_session = LiveStreamSession.objects.create(
                drone=Drone.objects.get(drone_name=drone_name),
                start_time=sessionStartTime,
                is_active=True,
                recording_url=recordingFile
            )
            print(f"Created new live stream session for drone {drone_name} with recording file {recordingFile}.", flush=True)
        else:
            print(f"Giving up! Recording has not started yet for drone: {drone_name}. Timestamp difference is still negative: {timestampDifference}", flush=True)
            return JsonResponse({'message': 'Recording has not started yet'}, status=400)

        return JsonResponse({'message': 'Stream started event received'}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)


@method_decorator(csrf_exempt, name='dispatch') 
def mtx_stream_ended(request, drone_name):
    if request.method == 'POST':
        # deactivate any live stream sessions
        try:
            live_stream_sessions = LiveStreamSession.objects.filter(drone__drone_name=drone_name, is_active=True)
            for session in live_stream_sessions:
                session.end_time = datetime.datetime.now(pytz.timezone(settings.TIME_ZONE))
                session.is_active = False
                session.save()
                print(f"Deactivated live stream session for drone {drone_name}.", flush=True)
        except LiveStreamSession.DoesNotExist:
            print(f"No active live stream session found for drone {drone_name}.", flush=True)
        print(f"Recording completed for drone: {drone_name}", flush=True)

        return JsonResponse({'message': 'Recording completed event received'}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)


@method_decorator(csrf_exempt, name='dispatch') 
def platform_is_here(request):
    """
    Endpoint to check if the platform is reachable.
    This can be used for health checks or to verify connectivity.
    """
    if request.method == 'GET':
        return JsonResponse({'message': 'Platform is here!'}, status=200)
    else:
        return JsonResponse({'message': 'Invalid request method. Only GET requests are accepted.'}, status=400)


#################################################################
######################### CHAT VIEWS ############################
#################################################################

class ChatRoomDetailView(LoginRequiredMixin, generic.DetailView):
    """Display chat room with message history"""
    model = ChatRoom
    template_name = "aiders/chat_room.html"
    context_object_name = "room"
    
    def get_object(self):
        room = super().get_object()
        user = self.request.user
        
        # Check if user can access this room
        if room.operation and user.joined_operation != room.operation:
            if not user.has_perm(f'aiders.view_operation_{room.operation.id}'):
                raise PermissionDenied("You do not have permission to access this chat room.")
        
        # Create or update user's membership
        membership, created = ChatRoomMember.objects.get_or_create(
            room=room,
            user=user,
            defaults={'is_active': True}
        )
        if not created:
            membership.update_last_seen()
            membership.is_active = True
            membership.save()
            
        return room
    
    def get_context_data(self, **kwargs):
        context = super().get_context_data(**kwargs)
        room = self.object
        
        # Get recent messages
        context['messages'] = room.messages.select_related('user').order_by('timestamp')[:50]
        
        # Get room members
        context['members'] = ChatRoomMember.objects.filter(
            room=room, 
            is_active=True
        ).select_related('user').order_by('user__username')
        
        # Add WebSocket URL
        context['room_id'] = room.id
        
        return context


@login_required
def chat_room_messages_api(request, room_id):
    """API endpoint to get chat messages for a room"""
    if request.method == 'GET':
        try:
            room = ChatRoom.objects.get(id=room_id)
            user = request.user
            
            # Check access permission
            if room.operation and user.joined_operation != room.operation:
                if not user.has_perm(f'aiders.view_operation_{room.operation.id}'):
                    return JsonResponse({'error': 'Permission denied'}, status=403)
            
            # Get pagination parameters
            page = int(request.GET.get('page', 1))
            per_page = int(request.GET.get('per_page', 50))
            
            # Get messages
            messages = room.messages.select_related('user').order_by('-timestamp')
            
            # Paginate
            start = (page - 1) * per_page
            end = start + per_page
            messages_page = messages[start:end]
            
            # Serialize messages
            messages_data = []
            for msg in messages_page:
                messages_data.append({
                    'id': msg.id,
                    'user': msg.user.username,
                    'user_id': msg.user.id,
                    'content': msg.content,
                    'timestamp': msg.timestamp.isoformat(),
                    'is_edited': msg.is_edited,
                    'edited_at': msg.edited_at.isoformat() if msg.edited_at else None
                })
            
            return JsonResponse({
                'messages': messages_data,
                'has_more': len(messages) > end
            })
            
        except ChatRoom.DoesNotExist:
            return JsonResponse({'error': 'Room not found'}, status=404)
        except Exception as e:
            return JsonResponse({'error': str(e)}, status=500)
    
    return JsonResponse({'error': 'Method not allowed'}, status=405)


@login_required
def get_operation_chat_room(request, operation_id):
    """Get or create chat room for an operation"""
    try:
        operation = Operation.objects.get(id=operation_id)
        # Check if user has permission to view this operation
        if not request.user.has_perm(f"aiders.view_operation_{operation.id}"):
            return JsonResponse({'error': 'Permission denied'}, status=403)
        
        # Get or create chat room for this operation
        room, created = ChatRoom.get_or_create_for_operation(operation)
        
        # Add user as member if not already
        member, member_created = ChatRoomMember.objects.get_or_create(
            room=room,
            user=request.user,
            defaults={'is_active': True}
        )
        if not member_created and not member.is_active:
            member.is_active = True
            member.save()
        
        return JsonResponse({
            'room_id': room.id,
            'room_name': room.name,
            'created': created
        })
    except Operation.DoesNotExist:
        return JsonResponse({'error': 'Operation not found'}, status=404)
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)


# Weather API Views
import requests

@login_required
def weather_config_view(request):
    """View for configuring weather API settings"""
    # from .forms.weather import WeatherConfigForm
    
    try:
        config = WeatherConfig.objects.first()
    except WeatherConfig.DoesNotExist:
        config = None
    
    if request.method == 'POST':
        form = WeatherConfigForm(request.POST, instance=config)
        if form.is_valid():
            config = form.save()
            messages.success(request, 'Weather configuration saved successfully!')
            return redirect('weather_config')
        else:
            messages.error(request, 'Please correct the errors below.')
    else:
        form = WeatherConfigForm(instance=config)
    
    # Get recent weather data
    recent_weather = WeatherAPI.objects.all()[:10]
    
    return render(request, 'aiders/weather_config.html', {
        'form': form,
        'config': config,
        'recent_weather': recent_weather
    })


@login_required
def weather_data_view(request):
    """View for displaying weather data"""
    weather_data = WeatherAPI.objects.all()[:50]  # Show last 50 entries
    
    return render(request, 'aiders/weather_data.html', {
        'weather_data': weather_data
    })


def fetch_weather_data():
    """Function to fetch weather data from WeatherAPI.com"""
    try:
        config = WeatherConfig.objects.filter(is_active=True).first()
        if not config:
            print("No active weather configuration found")
            return
        
        cities = config.get_cities_list()
        api_key = config.api_key
        
        for city in cities:
            try:
                # WeatherAPI.com current weather endpoint
                url = f"http://api.weatherapi.com/v1/current.json"
                params = {
                    'key': api_key,
                    'q': city,
                    'aqi': 'no'  # Don't include air quality data
                }
                
                response = requests.get(url, params=params, timeout=10)
                response.raise_for_status()
                data = response.json()
                
                # Parse weather data from WeatherAPI.com response
                location = data['location']
                current = data['current']
                
                weather_entry = WeatherAPI(
                    city=location['name'],
                    country=location['country'],
                    region=location['region'],
                    latitude=location['lat'],
                    longitude=location['lon'],
                    temperature=current['temp_c'],
                    feels_like=current['feelslike_c'],
                    humidity=current['humidity'],
                    pressure=current['pressure_mb'],
                    wind_speed=current['wind_kph'],
                    wind_direction=current['wind_degree'],
                    wind_dir_text=current['wind_dir'],
                    weather_condition=current['condition']['text'],
                    weather_icon=current['condition']['icon'],
                    visibility=current['vis_km'],
                    uv_index=current['uv'],
                    cloud_cover=current['cloud'],
                    gust_kph=current.get('gust_kph'),
                    api_timestamp=timezone.make_aware(
                        timezone.datetime.fromisoformat(current['last_updated'].replace(' ', 'T')),
                        timezone.get_current_timezone()
                    )
                )
                weather_entry.save()
                print(f"Weather data saved for {city}")
                
            except requests.exceptions.RequestException as e:
                print(f"Error fetching weather for {city}: {e}")
            except Exception as e:
                print(f"Error processing weather data for {city}: {e}")
                
    except Exception as e:
        print(f"Error in fetch_weather_data: {e}")


@login_required  
def manual_weather_update(request):
    """Manual trigger for weather data update"""
    if request.method == 'POST':
        try:
            fetch_weather_data()
            messages.success(request, 'Weather data updated successfully!')
        except Exception as e:
            messages.error(request, f'Error updating weather data: {str(e)}')
    
    return redirect('weather_config')


@login_required
def weather_map_data(request):
    """API endpoint to serve weather data for map display"""
    try:
        # Get the latest weather data for each city (most recent entry per city)
        from django.db.models import Max
        from django.utils import timezone
        from datetime import timedelta
        
        # Calculate the cutoff time (1 hour ago)
        one_hour_ago = timezone.now() - timedelta(hours=1)
        
        # Get the latest timestamp for each city, but only for data within the last hour
        latest_entries = WeatherAPI.objects.filter(
            timestamp__gte=one_hour_ago
        ).values('city', 'country').annotate(
            latest_time=Max('timestamp')
        )
        
        # Get the actual weather records for those latest timestamps
        weather_data = []
        for entry in latest_entries:
            latest_weather = WeatherAPI.objects.filter(
                city=entry['city'],
                country=entry['country'],
                timestamp=entry['latest_time'],
                timestamp__gte=one_hour_ago  # Double-check the timestamp is within 1 hour
            ).first()
            
            if latest_weather:
                weather_data.append({
                    'city': latest_weather.city,
                    'country': latest_weather.country,
                    'region': latest_weather.region,
                    'latitude': latest_weather.latitude,
                    'longitude': latest_weather.longitude,
                    'temperature': latest_weather.temperature,
                    'feels_like': latest_weather.feels_like,
                    'humidity': latest_weather.humidity,
                    'pressure': latest_weather.pressure,
                    'wind_speed': latest_weather.wind_speed,
                    'wind_direction': latest_weather.wind_direction,
                    'wind_dir_text': latest_weather.wind_dir_text,
                    'weather_condition': latest_weather.weather_condition,
                    'weather_icon': latest_weather.weather_icon,
                    'visibility': latest_weather.visibility,
                    'uv_index': latest_weather.uv_index,
                    'cloud_cover': latest_weather.cloud_cover,
                    'timestamp': latest_weather.timestamp.isoformat(),
                })
        
        return JsonResponse({
            'status': 'success',
            'weather_data': weather_data,
            'count': len(weather_data)
        })
        
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': str(e)
        }, status=500)
