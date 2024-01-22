import csv
import json
import logging
import math
import os
import shutil
import sys
import threading
import time
import zipfile
from datetime import datetime
from decimal import Decimal

import numpy as np
import open3d as o3d
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
from django.core.paginator import Paginator
from django.db.models import Avg
from django.forms.models import model_to_dict
from django.http import (FileResponse, Http404, HttpResponse,
                         HttpResponseNotFound, HttpResponseRedirect,
                         JsonResponse)
from django.shortcuts import (get_list_or_404, get_object_or_404, redirect,
                              render)
from django.urls import resolve, reverse, reverse_lazy
from django.utils import timezone
from django.utils.decorators import method_decorator
from django.views import View, generic
from django.views.decorators.csrf import csrf_exempt, csrf_protect
from guardian.shortcuts import assign_perm
from json2html import *
from logic import utils
from logic.algorithms.build_map import (build_map_request_handler,
                                        img_georeference)
from logic.algorithms.external_request import patho_request
from logic.algorithms.flying_report import flying_report
from logic.algorithms.mission import mission_request_handler
from logic.algorithms.safe_drones import calculations_safe_drones
from logic.Constants import Constants
from PIL import Image
from rest_framework import generics, permissions, status
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework.views import APIView

from .factories import *
from .forms import *
from .httpRequests import (postDetectionStartToCv, postDetectionStopToCv,
                           postRequestForLidarStartOrStop,
                           postRequestForOpenWaterSamplingValve, 
                           postDetectionStartToSafeML, postDetectionStopToSafeML)
from .models import ManuallySetObject, ManuallySetObjectLocation, Operation
from .permissions import IsOwnerOrReadOnly
from .serializers import *

from django import forms

logger = logging.getLogger(__name__)

# Function for creating Thread instances with stop function and timer function


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


class DatabaseFiller(APIView):
    """
    A class that populates the database with dummy data.
    It utilizes the Factory notion, using the Factory Boy library
    Reference: https://factoryboy.readthedocs.io/en/stable/orms.html
    """

    def get(self, request):
        UserFactory.create_batch(20)
        OperationFactory.create_batch(20)
        mission_points = MissionPointFactory.create_batch(10)
        MissionFactory.create_batch(20, mission_points=tuple(mission_points))
        mission = Mission.objects.all().first()
        drones = DroneFactory.create_batch(20)
        WeatherStationFactory.create_batch(50)
        TelemetryFactory.create_batch(50)
        LiveStreamSessionFactory.create_batch(20)
        RawFrameFactory.create_batch(20)
        DetectionFactory.create_batch(20)
        DetectionSessionFactory.create_batch(50)
        DetectionFrameFactory.create_batch(20)
        DetectedObjectFactory.create_batch(20)
        AlgorithmFactory.create_batch(20)
        WaterSamplerFactory.create_batch(20)
        ErrorMessageFactory.create_batch(20)
        FrontEndUserInputFactory.create_batch(20)
        LidarPointSessionFactory.create_batch(20)
        LidarPointFactory.create_batch(20)
        BuildMapImageFactory.create_batch(50)
        BuildMapSessionFactory.create_batch(20)
        ControlDeviceFactory.create_batch(20)
        MissionLogFactory.create_batch(20)
        return redirect("login")


class OperationListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    """
    List all operations or create new one. The get and create methods are inherited,
    using the generics.ListCreateAPIView.
    Tutorial Reference: https://www.django-rest-framework.org/tutorial/3-class-based-views/
    """

    queryset = Operation.objects.all()
    serializer_class = OperationSerializer

    """
     Ensure that authenticated requests get read-write access, and unauthenticated requests get read-only access
    """
    permission_classes = [permissions.IsAuthenticatedOrReadOnly, IsOwnerOrReadOnly]

    def perform_create(self, serializer):
        """
        Allows us to modify how the instance save is managed,
        and handle any information that is implicit in the incoming request or requested URL.
        """
        serializer.save(operator=self.request.user)  # Operations are associated with the user that created them


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
        if lidarCommand == "START":
            if latestSession is not None:
                return JsonResponse(
                    {"message": "Lidar Session is already running.", "lidar_session_id": latestSession.id}, status=200
                )
            latestSession = LidarPointSession.objects.create(
                user_id=userId, operation_id=operationId, drone_id=droneId, is_active=True
            )
            postRequestForLidarStartOrStop(droneId, droneName, latestSession.id, lidarCommand)
            return JsonResponse(
                {"message": "Lidar Session is started.", "lidar_session_id": latestSession.id}, status=200
            )
        elif lidarCommand == "STOP":
            print(latestSession is not None, flush=True)
            if latestSession is not None:
                deactivateSession = LidarPointSession.deactivateSession(latestSession)
                if deactivateSession:
                    postRequestForLidarStartOrStop(droneId, droneName, latestSession.id, lidarCommand)
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


class ControlDeviceDataAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    def control_device_save_data_to_db(self):
        try:
            ControlDevice.objects.create(
                drone=self["drone"],
                cpu_usage=self["cpu_usage"],
                cpu_core_usage=self["cpu_core_usage"],
                cpu_core_frequency=self["cpu_core_frequency"],
                cpu_temp=self["cpu_temp"],
                cpu_fan_RPM=self["cpu_fan_RPM"],
                gpu_usage=self["gpu_usage"],
                gpu_frequency=self["gpu_frequency"],
                gpu_temp=self["gpu_temp"],
                ram_usage=self["ram_usage"],
                swap_usage=self["swap_usage"],
                swap_cache=self["swap_cache"],
                emc_usage=self["emc_usage"],
            )
        except Exception as e:
            logger.error(f'Control Device {self["drone"].drone_name} Serializer data are not valid. Error: {e}.')


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


class MissionPointsListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = MissionPoint.objects.all()
    serializer_class = MissionPointSerializer

    def list(self, request, *args, **kwargs):
        """
        Overriding the default method. We want a special use case here. We want to list
        the mission points for a particular mission for which the specified drone is part od
        Args:
            request:
            *args:
            **kwargs:
        Returns:
        """
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        # Get the mission points for the mission that this drone is currently participating
        qs = Drone.objects.filter(drone_name=drone_name, operation=Operation.objects.get(operation_name=operation_name))
        drone = get_object_or_404(qs)
        mission = drone.mission
        if not mission:
            raise Http404("This drone is not in any active missions at the moment")

        mission_points = mission.mission_points.all()
        queryset = self.filter_queryset(mission_points)

        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return Response(serializer.data)


class UserList(LoginRequiredMixin, generics.ListAPIView):
    queryset = get_user_model().objects.all()
    serializer_class = UserSerializer

    def get(self, request, *args, **kwargs):
        users = User.objects.exclude(username="AnonymousUser")
        return render(request, "aiders/users.html", {"users": users})


class DroneList(LoginRequiredMixin, generics.ListAPIView):
    queryset = Drone.objects.all()
    serializer_class = DroneSerializer

    def get(self, request, *args, **kwargs):
        drones = Drone.objects.all()
        return render(request, "aiders/drones.html", {"drones": drones})

    def save_drone_to_db(self):
        serializer = DroneSerializer(data=self)
        if serializer.is_valid():
            drone = serializer.save()
            logger.info(f"Drone Serializer id {drone.pk} is saved.")
        else:
            logger.error(f"Drone Serializer data are not valid. Error: {serializer.errors}.")


class DeviceList(LoginRequiredMixin, generics.ListAPIView):
    queryset = Device.objects.all()
    serializer_class = DeviceSerializer

    def get(self, request, *args, **kwargs):
        devices = Device.objects.all()
        return render(request, "aiders/devices.html", {"devices": devices})

    def save_device_to_db(self):
        serializer = DeviceSerializer(data=self)
        if serializer.is_valid():
            device = serializer.save()
            logger.info(f"Device Serializer id {device.pk} is saved.")
        else:
            logger.error(f"Device Serializer data are not valid. Error: {serializer.errors}.")


class BaloraList(LoginRequiredMixin, generics.ListAPIView):
    queryset = BaloraMaster.objects.all()
    serializer_class = LoraSerializer

    def get(self, request, *args, **kwargs):
        loras = BaloraMaster.objects.all()
        return render(request, "aiders/balora.html", {"loras": loras})

    def save_lora_to_db(self):
        serializer = LoraSerializer(data=self)
        if serializer.is_valid():
            balora = serializer.save()
        else:
            logger.error(f"Balora Serializer data are not valid. Error: {serializer.errors}.")

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
        return render(request, "aiders/manage_operations.html", {"operations": operations, "users": users, "use_online_maps": False})


# class JoinOperationView(LoginRequiredMixin,View):
#     def get(self, request, *args, **kwargs):
#         operation_id = self.kwargs.get("operation_id")
#         operation = Operation.objects.get(pk=operation_id)
#         return render(request, 'aiders/join_operation.html', {'operation': operation})


class ManagePermissionsView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.edit_permissions"):
            raise PermissionDenied("You do not have permission to read the permissions.")
        users = User.objects.exclude(username="AnonymousUser")
        for user in users:
            self.add_user_perm(user)
        operation_groups = ""
        all_groups = Group.objects.all()
        for group in all_groups:
            if str(group.name).__contains__(" operation join"):
                operation_groups = operation_groups + (group.name).replace(" operation join", "") + ","
        return render(request, "aiders/manage_permissions.html", {"users": users, "all_groups": operation_groups})

    def post(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.edit_permissions"):
            raise PermissionDenied("You do not have permission to change the permissions.")
        for user in User.objects.exclude(username="AnonymousUser"):
            User.update_permissions(user.id, "permission_edit_permissions", str(user.id) in request.POST.getlist("permission_edit_permissions"))
            User.update_permissions(user.id, "permission_create_operations", str(user.id) in request.POST.getlist("permission_create_operations"))
        users = User.objects.exclude(username="AnonymousUser")

        for user in users:
            self.add_user_perm(user)
        operation_groups = ""
        all_groups = Group.objects.all()
        for group in all_groups:
            if str(group.name).__contains__(" operation join"):
                operation_groups = operation_groups + (group.name).replace(" operation join", "") + ","
        return render(request, "aiders/manage_permissions.html", {"users": users, "all_groups": operation_groups}, status=status.HTTP_202_ACCEPTED)

    def add_user_perm(self, user):
        user.permission_edit_permissions = user.has_perm("aiders.edit_permissions")
        user.permission_create_operations = user.has_perm("aiders.create_operations")
        user.save()


class ManageUserPermissionsView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        if not request.user.has_perm("aiders.edit_permissions"):
            raise PermissionDenied("You do not have permission to change the permissions.")
        user_name = self.kwargs.get("user_name")
        group_list = request.POST.get("selected")
        group_list = group_list.split(",")
        for group in Group.objects.all():
            if str(group.name).__contains__(" operation join"):
                User.objects.filter(username=user_name)[0].groups.remove(group)
        for group_name in group_list:
            group_object = Group.objects.filter(name=f"{group_name} operation join").last()
            User.objects.filter(username=user_name)[0].groups.add(group_object)
        return HttpResponse(status=status.HTTP_200_OK)


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
                context = {"operation": joined_op_obj, "net_ip": os.environ.get("NET_IP", "localhost"), "ws_port": os.environ.get("WS_PORT", 8000), "version": os.environ.get("VERSION", 1)}
                if disasterEpicenterGPS is not None:
                    context["disaster_epicenter_latitude"] = disasterEpicenterGPS['latitude']
                    context["disaster_epicenter_longitude"] = disasterEpicenterGPS['longitude']
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
        context["use_online_map"] = use_online_map
        return render(request, "aiders/platform.html", context)

    return render(request, "aiders/login.html", context)


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
        operationName = self.kwargs.get("operation_name")
        drone_name = request.POST.get("drone_name")
        activateBuildMap = request.POST.get("start_build_map_boolean")
        overlap = request.POST.get("overlap")
        drone = Drone.objects.get(drone_name=drone_name)
        if activateBuildMap == "true":
            build_map_request_handler.PostRequestForBuildMapStartOrStop(drone_name, "START", overlap)
            latestActiveBuildMapSession = BuildMapSession.getLatestActiveSessionIdByDroneId(drone.id)
            drone.build_map_activated = True
            drone.save()
            if latestActiveBuildMapSession:
                print("Already active build map session", flush=True)
                BuildMapSessionId = latestActiveBuildMapSession.id
            else:
                print("Active build map session", flush=True)
                BuildMapSessionId = BuildMapSession.createBuildMapSessionByUserIdOperationNameDrone(User.objects.get(id = request.user.id), operationName, drone)
            logger.info("User sending build map request Start for drone {}.".format(drone_name))
            return JsonResponse({'data': BuildMapSessionId}, status=200)
        else:
            drone.build_map_activated = False
            drone.save()
            BuildMapSession.objects.filter(
                operation=Operation.objects.get(operation_name=operationName), drone=drone, is_active=True
            ).update(end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ), is_active=False)
            # BuildMapSessionId = BuildMapSession.deactivateBuildMapSessionByOperationNameDrone(operationName, drone)
            build_map_request_handler.PostRequestForBuildMapStartOrStop(drone_name, "STOP", overlap)
            logger.info("User sending build map request Stop for drone {}.".format(drone_name))
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
        print(request.body)
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
                    d_roll=None,
                    d_pitch=None,
                    d_yaw=None,
                    g_roll=None,
                    g_pitch=None,
                    g_yaw=None,
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
            device_name = request.POST.get("deviceName")
            session = DeviceSession.objects.filter(
                is_active=True,
                device=Device.objects.get(name=device_name),
            ).last()

            if not os.path.exists(default_storage.path(session.folder_path)):
                os.mkdir(default_storage.path(session.folder_path))               

            img_file = request.FILES.get("image_file")
            img_name = request.POST.get("img_name")
            device_latitude = float(request.POST.get("latitude"))
            device_longitude = float(request.POST.get("longitude"))

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

                if not UserPreferences.objects.filter(user=user).exists():
                    UserPreferences.objects.create(use_online_map=True, user=user)

                login(request, user, backend="django.contrib.auth.backends.ModelBackend")

                if redirect_to != "None":
                    return HttpResponseRedirect(redirect_to)
                return redirect("manage_operations")
        else:
            messages.error(request, "Wrong username or password!")
            return render(request, "aiders/login.html", {"auth_form": AuthenticationForm, "next": redirect_to})


def logout_view(request):
    logout(request)
    # Redirect to a success page
    return redirect("login")


def new_operation_form_view(request):
    if not request.user.has_perm("aiders.create_operations"):
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
    # Sinadra
    if(request.POST.get("dense_area_of_buildings")) == "true":
        dense_area_of_buildings_value= True
    elif (request.POST.get("dense_area_of_buildings")) == "false":
        dense_area_of_buildings_value= False
    else:
        dense_area_of_buildings_value= None

    if(request.POST.get("risk_of_explosion_and_fire")) == "true":
        risk_of_explosion_and_fire_value= True
    elif (request.POST.get("risk_of_explosion_and_fire")) == "false":
        risk_of_explosion_and_fire_value= False
    else:
        risk_of_explosion_and_fire_value= None
    if request.POST.get("disaster_epicenter_latitude") == '':
        disasterEpicenterLatitudeValue = None
    else:
        disasterEpicenterLatitudeValue = request.POST.get("disaster_epicenter_latitude")
    if request.POST.get("disaster_epicenter_longtitude") == '':
        disasterEpicenterLongtitudeValue = None
    else:
        disasterEpicenterLongtitudeValue = request.POST.get("disaster_epicenter_longtitude")
    if request.POST.get("max_extreme_temperature") == '':
        maxExtremeTemperatureValue = None
    else:
        maxExtremeTemperatureValue = request.POST.get("max_extreme_temperature")

    operation_instance = Operation.objects.create(
        operation_name=request.POST.get("operation_name"),
        location=request.POST.get("location"),
        description=request.POST.get("description"),
        operator=request.user,

        # Sinadra
        disaster_epicenter_latitude = disasterEpicenterLatitudeValue,
        disaster_epicenter_longtitude = disasterEpicenterLongtitudeValue,
        dense_area_of_buildings = dense_area_of_buildings_value,
        max_extreme_temperature = maxExtremeTemperatureValue,
        risk_of_explosion_and_fire = risk_of_explosion_and_fire_value,
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
    # Save Permissions
    group_join_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation join")
    group_edit_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation edit")
    assign_perm("join_operation", group_join_operation, operation_instance)
    assign_perm("edit_operation", group_edit_operation, operation_instance)

    for user_id in request.POST.getlist("users_allow"):
        User.objects.filter(pk=user_id)[0].groups.add(group_join_operation)

    logger.info(f"Operation with id {operation_instance.pk} is created successfully.")
    return redirect("manage_operations")


def edit_operation_form_view(request, operation_name):
    if not request.user.has_perm("aiders.create_operations"):
        raise PermissionDenied("You do not have permission to create the operation.")
    if request.method == "POST":
        if operation_name == request.POST.get("operation_name"):
            return edit_operation_form_save(operation_name, request)
    else:
        operation_instance = Operation.objects.get(operation_name=operation_name)

    users_all = []
    users_allow = []
    for user in User.objects.all():
        if user.username != "AnonymousUser":
            if user.has_perm("join_operation", operation_instance):
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
        
    # Sinadra
    if(request.POST.get("dense_area_of_buildings")) == "true":
        dense_area_of_buildings_value= True
    elif (request.POST.get("dense_area_of_buildings")) == "false":
        dense_area_of_buildings_value= False
    else:
        dense_area_of_buildings_value= None

    if(request.POST.get("risk_of_explosion_and_fire")) == "true":
        risk_of_explosion_and_fire_value= True
    elif (request.POST.get("risk_of_explosion_and_fire")) == "false":
        risk_of_explosion_and_fire_value= False
    else:
        risk_of_explosion_and_fire_value= None
    if request.POST.get("disaster_epicenter_latitude") == '':
        disasterEpicenterLatitudeValue = None
    else:
        disasterEpicenterLatitudeValue = request.POST.get("disaster_epicenter_latitude")
    if request.POST.get("disaster_epicenter_longtitude") == '':
        disasterEpicenterLongtitudeValue = None
    else:
        disasterEpicenterLongtitudeValue = request.POST.get("disaster_epicenter_longtitude")
    if request.POST.get("max_extreme_temperature") == '':
        maxExtremeTemperatureValue = None
    else:
        maxExtremeTemperatureValue = request.POST.get("max_extreme_temperature")

    operation_instance.disaster_epicenter_latitude = disasterEpicenterLatitudeValue
    operation_instance.disaster_epicenter_longtitude = disasterEpicenterLongtitudeValue
    operation_instance.dense_area_of_buildings = dense_area_of_buildings_value
    operation_instance.max_extreme_temperature = maxExtremeTemperatureValue
    operation_instance.risk_of_explosion_and_fire = risk_of_explosion_and_fire_value

    operation_instance.save()
    Group.objects.get(name=f"{operation_instance.operation_name} operation join").delete()
    group_join_operation = Group.objects.create(name=f"{operation_instance.operation_name} operation join")
    assign_perm("join_operation", group_join_operation, operation_instance)

    for user_id in request.POST.getlist("users_allow"):
        User.objects.filter(pk=user_id)[0].groups.add(group_join_operation)
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


class ExecuteAlgorithmAPIView(LoginRequiredMixin, APIView):
    def post(self, request, *args, **kwargs):
        return Response(
            utils.handleAlgorithmExecution(
                Operation.objects.get(operation_name=kwargs["operation_name"]).pk,
                request.data["input"],
                request.data["canBeLoadedOnMap"],
                request.data["algorithmName"],
                request.user.pk,
            )
        )


class ExecuteMissionAPIView(LoginRequiredMixin, APIView):
    def get(self, request, *args, **kwargs):
        operation_name = kwargs["operation_name"]
        drone_name = kwargs["drone_name"]
        user = request.user
        operation = Operation.objects.get(operation_name=operation_name)
        drone = Drone.objects.get(drone_name=drone_name)

        mission_log = MissionLog.objects.filter(action="START_MISSION", user=user.pk, drone=drone, operation=operation).last()
        return Response(mission_log.mission.mission_type)

    def post(self, request, *args, **kwargs):
        # print("Request of the Execute Mission:", request, "\nand kwargs:", kwargs)
        operation_name = kwargs["operation_name"]
        drone_name = kwargs["drone_name"]
        actionDetails = request.data
        user_name = request.user.username
        operation = Operation.objects.get(operation_name=operation_name)

        User = get_user_model()
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
            request.user.pk,
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

    # def get(self, request, *args, **kwargs):
    #     context = self.get_context_data()
    #     return self.render_to_response(context)
    #
    #     # self.object = self.get_object()
    #     # context = self.get_context_data(object=self.object)
    #     # return self.render_to_response(context)

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))

        if not self.request.user.has_perm("join_operation", Operation.objects.filter(operation_name=self.kwargs.get("operation_name"))[0]):
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
    if request.method == "GET":
        opQuery = Operation.objects.filter(operation_name=operation_name)

        if opQuery.exists():
            operation = get_object_or_404(opQuery)

            if operation.active:
                operation.active = False
                operation.save()
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
    if not request.user.has_perm("join_operation", Operation.objects.filter(operation_name=operation_name)[0]):
        raise PermissionDenied("You do not have permission to join the operation.")
    if request.method == "POST":
        opQuery = Operation.objects.filter(operation_name=operation_name)
        if opQuery.exists():
            operation = get_object_or_404(opQuery)
            if operation.active:
                User.objects.filter(pk=request.user.id).update(joined_operation=operation)
                # get_object_or_404(user_query)
                return redirect("home")
            else:
                raise Http404("Operation Not Found")
        else:
            raise Http404("Operation Not Found")
    return JsonResponse({"success": False})


@csrf_protect
def register_request(request):
    if request.method == "POST":
        form = NewUserForm(request.POST)
        if form.is_valid():
            user = form.save()
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


@api_view(["GET"])
def detection_types_api_view(request, operation_name):
    if request.method == "GET":
        from logic.algorithms.object_detection.src.models.label import \
            get_labels_all

        return Response({"detection_types": list(get_labels_all())})

    return Response(status=status.HTTP_400_BAD_REQUEST)


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

        if not self.request.user.has_perm("join_operation", Operation.objects.filter(operation_name=self.kwargs.get("operation_name"))[0]):
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


class ballisticActivatedAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        drone_name = request.POST.get("drone_id")
        operation_name = kwargs.get("operation_name")
        if Drone.objects.get(drone_name=drone_name).ballistic_available:
            try:
                # ballistic.publish_message(drone_name, 1)
                Ballistic.objects.create(
                    drone=Drone.objects.get(drone_name=drone_name),
                    operation=Operation.objects.get(operation_name=operation_name),
                    user=User.objects.get(pk=request.user.pk),
                    telemetry=Telemetry.objects.filter(drone=Drone.objects.get(drone_name=drone_name)).last(),
                )
                logger.info(f"Ballistic activated for drone {drone_name}.")
                return HttpResponse("Sending message to drone.", status=status.HTTP_200_OK)
            except Exception as e:
                logger.error(f"Ballistic encounter an error for drone {drone_name}. Error: {e}")
        return HttpResponse(
            f"Ballistic encounter an error for drone {drone_name}.",
            status=status.HTTP_200_OK,
        )


class BaloraPM25APIView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        return JsonResponse(
            list(
                BaloraTelemetry.objects.filter(
                    operation__operation_name=kwargs.get("operation_name"), baloraMaster__is_connected_with_platform=True
                ).values("pm25", "latitude", "longitude")
            ),
            safe=False,
        )


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
    if request.method == 'POST':
        data = json.loads(request.body)
        lidarSessionId = data.get("sessionId")
        if utils.threadStarted("processPointCloudDataSession"+lidarSessionId):
            return JsonResponse({"message": "Session "+lidarSessionId+" is already processing."})
        thread = threading.Thread(target=processPointsByOpen3d, args=(lidarSessionId,))
        thread.name = "processPointCloudDataSession"+lidarSessionId
        thread.start()
        return JsonResponse({"message": "Session " + lidarSessionId + " is starting processing."})

def processPointsByOpen3d(_lidarSessionId):
    # Process Points
    lidarListOfPoints = LidarPoint.getAllLidarPointsBySessionId(_lidarSessionId)
    pointsValues = [(d['x'], d['y'], d['z']) for d in lidarListOfPoints]
    pointsColors = [(d['red']/255, d['green']/255, d['blue']/255) for d in lidarListOfPoints]  # normalize color values

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointsValues)
    pcd.colors = o3d.utility.Vector3dVector(pointsColors)
    alpha = 0.08
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha, tetra_mesh, pt_map)

    # For Visualization
    # Create a coordinate frame (which includes arrows)
    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])

    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()

    directory = default_storage.path("lidarPointCloudMesh")
    if not os.path.exists(directory):
        os.makedirs(directory) 
    fullPath = directory+f"/lidarSession_{str(_lidarSessionId)}.glb"
    if os.path.exists(fullPath):
        os.remove(fullPath)
    o3d.io.write_triangle_mesh(fullPath, mesh)
    LidarPointSession.updateProcessedSessionBySessionIdAndPath(_lidarSessionId, f"lidarPointCloudMesh/lidarSession_{str(_lidarSessionId)}.glb")

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


##############################################################
######################### MAVLINK ############################
##############################################################


class MavlinkForm(forms.Form):
    name = forms.CharField(max_length=255, help_text="A unique identifier for the UAV", widget=forms.TextInput(attrs={'size': '14', 'class': 'form-control'}))
    model = forms.CharField(max_length=255, widget=forms.TextInput(attrs={'size': '14', 'class': 'form-control'}))
    ip = forms.CharField(max_length=255, required=False, label="IP Address", widget=forms.TextInput(attrs={'size': '11', 'class': 'form-control'}))
    port = forms.CharField(max_length=255, help_text="Default MAVLink port is 14550", widget=forms.TextInput(attrs={'size': '5', 'class': 'form-control'}))
    live_stream_url = forms.CharField(max_length=255, help_text="RTMP or RTSP", widget=forms.TextInput(attrs={'size': '30', 'class': 'form-control'}))
    # operation = forms.CharField(widget=forms.Textarea)


def mavlinkAddFormView(request):
    if request.method == 'POST':
        form = MavlinkForm(request.POST)
        if form.is_valid():
            # Retrieve data from the form
            name = form.cleaned_data['name']
            model = form.cleaned_data['model']
            type = "MAVLINK"
            ip = form.cleaned_data['ip']
            port = form.cleaned_data['port']
            live_stream_url = form.cleaned_data['live_stream_url']
            drone = Drone(drone_name=name, model=model, type=type, ip=ip, port=port, live_stream_url=live_stream_url, camera_model="no_cam", is_connected_with_platform=False, time=datetime.datetime.now())
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
            drone.live_stream_url = form.cleaned_data['live_stream_url']
            drone.save()

            return redirect('drones_list')
    else:
        # populate the form with the existing data
        form = MavlinkForm(initial={'name': drone.drone_name, 'model': drone.model, 'ip': drone.ip, 'port': drone.port, 'live_stream_url': drone.live_stream_url})

    return render(request, 'aiders/mavlink-edit.html', {'form': form, 'drone': drone})


def mavlinkManageView(request, pk):
    drone = Drone.objects.get(id=pk)
    return render(request, 'aiders/mavlink-manage.html', {'drone': drone})


def mavlinkCheckConnection(request, pk):
    if request.method == 'POST':
        drone = Drone.objects.get(id=pk)
        return JsonResponse({'connected': drone.is_connected_with_platform}, status=200)
    return JsonResponse({'error': "not supported"}, status=500)


#####################
# MAVLINK API CALLS #
#####################


def mavlinkConnect(request):
    from .httpRequests import postRequestForMavlink
    if request.method == 'POST':
        data = json.loads(request.body)
        payload = {
            "name": data.get('name'),
            "ip": data.get('ip'),
            "port": data.get('port'),
            "model": data.get('model'),
            "operationId": data.get('operationId'),
        }
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
    # from scipy.spatial import ConvexHull
    from alpha_shapes import Alpha_Shaper
    from shapely.geometry import mapping    
    operation = Operation.objects.filter(operation_name=operation_name).last()

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
                droneTelemetry = list(Telemetry.objects.filter(drone_id=drone.id, operation_id=operation.id, time__gte=fromDatetimeObj, time__lt=toDatetimeObj)[0:100000].values('lat', 'lon', 'secondsOn', 'time'))
                previousSecondsOn = 0
                currentSecondsOn = 0
                # break down drone's telemetry into sessions based on the secondsOn field
                droneTelemetrySessions = []
                sessionPoints = []
                for t in droneTelemetry:
                    if(t['lat'] != 0 and t['lon'] != 0):
                        currentSecondsOn = t['secondsOn']
                        if abs(currentSecondsOn - previousSecondsOn) > 29:
                            if(len(sessionPoints)) > 0:
                                droneTelemetrySessions.append(sessionPoints)
                            sessionPoints = []
                        sessionPoints.append([t['lat'], t['lon'], t['time']])
                        previousSecondsOn = currentSecondsOn

                if(len(sessionPoints)) > 0:
                    droneTelemetrySessions.append(sessionPoints) # append the last session

                # # loop telemetry sessions and create polygons
                # for s in droneTelemetrySessions:
                #     droneTelemetryTuple = [(d[0], d[1]) for d in s]
                #     hull = ConvexHull(droneTelemetryTuple)
                #     dronePolygon = json.dumps([hull.points[i].tolist() for i in hull.vertices])
                #     dronePolygons.append(dronePolygon)
                #     droneData.append([drone.drone_name, s[0][2]]) # drone name and start time of session

                # loop telemetry sessions and create polygons
                for s in droneTelemetrySessions:
                    droneTelemetryTuple = [(d[0], d[1]) for d in s]
                    try:
                        shaper = Alpha_Shaper(droneTelemetryTuple)
                        alpha_shape = shaper.get_shape(alpha=5)
                        shapeMapping = mapping(alpha_shape)
                        for i in range(len(shapeMapping['coordinates'])):
                            dronePolygon = json.dumps(shapeMapping['coordinates'][i])
                            dronePolygons.append(dronePolygon)
                            droneData.append([drone.drone_name, s[0][2]])
                    except Exception as e:
                        print(e, flush=True)                

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



##################################
######### STREAM REPLAY ##########
##################################


def getAvailableStreams(request, *args, **kwargs):
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


def stream_replay(request, stream_type, session_id):
    print(stream_type, flush=True) 
    # TODO: check user permissions

    if stream_type == "raw":
        session = LiveStreamSession.objects.get(id=session_id)
        frames = list(RawFrame.objects.filter(live_stream_session_id=session_id).values('frame', 'time'))
    else:
        session = DetectionSession.objects.get(id=session_id)
        frames = list(DetectionFrame.objects.filter(detection_session_id=session_id).values('frame', 'time'))

    drone_id = session.drone_id
    drone = Drone.objects.get(id=drone_id)    

    # Convert datetime to string
    for frame in frames:
        frame['time'] = frame['time'].strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    # print(frames, flush=True)
    return render(request, 'aiders/stream_replay.html', {'frames': frames, 'drone': drone, 'session_id': session_id, 'stream_type': stream_type})



#############################################################################################################
# SafeML

class DetectionSafeMLStartOrStopAPIView(LoginRequiredMixin,APIView):
    def post(self, request, *args, **kwargs):
        data = request.data
        detectionStatus = data.get("detectionStatus")
        userId = request.user.pk
        droneName = data.get("droneName")
        droneId = Drone.getDroneIdByName(droneName)
        operationName = data.get("operationName")
        operationId = Operation.getOperationIdByName(operationName)
        detectionType = data.get("detectionType")
        if detectionStatus == 'Start':
            apiResponse = postDetectionStartToSafeML(userId, operationId, droneId, droneName, detectionType)
        elif detectionStatus == 'Stop':
            apiResponse = postDetectionStopToSafeML(operationId, droneId, droneName, detectionType)
        else:
            # Handle other cases or provide an error response
            return HttpResponse("Invalid detectionStatus", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse(apiResponse, status=status.HTTP_200_OK)
    

#############################################################################################################


def safeDronesResults(request):
    calculations_safe_drones.start()

    # print(datetime.datetime.now().strftime("%H:%M:%S")+" * SAFE DRONES RESULTS REQUESTED", flush=True)

    
    from django.db.models import Subquery, OuterRef

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

def sinadraStartOrStop(request):
    from .httpRequests import postRequestForSinadraStartOrStop
    if request.method == 'POST':
        data = json.loads(request.body)
        operationId = data.get('operationId')
        command = data.get('command')
        result=postRequestForSinadraStartOrStop(operationId, command)
        return HttpResponse(result)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)

def sinadraResults(request):
    if request.method == 'POST':
        data = json.loads(request.body)
        operationId = data.get('operationId')
        operation = Operation.objects.get(pk=operationId)
        if not operation.sinadra_active:
            return JsonResponse({'error': 'Operation is not active.'}, status=400)
        try:
            latestData = SinadraData.objects.filter(operation_id=operationId).latest('time')
        except Exception:
            return JsonResponse({'error': 'No data found.'}, status=400)
        responseData = {
            'operation_name': operation.id,
            'disaster_epicenter_latitude': operation.disaster_epicenter_latitude,
            'disaster_epicenter_longtitude': operation.disaster_epicenter_longtitude,
            'dense_area_of_buildings': operation.dense_area_of_buildings,
            'max_extreme_temperature': operation.max_extreme_temperature,
            'risk_of_explosion_and_fire': operation.risk_of_explosion_and_fire,
            'latest_human_injury_risk_prediction': latestData.human_injury_risk_prediction,
            'time': latestData.time.strftime('%Y-%m-%dT%H:%M:%S')
        }

        # get the scue value of SafeML (if available)
        try:
            latest_safeml_output = SafemlOutput.objects.filter(
                detection_session__is_active=True,
                detection_session__operation_id=operationId
            ).latest('time')
            responseData["safeml_scue"] = latest_safeml_output.scue
        except Exception as e:
            responseData["safeml_scue"] = None

        # print(responseData, flush=True)
        return JsonResponse(responseData)
    else:
        return JsonResponse({'message': 'Invalid request method. Only POST requests are accepted.'}, status=400)


from django.db.models import Avg, Max, Min


# Delete later
class TestingBuildMap(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        # write data and save lidar to db
        # with open("lidarPoints.txt", "r") as file:
        #     self = file.read()
        #     lidar_session = LidarPointSession.objects.all().last()
        #     drone_name = "drone1"
        #     batch_size = 5000
        #     batch_list = []
        #     telemetry = Telemetry.objects.filter(drone__drone_name=drone_name).last()

        #     for single_point in self.split("|"):
        #         single_point_list = single_point.split(",")
        #         if len(single_point_list) < 6:  # Check if single_point_list has enough elements
        #             continue  # Skip this point if it doesn't
        #         point = LidarPoint(
        #             x=single_point_list[2],
        #             y=single_point_list[0],
        #             z=single_point_list[1],
        #             r=single_point_list[3],
        #             g=single_point_list[4],
        #             b=single_point_list[5],
        #             lidar_point_session=lidar_session,
        #             telemetry=telemetry,
        #         )
        #         batch_list.append(point)
        #         if len(batch_list) >= batch_size:
        #             LidarPoint.objects.bulk_create(batch_list)
        #             batch_list.clear()
        #     if batch_list:
        #         LidarPoint.objects.bulk_create(batch_list)
        # with open("live_custom_data_lidar_tel.csv", "r") as csvfile:
        #     print("before")
        #     reader = csv.reader(csvfile)
        #     rows = list(reader)
        #     print("after")
        #     Drone.objects.filter(pk=1).update(is_connected_with_platform=True)
        #     lidar_session = LidarPointSession.objects.create(
        #         user=User.objects.all().last(), operation=Operation.objects.all().last(), drone=Drone.objects.all().last()
        #     )
        #     print("after Lidar")
        #     batch_list = []
        #     for row in rows:
        #         if len(row) == 18:
        #             if batch_list is not None:
        #                 LidarPoint.objects.bulk_create(batch_list)
        #             Telemetry.objects.create(
        #                 drone=Drone.objects.all().last(),
        #                 battery_percentage=row[2],
        #                 gps_signal=row[3],
        #                 satellites=row[4],
        #                 heading=row[5],
        #                 velocity=row[6],
        #                 homeLat=row[7],
        #                 homeLon=row[8],
        #                 lat=row[9],
        #                 lon=row[10],
        #                 alt=row[11],
        #                 drone_state=row[12],
        #                 secondsOn=row[13],
        #                 gimbal_angle=row[14],
        #                 water_sampler_in_water=row[15],
        #                 operation=Operation.objects.all().last(),
        #                 mission_log=None,
        #             )
        #             batch_list = []
        #         else:
        #             point = LidarPoint(
        #                 x=float(row[1]),
        #                 y=float(row[2]),
        #                 z=float(row[3]),
        #                 intensity=float(row[4]),
        #                 red=float(row[5]),
        #                 green=float(row[6]),
        #                 blue=float(row[7]),
        #                 lidar_point_session=lidar_session,
        #                 telemetry=Telemetry.objects.get(pk=row[8]),
        #             )
        #             batch_list.append(point)
        #             if len(batch_list) > 50000:
        #                 LidarPoint.objects.bulk_create(batch_list)
        #                 batch_list = []
        # with open("droneTelemetry_17_5.json", "r") as json_file:
        #     data = json.load(json_file)
        # for d in data:
        #     Telemetry.objects.create(
        #         drone=Drone.objects.all()[0],
        #         lat=d["fields"]["lat"],
        #         lon=d["fields"]["lon"],
        #         alt=d["fields"]["alt"],
        #         heading=d["fields"]["heading"],
        #         battery_percentage=d["fields"]["battery_percentage"],
        #         gps_signal=d["fields"]["gps_signal"],
        #         satellites=d["fields"]["satellites"],
        #         velocity=d["fields"]["velocity"],
        #         homeLat=d["fields"]["homeLat"],
        #         homeLon=d["fields"]["homeLon"],
        #         drone_state=d["fields"]["drone_state"],
        #         secondsOn=d["fields"]["secondsOn"],
        #         gimbal_angle=d["fields"]["gimbal_angle"],
        #     )
        # my_list = []
        # with open("B3_receiver.csv", "r") as file:
        #     csv_reader = csv.reader(file)
        #     next(csv_reader)
        #     for row in csv_reader:
        #         BaloraTelemetry.objects.create(
        #             baloraMaster=BaloraMaster.objects.get(name="BALORA_MASTER"),
        #             balora=Balora.objects.get(name="balora_1"),
        #             latitude=row[3],
        #             longitude=row[4],
        #             pm1=random.randint(1, 12),
        #             pm25=random.randint(1, 250),
        #             received_signal_strength_indication=row[6],
        #             SignalToNoiseRatio=row[7],
        #             operation=Operation.objects.all().last(),
        #             secondsOn=0,
        #         )
        #         my_list.append([row[3], row[4]])
        #         time.sleep(0.1)
        # with open("B4_receiver.csv", "r") as file:
        #     csv_reader = csv.reader(file)
        #     next(csv_reader)
        #     for row in csv_reader:
        #         BaloraTelemetry.objects.create(
        #             balora=BaloraMaster.objects.get(name="Balora2"),
        #             baloraNetwork=Balora.objects.get(name=row[2]),
        #             latitude=row[3],
        #             longitude=row[4],
        #             pm1=random.randint(1, 150),
        #             pm25=random.randint(1, 150),
        #             received_signal_strength_indication=row[6],
        #             SignalToNoiseRatio=row[7],
        #             operation=Operation.objects.all().last(),
        #             secondsOn=0,
        #         )
        #         my_list.append([row[3], row[4]])
        #         time.sleep(0.1)
        return render(
            request,
            "aiders/testing.html",
        )

@method_decorator(csrf_exempt, name='dispatch')
class DetectedObjectDescriptionSetPIView(LoginRequiredMixin, View):

    def post(self, request, *args, **kwargs):

        operation_name = self.kwargs.get("operation_name")

        operation = Operation.objects.get(operation_name=self.kwargs.get("operation_name"))
        
        track_id = self.kwargs.get("track_id")

        #DetectedObject = DetectedObject.objects.get(operation_name=operation_name,)
        
        user = request.user

        # TODO: check if user has permission in this operation and if is allowed to change set the detected object description 

        jdata = json.loads(request.body)

        #logger.info( jdata )

        # check if the detected object already has a description
        # Detected_Object_Description = DetectedObjectDescription.objects.filter(track_id=track_id).first()
        
        # if Detected_Object_Description :
        #     serializer = DetectedObjectDescriptionSerializer( Detected_Object_Description, data=jdata)
        # else: 
        
        # a new object will be created
        serializer = DetectedObjectDescriptionSerializer( data=jdata)

        if serializer.is_valid() : 

            Result_Detected_Object_Description = serializer.save( updated_by = request.user )

            #logger.info(Result_Detected_Object_Description)

            DetectedObjectDescription_data = DetectedObjectDescriptionSerializer(Result_Detected_Object_Description).data

            #TODO:do the following in the serializer
            DetectedObjectDescription_data['updated_by_username'] = request.user.username 

            return HttpResponse(    json.dumps({ "DetectedObjectDescription" :  DetectedObjectDescription_data }) , 
                                    content_type='application/json',
                                    status=status.HTTP_201_CREATED)

        return HttpResponse(    json.dumps( serializer.errors ), 
                                content_type='application/json', 
                                status=status.HTTP_400_BAD_REQUEST ) 




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

