# -----------------------------------------------------------
# Creates the views for the database.
# This views are called when user navigates to a certain url.
# They are responsible for either rendering an HTML template or the API data that are requested
# For example: Navigating to the url 'api/operations/' will trigger the OperationListCreateAPIView class
# Reference: https://docs.djangoproject.com/en/4.0/topics/class-based-views/
# -----------------------------------------------------------
import csv
import decimal
import json
import logging
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
import pandas as pd
import pytz
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
from django.db.models import Avg
from django.forms.models import model_to_dict
from django.http import (FileResponse, Http404, HttpResponse,
                         HttpResponseNotFound, HttpResponseRedirect,
                         JsonResponse)
from django.shortcuts import (get_list_or_404, get_object_or_404, redirect,
                              render)
from django.urls import resolve, reverse, reverse_lazy
from django.views import View, generic
from django.views.decorators.csrf import csrf_exempt, csrf_protect
from formtools.wizard.views import SessionWizardView
from guardian.shortcuts import assign_perm
from json2html import *
from logic import utils
from logic.algorithms.ballistic import ballistic
from logic.algorithms.build_map import (build_map_request_handler,
                                        img_georeference)
from logic.algorithms.flying_report import flying_report
from logic.algorithms.lidar_point_cloud import lidar_points
from logic.algorithms.mission import mission_request_handler
from logic.algorithms.range_detection import range_detection
from logic.algorithms.water_collector import water_collector
from logic.algorithms.weather_station import (weather_station_ros_publisher,
                                              weather_station_ros_subscriber)
from logic.Constants import Constants
from PIL import Image
from rest_framework import generics, permissions, status
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework.views import APIView

from .factories import *
from .forms import FlyingReportForm, JoinOperationForm, NewUserForm
from .models import Operation
from .permissions import IsOwnerOrReadOnly
from .serializers import *

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

# SAASAS


class DatabaseFiller(APIView):
    '''
    A class that populates the database with dummy data.
    It utilizes the Factory notion, using the Factory Boy library
    Reference: https://factoryboy.readthedocs.io/en/stable/orms.html
    '''

    def get(self, request):
        UserFactory.create_batch(20)
        UserLogFactory.create_batch(20)
        OperationFactory.create_batch(20)
        mission_points = MissionPointFactory.create_batch(10)
        MissionFactory.create_batch(20, mission_points=tuple(mission_points))
        mission = Mission.objects.all().first()
        drones = DroneFactory.create_batch(20)
        DroneToOperationLogFactory.create_batch(20)
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
        LoraTransmitterFactory.create_batch(20)
        LoraTransmitterLocationFactory.create_batch(20)
        LidarPointSessionFactory.create_batch(20)
        LidarPointFactory.create_batch(20)
        BuildMapImageFactory.create_batch(50)
        BuildMapSessionFactory.create_batch(20)
        ControlDeviceFactory.create_batch(20)
        MissionLogFactory.create_batch(20)
        return redirect('login')


class OperationListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    '''
    List all operations or create new one. The get and create methods are inherited,
    using the generics.ListCreateAPIView.
    Tutorial Reference: https://www.django-rest-framework.org/tutorial/3-class-based-views/
    '''

    queryset = Operation.objects.all()
    serializer_class = OperationSerializer

    '''
     Ensure that authenticated requests get read-write access, and unauthenticated requests get read-only access
    '''
    permission_classes = [permissions.IsAuthenticatedOrReadOnly,
                          IsOwnerOrReadOnly]

    def perform_create(self, serializer):
        '''
           Allows us to modify how the instance save is managed,
           and handle any information that is implicit in the incoming request or requested URL.
       '''
        serializer.save(
            operator=self.request.user)  # Operations are associated with the user that created them


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
    lookup_field = 'drone_name'

    def get_object(self):
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        obj = Drone.objects.get(drone_name=drone_name)
        if obj is None:
            raise Http404
        return obj

    def patch(self, request, *args, **kwargs):
        '''
        Partially update the attributes of a drone.
        This is useful for example in case the drone is connected/disconnected from the platform, we update (patch)
        the "is_drone_active" field to true/false. OR we can update its DroneDetection field
        '''
        operation_name = self.kwargs.get("operation_name")
        operation_obj = Operation.objects.filter(
            operator=request.user, active=True)
        drone_name = self.kwargs.get("drone_name")
        qs = Drone.objects.filter(
            name=drone_name, operation__operation_name=operation_name)
        obj = get_object_or_404(qs)
        serializer = DroneSerializer(
            obj, data=json.loads(request.body), partial=True)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data)


class DetectionRetrieveAPIView(LoginRequiredMixin, generics.RetrieveUpdateDestroyAPIView):
    """
    Retrieve, update (patch) or delete a detection drone instance
    """
    queryset = Drone.objects.all()
    serializer_class = DetectionDroneSerializer
    lookup_field = 'drone_name'

    def patch(self, request, *args, **kwargs):
        '''
        Partially update the attributes of a detection drone.
        This is useful when we just want to change the detection status of the drone
        '''
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")
        qs = Detection.objects.filter(
            name=drone_name, operation__operation_name=operation_name)
        obj = get_object_or_404(qs)
        serializer = DetectionSerializer(
            obj, data=json.loads(request.body), partial=True)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data)


class MissionListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = Mission.objects.all()
    serializer_class = MissionSerializer

    def mission_save_to_db(missionObj, dronePK, userPK, operationPK):
        serializer = MissionSerializer(data=missionObj)
        if serializer.is_valid():
            createdMission = serializer.save()
            Drone.objects.filter(pk=dronePK).update(mission=createdMission.pk)
            logger.info('Mission with id {} is created successfully.'.format(
                createdMission.pk))
            MissionLoggerListCreateAPIView.mission_logger_save_to_db(
                'START_MISSION', createdMission, userPK, operationPK, dronePK)
            return True
        else:
            msg = 'Mission is not valid and is not created. Error: {}.'.format(
                serializer.errors)
            from .consumers import ErrorMsg
            ErrorMsg.set_message_and_error(logger, Drone.objects.get(
                pk=dronePK).operation.operation_name, msg)
            return False


class MissionLoggerListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = MissionLog.objects.all()
    serializer_class = MissionLoggerSerializer

    def mission_logger_save_to_db(action, mission, userPK, operationPK, dronePK):
        if Mission.objects.get(pk=mission.pk).mission_type == 'SEARCH_AND_RESCUE_MISSION':
            algorithm = Algorithm.objects.filter(
                algorithm_name='CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM', user=userPK, operation=operationPK).last()
            algorithmPK = algorithm.pk
        else:
            algorithmPK = None
        missionLoggerData = {
            'action': action,
            'mission': Mission.objects.get(pk=mission.pk).pk,
            'user': userPK,
            'operation': operationPK,
            'drone': dronePK,
            'algorithm': algorithmPK
        }
        serializerMissionLogger = MissionLoggerSerializer(
            data=missionLoggerData)
        if serializerMissionLogger.is_valid():
            createdMissionLogger = serializerMissionLogger.save()
            logger.info('Mission Logger is saved successfully.')
        else:
            msg = 'Mission Logger is not valid. Error: {}.'.format(
                serializerMissionLogger.errors)
            from .consumers import ErrorMsg
            ErrorMsg.set_message_and_error(logger, Drone.objects.get(
                pk=dronePK).operation.operation_name, msg)


class MissionRetrieveAPIView(LoginRequiredMixin, generic.ListView):
    model = MissionLog
    # fields = ('__all__')
    template_name = 'aiders/missions.html'
    queryset = MissionLog.objects.all()
    success_url = reverse_lazy('home')

    # def get(self, request, *args, **kwargs):
    #     context = self.get_context_data()
    #     return self.render_to_response(context)
    #
    #     # self.object = self.get_object()
    #     # context = self.get_context_data(object=self.object)
    #     # return self.render_to_response(context)

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(
            operation_name=self.kwargs.get('operation_name'))

        if not self.request.user.has_perm('join_operation', Operation.objects.filter(operation_name=self.kwargs.get('operation_name'))[0]):
            raise PermissionDenied(
                "You do not have permission to join the operation.")
        context = super(MissionRetrieveAPIView,
                        self).get_context_data(**kwargs)
        missions = list(MissionLog.objects.filter(
            action="START_MISSION", operation=operation))

        missionRemoveList = []
        for mission in missions:
            if not list(MissionLog.objects.filter(mission=mission.mission, action="FINISH_MISSION", operation=operation)):
                missionRemoveList.append(mission)

        for mission in missionRemoveList:
            missions.remove(mission)

        context['mission_results'] = missions
        context['operation_name'] = self.kwargs.get('operation_name')
        # Create any data and add it to the context
        return context

# working on Replay mission database to front end


class ReplayMissionOnlineAPIView(LoginRequiredMixin, View):
    def format_time(date, prev_date=0):
        edit_date = date.astimezone(pytz.timezone(
            settings.TIME_ZONE)).strftime("%Y-%m-%dT%H:%M:%S.%f")
        if prev_date == edit_date[:-4]+'Z':
            edit_date = edit_date[:-5]+str(int(edit_date[-6])+1)+'Z'
        else:
            edit_date = edit_date[:-4]+'Z'
        return edit_date

    def save_data(table, time_field_name, description, save_table):
        prev_date = 0
        time_field_name_edit = time_field_name
        for data in table:
            time_field_name = time_field_name_edit
            if isinstance(time_field_name, list):
                for time_field in time_field_name[1:]:
                    data[time_field] = ReplayMissionOnlineAPIView.format_time(
                        data[time_field])
                time_field_name = time_field_name[0]
            data[time_field_name] = ReplayMissionOnlineAPIView.format_time(
                data[time_field_name], prev_date)
            if data[time_field_name] in save_table:
                if description in save_table[data[time_field_name]]:
                    number = 1
                    while True:
                        if description+' '+str(number) in save_table[data[time_field_name]]:
                            number = number+1
                        else:
                            save_table[data[time_field_name]
                                       ][description+' '+str(number)] = data
                            break
                else:
                    save_table[data[time_field_name]][description] = data
            else:
                save_table[data[time_field_name]] = {}
                save_table[data[time_field_name]][description] = data
            prev_date = data[time_field_name]
        return save_table

    def edit_drone_data(drone_list):
        for drone in drone_list:
            drone['drone_name'] = Drone.objects.get(
                pk=drone['drone']).drone_name
        return drone_list

    def get(self, request, *args, **kwargs):
        replay_data = {}
        time_series_data = {}
        operation_name = self.kwargs.get('operation_name')
        mission_id = self.kwargs.get('mission_id')
        mission = Mission.objects.get(id=mission_id)
        Mission_start = MissionLog.objects.filter(
            mission=mission, action="START_MISSION")[0]
        Mission_end = MissionLog.objects.filter(
            mission=mission, action="FINISH_MISSION").last()
        replay_data.update({"start_time": ReplayMissionOnlineAPIView.format_time(
            Mission_start.executed_at), "end_time": ReplayMissionOnlineAPIView.format_time(Mission_end.executed_at)})
        DronesInOperation = list(Telemetry.objects.filter(operation=Operation.objects.get(operation_name=operation_name), received_at__range=(
            Mission_start.executed_at, Mission_end.executed_at)).values('drone').annotate(n=models.Count("pk")))
        TelemetryInOperation = list(Telemetry.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), received_at__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        BuildMapSessionInOperation = list(BuildMapSession.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), start_time__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        WeatherStationInOperation = list(WeatherStation.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), current_time__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        ErrorMessageInOperation = list(ErrorMessage.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), time__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        DetectionSessionInOperation = list(DetectionSession.objects.filter(
            operation=Operation.objects.get(operation_name=operation_name)).values())
        AlgorithmInOperation = list(Algorithm.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), executed_at__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        FrontEndUserInputInOperation = list(FrontEndUserInput.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), time__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
        Missions = list(Telemetry.objects.filter(operation=Operation.objects.get(operation_name=operation_name), received_at__range=(
            Mission_start.executed_at, Mission_end.executed_at)).values('mission_log').annotate(n=models.Count("mission_log__mission")))
        missionList = []
        for current_mission_data in Missions:
            if current_mission_data['mission_log'] != None:
                mission = MissionLog.objects.get(
                    pk=current_mission_data['mission_log']).mission
                mission_points = list(mission.mission_points.values())
                for mission_point in mission_points:
                    for field in mission_point:
                        if isinstance(mission_point[field], point.Point):
                            mission_point[field] = [float(mission_point[field].coords[0]), float(
                                mission_point[field].coords[1])]
                mission_object = Mission.objects.filter(
                    id=mission.pk).values().last()
                mission_object['mission_points'] = mission_points
                mission_object['executed_at'] = ReplayMissionOnlineAPIView.format_time(
                    mission_object['executed_at'])
                mission_object['dronePK'] = MissionLog.objects.get(
                    pk=current_mission_data['mission_log']).drone.pk
                missionList.append(mission_object)
        replay_data.update({"mission_data": missionList})
        replay_data.update(
            {"drone_available": ReplayMissionOnlineAPIView.edit_drone_data(DronesInOperation)})

        if TelemetryInOperation != []:
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                TelemetryInOperation, 'received_at', 'telemetry', time_series_data)
        if BuildMapSessionInOperation != []:
            all_images = []
            for session in BuildMapSessionInOperation:
                BuildMapImageInOperation = list(
                    BuildMapImage.objects.filter(session=session['id']).values())
                all_images = all_images+BuildMapImageInOperation
            for image in all_images:
                for field in image:
                    if isinstance(image[field], decimal.Decimal):
                        image[field] = float(image[field])
                    if isinstance(image[field], point.Point):
                        image[field] = [float(image[field].coords[0]), float(
                            image[field].coords[1])]
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                all_images, 'time', 'build_map_image', time_series_data)
        if DetectionSessionInOperation != []:
            for session in DetectionSessionInOperation:
                DetectionFrameInOperation = list(DetectionFrame.objects.filter(
                    detection_session=session['id'], saved_at__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
                for frame in DetectionFrameInOperation:
                    frame['drone_id'] = Drone.objects.get(
                        id=session['drone_id']).drone_name
                time_series_data = ReplayMissionOnlineAPIView.save_data(
                    DetectionFrameInOperation, 'saved_at', 'detection_frame', time_series_data)
                DetectionObjectsInOperation = list(DetectedObject.objects.filter(
                    detection_session=session['id'], detected_at__range=(Mission_start.executed_at, Mission_end.executed_at)).values())
                for objects in DetectionObjectsInOperation:
                    objects['drone_id'] = Drone.objects.get(
                        id=session['drone_id']).drone_name
                time_series_data = ReplayMissionOnlineAPIView.save_data(
                    DetectionObjectsInOperation, 'detected_at', 'detected_object', time_series_data)
        if WeatherStationInOperation != []:
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                WeatherStationInOperation, 'current_time', 'weather_station', time_series_data)
        if AlgorithmInOperation != []:
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                AlgorithmInOperation, 'executed_at', 'algorithm', time_series_data)
        if ErrorMessageInOperation != []:
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                ErrorMessageInOperation, 'time', 'error', time_series_data)
        if FrontEndUserInputInOperation != []:
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                FrontEndUserInputInOperation, 'time', 'user_input', time_series_data)
        for drone in DronesInOperation:
            RawFrameInOperation = list(RawFrame.objects.filter(
                live_stream_session__drone=Drone.objects.get(drone_name=drone['drone_name']), saved_at__range=(
                    Mission_start.executed_at, Mission_end.executed_at)).values())
            time_series_data = ReplayMissionOnlineAPIView.save_data(
                RawFrameInOperation, 'saved_at', 'video_frame', time_series_data)
        replay_data.update({"time_series_data": time_series_data})
        use_online_map = UserPreferences.objects.get(
            user=request.user).use_online_map
        return render(request, "aiders/replay_mission.html", {
            "replay_data": replay_data,
            "operation_name": operation_name,
            'operation': Operation.objects.get(operation_name=operation_name),
            'mission_drone': Mission_start.drone.drone_name,
            'use_online_map': use_online_map
        })


class TelemetryListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = Telemetry.objects.all().order_by('-received_at')[:10]
    serializer_class = TelemetrySerializer


class ControlDeviceDataAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    def control_device_save_data_to_db(jetsonObj):
        try:
            ControlDevice.objects.create(
                drone=jetsonObj['drone'],
                cpu_usage=jetsonObj['cpu_usage'],
                cpu_core_usage=jetsonObj['cpu_core_usage'],
                cpu_core_frequency=jetsonObj['cpu_core_frequency'],
                cpu_temp=jetsonObj['cpu_temp'],
                cpu_fan_RPM=jetsonObj['cpu_fan_RPM'],
                gpu_usage=jetsonObj['gpu_usage'],
                gpu_frequency=jetsonObj['gpu_frequency'],
                gpu_temp=jetsonObj['gpu_temp'],
                ram_usage=jetsonObj['ram_usage'],
                swap_usage=jetsonObj['swap_usage'],
                swap_cache=jetsonObj['swap_cache'],
                emc_usage=jetsonObj['emc_usage'],
            )
        except Exception as e:
            logger.error('Control Device {} Serializer data are not valid. Error: {}.'.format(
                jetsonObj["drone"].drone_name, e))


class TelemetryRetrieveAPIView(LoginRequiredMixin, generics.RetrieveUpdateDestroyAPIView):
    # queryset = Telemetry.objects.all().select_related('drone')

    serializer_class = TelemetrySerializer

    def get_object(self):
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        '''
        The following query set makes use of the  "Lookups that span relationships
        # lookups-that-span-relationships
        Reference: https://docs.djangoproject.com/en/1.11/topics/db/queries/
        '''
        obj = Telemetry.objects.filter(drone__drone_name=drone_name).last()
        if obj is None:
            raise Http404
        self.check_object_permissions(self.request, obj)
        return obj

    def save_telemetry_in_db(telemetryObj):
        telemetryObj['water_sampler_in_water'] = water_collector.water_sampler_under_water
        serializer = TelemetrySerializer(data=telemetryObj)
        if serializer.is_valid():
            serializer.save()
        else:
            msg = 'Telemetry Serializer data are not valid. Error: {}.'.format(
                serializer.error)
            from .consumers import ErrorMsg
            ErrorMsg.set_message_and_error(logger, Drone.objects.get(
                pk=telemetryObj.drone).operation.operation_name, msg)

    def save_error_drone_data_in_db(errorObj):
        serializer = ErrorMessageSerializer(data=errorObj)
        if serializer.is_valid():
            serializer.save()
        else:
            logger.error('Error Message Serializer data are not valid. Error: {}.'.format(
                serializer.error))


class MissionPointsListCreateAPIView(LoginRequiredMixin, generics.ListCreateAPIView):
    queryset = MissionPoint.objects.all()
    serializer_class = MissionPointSerializer

    def list(self, request, *args, **kwargs):
        '''
            Overriding the default method. We want a special use case here. We want to list
            the mission points for a particular mission for which the specified drone is part od
            Args:
                request:
                *args:
                **kwargs:
            Returns:
        '''
        operation_name = self.kwargs.get("operation_name")
        drone_name = self.kwargs.get("drone_name")

        # Get the mission points for the mission that this drone is currently participating
        qs = Drone.objects.filter(drone_name=drone_name, operation=Operation.objects.get(
            operation_name=operation_name))
        drone = get_object_or_404(qs)
        mission = drone.mission
        if (not mission):
            raise Http404(
                "This drone is not in any active missions at the moment")

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
        return render(request, 'aiders/users.html', {'users': users})


class DroneList(LoginRequiredMixin, generics.ListAPIView):
    queryset = Drone.objects.all()
    serializer_class = DroneSerializer

    def get(self, request, *args, **kwargs):
        drones = Drone.objects.all()
        return render(request, 'aiders/drones.html', {'drones': drones})

    def save_drone_to_db(droneObj):
        serializer = DroneSerializer(data=droneObj)
        if serializer.is_valid():
            drone = serializer.save()
            logger.info('Drone Serializer id {} is saved.'.format(drone.pk))
        else:
            logger.error(
                'Drone Serializer data are not valid. Error: {}.'.format(serializer.errors))


class UserDetail(LoginRequiredMixin, generics.RetrieveAPIView):
    queryset = get_user_model().objects.all()
    serializer_class = UserSerializer


class AlgorithmRetrieveView(LoginRequiredMixin, View):
    queryset = Algorithm.objects.all()
    serializer_class = AlgorithmSerializer

    def get(self, request, *args, **kwargs):

        attribute = self.kwargs.get("attr")
        '''
        Retrieve the algorithm with the specified id
        but only the "input" or "output" attribute
        '''
        if attribute != "input" and attribute != "output":
            return Response(status=status.HTTP_400_BAD_REQUEST)
        pk = self.kwargs.get("pk")

        algorithm = get_object_or_404(Algorithm.objects.filter(pk=pk))
        serializer = AlgorithmSerializer(algorithm)
        # res = Response(serializer.data)
        # attr = res.data.get(attribute)
        # res.data = attr
        attr_json = serializer.data.get(attribute)
        attr_html_tbale = json2html.convert(json=attr_json)

        return render(request, 'aiders/algorithm_info.html', {'attr_name': attribute, 'attr_object_html_format': attr_html_tbale})
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

    def save_algorithm_to_db(algorithmObj):
        serializer = AlgorithmSerializer(data=algorithmObj)
        if serializer.is_valid():
            serializer.save()
            logger.info('Algorithm Serializer is saved.')
        else:
            logger.error('Algorithm Serializer data are not valid. Error: {}.'.format(
                serializer.errors))


class ManageOperationsView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        operations = Operation.objects.all()
        users = User.objects.all()
        return render(request, 'aiders/manage_operations.html', {'operations': operations, 'users': users, 'use_online_maps': False})

# class JoinOperationView(LoginRequiredMixin,View):
#     def get(self, request, *args, **kwargs):
#         operation_id = self.kwargs.get("operation_id")
#         operation = Operation.objects.get(pk=operation_id)
#         return render(request, 'aiders/join_operation.html', {'operation': operation})


class ManagePermissionsView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        if not request.user.has_perm('aiders.edit_permissions'):
            raise PermissionDenied(
                "You do not have permission to read the permissions.")
        users = User.objects.exclude(username="AnonymousUser")
        for user in users:
            user.permission_edit_permissions = user.has_perm(
                'aiders.edit_permissions')
            user.permission_create_operations = user.has_perm(
                'aiders.create_operations')
            user.save()
        operation_groups = ''
        all_groups = Group.objects.all()
        for group in all_groups:
            if(str(group.name).__contains__(' operation join')):
                operation_groups = operation_groups + \
                    (group.name).replace(' operation join', '')+','
        return render(request, 'aiders/manage_permissions.html', {'users': users, 'all_groups': operation_groups})

    def post(self, request, *args, **kwargs):
        if not request.user.has_perm('aiders.edit_permissions'):
            raise PermissionDenied(
                "You do not have permission to change the permissions.")
        for user in User.objects.exclude(username="AnonymousUser"):
            User.update_permissions(user.id, 'permission_edit_permissions', str(
                user.id) in request.POST.getlist('permission_edit_permissions'))
            User.update_permissions(user.id, 'permission_create_operations', str(
                user.id) in request.POST.getlist('permission_create_operations'))
        users = User.objects.exclude(username="AnonymousUser")

        for user in users:
            user.permission_edit_permissions = user.has_perm(
                'aiders.edit_permissions')
            user.permission_create_operations = user.has_perm(
                'aiders.create_operations')
            user.save()
        operation_groups = ''
        all_groups = Group.objects.all()
        for group in all_groups:
            if(str(group.name).__contains__(' operation join')):
                operation_groups = operation_groups + \
                    (group.name).replace(' operation join', '')+','
        return render(request, 'aiders/manage_permissions.html', {'users': users, 'all_groups': operation_groups}, status=status.HTTP_202_ACCEPTED)


class ManageUserPermissionsView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        if not request.user.has_perm('aiders.edit_permissions'):
            raise PermissionDenied(
                "You do not have permission to change the permissions.")
        user_name = self.kwargs.get("user_name")
        group_list = request.POST.get('selected')
        group_list = group_list.split(',')
        for group in Group.objects.all():
            if(str(group.name).__contains__(' operation join')):
                User.objects.filter(username=user_name)[0].groups.remove(group)
        for group_name in group_list:
            group_object = Group.objects.filter(
                name=group_name+" operation join").last()
            User.objects.filter(username=user_name)[0].groups.add(group_object)
        return HttpResponse(status=status.HTTP_200_OK)


def index(request):
    '''
    Triggered when the main page of the web app is loaded on browser
    :param request:
    '''
    context = {'auth_form': AuthenticationForm}
    if request.user.is_authenticated:
        for drone in Drone.objects.filter(water_sampler_available=True):
            p = threading.Thread(
                target=water_collector.check_sensor, args=(drone.drone_name,))
            p.start()
        userQuery = User.objects.filter(pk=request.user.id)
        user = get_object_or_404(userQuery)
        joined_op_obj = user.joined_operation
        if (joined_op_obj):
            if request.method == 'POST':
                previous_page = resolve(
                    request.POST.get('next', '/')).func.view_class
                '''
                If we got here on the main page after a POST request, that means user posted some data from a form
                '''
                if (previous_page == AlgorithmListView):
                    '''
                    Check if we got here after user selected to show results for some algorithms (That is, if we got here from aiders/algorithms.html)
                    If this is the case, save the results to the request session and then redirect again to this page
                    This is because if we don't redirect, the "POST" request will persist.
                    Reference: https://stackoverflow.com/a/49178154/15290071
                    '''
                    algorithm_result_ids = request.POST.getlist(
                        'checkedAlgoResultIDs')
                    request.session['checkedAlgoResultIDs'] = algorithm_result_ids
                    return HttpResponseRedirect(reverse('home'))
                if (previous_page == MissionRetrieveAPIView):

                    mission_ids = request.POST.getlist('checkedMissionIDs')
                    request.session['checkedMissionIDs'] = mission_ids
                    return HttpResponseRedirect(reverse('home'))
            elif request.method == 'GET':

                context = {'operation': joined_op_obj,
                           'net_ip': os.environ.get("NET_IP", "localhost")}
                '''
                Check if there are any results to show for the algorithms
                '''
                user_wants_to_load_algorithm_results_on_map = True if request.session.get(
                    'checkedAlgoResultIDs') != None else False
                user_wants_to_load_missions_on_map = True if request.session.get(
                    'checkedMissionIDs') != None else False
                if (user_wants_to_load_algorithm_results_on_map):
                    algorithm_result_ids = request.session.get(
                        'checkedAlgoResultIDs')
                    try:
                        qs = Algorithm.objects.filter(
                            pk__in=algorithm_result_ids)
                        algorithm_results = get_list_or_404(qs)
                        algorithm_results = core_serializers.serialize(
                            'json', algorithm_results, fields=('pk', 'algorithm_name', 'output'))
                        context['algorithm_results'] = algorithm_results
                        del request.session['checkedAlgoResultIDs']
                    except:
                        '''
                        Remove the algorithm results from context if the user doesn't select an algorithm
                        '''
                        context.pop("algorithm_results", None)
                else:
                    '''
                    Remove the algorithm results from context if they exist.
                    user does not want to load any results on the map
                    e.g If the previous screen was the 'login' page, user just wants to log in, not to display any algorithm results
                    '''
                    context.pop("algorithm_results", None)

        else:
            context = {'join_operation_form': JoinOperationForm}

        use_online_map = UserPreferences.objects.get(
            user=request.user).use_online_map
        # context = {'auth_form': AuthenticationForm,'use_online_map':use_online_map}
        context['use_online_map'] = use_online_map
        return render(request, 'aiders/index.html', context)

    return render(request, 'aiders/login.html', context)


class DroneModifyOperationView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        drone_name = self.kwargs.get("drone_name")
        response = Operation.objects.filter(
            drones_to_operate=Drone.objects.get(drone_name=drone_name).pk)
        response = core_serializers.serialize('json', response)
        drone_data = Drone.objects.get(drone_name=drone_name)
        response = json.loads(response)
        for data in response:
            if str(data['fields']['operation_name']) == str(drone_data.operation):
                data['fields'].update({'Selected': 'Selected'})
        response = json.dumps(response)
        return HttpResponse(response)

    def post(self, request, *args, **kwargs):
        operation_name = request.POST['operation_name']
        drone_name = self.kwargs.get('drone_name')
        drone = Drone.objects.get(drone_name=drone_name)
        if operation_name == "None":
            drone.operation = None
            drone.save()
        else:
            try:
                drone.operation = Operation.objects.get(
                    operation_name=operation_name)
                drone.save()
            except:
                return HttpResponseNotFound("Operation not found", status=status.HTTP_400_BAD_REQUEST)
        return HttpResponse(drone_name, status=status.HTTP_202_ACCEPTED)


class BuildMapAPIView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        operation_name = self.kwargs.get('operation_name')
        drone_name = request.POST.get('drone_id')
        start_build_map = request.POST.get('start_build_map_boolean')
        multispectral_build_map = request.POST.get(
            'start_multispectral_build_map')
        overlap = request.POST.get("overlap")
        if start_build_map == 'true':
            build_map_request_handler.buildMapPublisherSingleMessage(
                drone_name, True, overlap)
            logger.info(
                'User sending build map request Start for drone {}.'.format(drone_name))
            buildSessionActive = BuildMapSession.objects.filter(user=User.objects.get(username=request.user.username), operation=Operation.objects.get(
                operation_name=operation_name), drone=Drone.objects.get(drone_name=drone_name)).last()
            droneActive = Drone.objects.get(
                drone_name=drone_name).build_map_activated
            if buildSessionActive == None:
                BuildMapSession.objects.create(user=User.objects.get(username=request.user.username), operation=Operation.objects.get(
                    operation_name=operation_name), drone=Drone.objects.get(drone_name=drone_name), folder_path=Constants.BUILD_MAP_DIR_PREFIX + drone_name + "_")
                drone = Drone.objects.get(drone_name=drone_name)
                drone.build_map_activated = True
                drone.save()
                return HttpResponse(status=status.HTTP_202_ACCEPTED)
            else:
                if buildSessionActive.is_active != True and droneActive != True:
                    BuildMapSession.objects.create(user=User.objects.get(username=request.user.username), operation=Operation.objects.get(
                        operation_name=operation_name), drone=Drone.objects.get(drone_name=drone_name), folder_path=Constants.BUILD_MAP_DIR_PREFIX + drone_name + "_")
                drone = Drone.objects.get(drone_name=drone_name)
                drone.build_map_activated = True
                drone.save()
                return HttpResponse(status=status.HTTP_202_ACCEPTED)
        elif start_build_map == 'false':
            build_map_request_handler.buildMapPublisherSingleMessage(
                drone_name, False, overlap)
            logger.info(
                'User sending build map request Stop for drone {}.'.format(drone_name))
            drone = Drone.objects.get(drone_name=drone_name)
            drone.build_map_activated = False
            drone.save()
            BuildMapSession.objects.filter(operation=Operation.objects.get(operation_name=operation_name), drone=Drone.objects.get(
                drone_name=drone_name), is_active=True).update(end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ), is_active=False)
            return HttpResponse(status=status.HTTP_202_ACCEPTED)
        logger.error(
            'Encounter an error when user send a build map request for drone {}.'.format(drone_name))
        return HttpResponse(status=status.HTTP_400_BAD_REQUEST)


class LidarPointsAPIView(LoginRequiredMixin, generic.UpdateView):
    def save_point_in_db(data, dji_name, lidar_session):
        if LidarPointSession.objects.get(id=lidar_session.id).is_active == True:
            LidarPoint.objects.create(
                points=data,
                lat=None,
                lon=None,
                drone=Drone.objects.get(drone_name=dji_name),
                lidar_point_session=lidar_session
            )


class BuildMapGetLastImageAPIView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        operation_name = self.kwargs.get('operation_name')
        drone_name = request.POST.get('drone_id')
        Session = BuildMapSession.objects.filter(operation=Operation.objects.get(operation_name=operation_name), drone=Drone.objects.get(
            drone_name=drone_name)).last()  # operation=Operation.objects.get(operation_name=operation_name),
        try:
            image = Session.images.all().last()
            image = model_to_dict(image)
        except:
            logger.error(
                'Encounter an error while searching for a Build Map image for drone {}.'.format(drone_name))
            return HttpResponse('', status=status.HTTP_404_NOT_FOUND)

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
        logger.info(
            'Found Build Map Image Successfully for drone {}.'.format(drone_name))
        return HttpResponse(json.dumps(image), status=status.HTTP_202_ACCEPTED)


class BuildMapGetLastAPIView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        operation_name = self.kwargs.get('operation_name')
        drone_name = request.POST.get('drone_id')
        buildMapSession = BuildMapSession.objects.filter(operation=Operation.objects.get(
            operation_name=operation_name), drone=Drone.objects.get(drone_name=drone_name)).last()
        if buildMapSession == None:
            logger.error(
                'Encounter an error while getting last image from Build Map Session for drone {}.'.format(drone_name))
            return HttpResponse(status=status.HTTP_404_NOT_FOUND)
        dictionary = {}
        dictionary['id'] = buildMapSession.pk
        dictionary['user'] = buildMapSession.user.username
        dictionary['drone_id'] = buildMapSession.drone.drone_name
        dictionary['start_time'] = str(
            buildMapSession.start_time.date())+" "+str(buildMapSession.start_time.time())
        response = json.dumps(dictionary)
        logger.info(
            'Found Build Map Session Successfully for drone {}.'.format(drone_name))
        return HttpResponse(response, status=status.HTTP_202_ACCEPTED)


@ csrf_exempt
def BuildMapImageView(request):
    if request.method == 'POST':
        img_file = request.FILES.get('image_file')
        img_name = request.POST.get('image_name')
        drone_name = request.POST.get('drone_name')
        drone_bearing = float(request.POST.get('bearing'))
        drone_alt = float(request.POST.get('alt'))
        drone_lat = float(request.POST.get('lat'))
        drone_lon = float(request.POST.get('lon'))
        extra_data = False
        try:
            d_roll = float(request.POST.get('d_roll'))
            d_pitch = float(request.POST.get('d_pitch'))
            d_yaw = float(request.POST.get('d_yaw'))
            g_roll = float(request.POST.get('g_roll'))
            g_pitch = float(request.POST.get('g_pitch'))
            g_yaw = float(request.POST.get('g_yaw'))
            extra_data = True
        except:
            extra_data = False
        drone_instance = Drone.objects.get(drone_name=drone_name)
        # if extra_data:
        #     # drone_bearing=drone_bearing+5
        #     drone_lat, drone_lon=img_georeference.high_accuracy_image_center(drone_lat, drone_lon, drone_alt, d_pitch, d_roll, drone_bearing)
        destinations = img_georeference.calcPoints(
            drone_lat, drone_lon, drone_bearing, drone_alt, img_name, drone_instance.model, drone_instance.camera_model)

        try:
            if drone_instance.is_connected_with_platform and drone_instance.build_map_activated:
                Session = BuildMapSession.objects.filter(
                    drone=Drone.objects.get(drone_name=drone_name)).last()
                Image.open(img_file)
                file_name = default_storage.save(os.path.join(
                    Session.folder_path, img_file.name),  img_file)
                if extra_data:
                    image = BuildMapImage.objects.create(
                        path=Session.folder_path+'/'+img_name,
                        top_left=Point(
                            destinations[2].longitude, destinations[2].latitude),
                        top_right=Point(
                            destinations[0].longitude, destinations[0].latitude),
                        bottom_left=Point(
                            destinations[1].longitude, destinations[1].latitude),
                        bottom_right=Point(
                            destinations[3].longitude, destinations[3].latitude),
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
                        path=Session.folder_path+'/'+img_name,
                        top_left=Point(
                            destinations[2].longitude, destinations[2].latitude),
                        top_right=Point(
                            destinations[0].longitude, destinations[0].latitude),
                        bottom_left=Point(
                            destinations[1].longitude, destinations[1].latitude),
                        bottom_right=Point(
                            destinations[3].longitude, destinations[3].latitude),
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
                logger.info(
                    'Saved Image Successfully for Build Map Session {}.'.format(Session.id))
                return HttpResponse({'status:success'}, status=status.HTTP_200_OK)
        except Exception as e:
            print(e)
            return HttpResponse({'status:failed'}, status=status.HTTP_400_BAD_REQUEST)


class BuildMapLoadAPIView(LoginRequiredMixin, generic.UpdateView):
    def get(self, request, *args, **kwargs):
        operation = Operation.objects.get(
            operation_name=self.kwargs['operation_name'])
        list_of_operation = list(operation.buildmapsession_set.all())

        response = []
        for data in list_of_operation:
            dictionary = {}
            dictionary['id'] = data.pk
            dictionary['user'] = data.user.username
            dictionary['drone_id'] = data.drone.drone_name
            dictionary['start_time'] = str(
                data.start_time.date())+" "+str(data.start_time.time())
            dictionary['end_time'] = str(
                data.end_time.date())+" " + str(data.end_time.time())
            # Checks if the Session haves images
            if list(BuildMapImage.objects.filter(session=data)) != []:
                response.append(dictionary)
        json_string = json.dumps(response)
        return HttpResponse(json_string)

    def post(self, request, *args, **kwargs):
        try:
            build_map_id = json.loads(
                request.body.decode('utf-8'))['build_map_id']
        except:
            return HttpResponse(status=status.HTTP_400_BAD_REQUEST)
        print(build_map_id)
        map_build = list(BuildMapImage.objects.filter(
            session_id=build_map_id).values())
        print(map_build)
        for data in map_build:
            data['time'] = str(data['time'])
            data['top_left'] = [float(data['top_left'].coords[0]), float(
                data['top_left'].coords[1])]
            data['top_right'] = [float(data['top_right'].coords[0]), float(
                data['top_right'].coords[1])]
            data['bottom_left'] = [float(data['bottom_left'].coords[0]), float(
                data['bottom_left'].coords[1])]
            data['bottom_right'] = [float(data['bottom_right'].coords[0]), float(
                data['bottom_right'].coords[1])]
            data['centre'] = [float(data['centre'].coords[0]), float(
                data['centre'].coords[1])]
            data['altitude'] = float(data['altitude'])
            data['bearing'] = float(data['bearing'])
        json_string = json.dumps(map_build)
        return HttpResponse(json_string, status=status.HTTP_201_CREATED)


class FirePredictionCreateAPIView(LoginRequiredMixin, generic.UpdateView):
    def post(self, request, *args, **kwargs):
        for jsonPostData in request:
            try:
                PostData = json.loads(jsonPostData)
                if PostData['user']:
                    operation = Operation.objects.get(
                        operation_name=self.kwargs['operation_name'])
                    operationPK = operation.pk
                    user = User.objects.get(username=PostData['user'])
                    userPK = user.pk
                    algorithmName = 'FIRE_PROPAGATION_ALGORITHM'
                    canBeLoadedOnMap = True
                    input = PostData
                    del input['user']
                    try:
                        output = utils.handleAlgorithmExecution(
                            operationPK, input, canBeLoadedOnMap, algorithmName, userPK)
                    except Exception as e:
                        print(e)
                        return HttpResponse(status=status.HTTP_400_BAD_REQUEST)
                    response = '['+str(output)+']'
                    return HttpResponse(response, status=status.HTTP_201_CREATED)
            except:
                pass
        raise Http404


def login_view(request):
    if request.method == 'GET':
        redirect_to = request.GET.get('next')
        if request.user.is_authenticated:
            if redirect_to != None:
                return HttpResponseRedirect(redirect_to)
            return HttpResponseRedirect(reverse('manage_operations'))
        return render(request, 'aiders/login.html', {'auth_form': AuthenticationForm, 'next': redirect_to})
    if request.method == 'POST':
        username = request.POST['username']
        password = request.POST['password']
        redirect_to = request.POST['next']
        user = authenticate(request, username=username, password=password)
        if user is not None:
            if user.is_active:
                if request.META.get('HTTP_X_FORWARDED_FOR'):
                    ip = request.META.get('HTTP_X_FORWARDED_FOR')
                else:
                    ip = request.META.get('REMOTE_ADDR')

                from user_agents import parse
                user_agent = parse(request.META.get('HTTP_USER_AGENT'))
                '''
                When user logs in, save a few data that concern their machine
                '''
                terminal = Terminal(ip_address=ip, user=user,
                                    os=user_agent.os.family,
                                    device=user_agent.device.family,
                                    logged_in=True,
                                    browser=user_agent.browser.family)
                terminal.save()

                if not UserPreferences.objects.filter(user=user).exists():
                    UserPreferences.objects.create(
                        use_online_map=True, user=user)

                login(request, user,
                      backend='django.contrib.auth.backends.ModelBackend')

                if redirect_to != "None":
                    return HttpResponseRedirect(redirect_to)
                return redirect('manage_operations')
        else:
            messages.error(request, 'Wrong username or password!')
            return render(request, 'aiders/login.html', {'auth_form': AuthenticationForm, 'next': redirect_to})


def logout_view(request):
    logout(request)
    # Redirect to a success page
    return redirect('login')


class NewOperationForm(LoginRequiredMixin, SessionWizardView):
    template_name = 'aiders/operation_new_wizard.html'

    def get_form_initial(self, step):
        if not self.request.user.has_perm('aiders.create_operations'):
            raise PermissionDenied(
                "You do not have permission to create the operation.")

    def done(self, form_list, form_dict, **kwargs):

        wizard_form = {k: v for form in form_list for k,
                       v in form.cleaned_data.items()}
        operation_instance = Operation.objects.none()
        wizard_form["operator"] = self.request.user

        operation_instance = Operation.objects.create(
            operation_name=wizard_form["operation_name"],
            location=wizard_form["location"],
            description=wizard_form["description"],
            operator=wizard_form["operator"],
        )
        drone_allow_list = Drone.objects.none()
        for drone_id in form_list[1].data.getlist('Drones in'):
            drone_allow_list = drone_allow_list | Drone.objects.filter(
                pk=drone_id)
            if form_list[1].data.getlist('drone_operation') == ['True']:
                print(Drone.objects.get(pk=drone_id).operation)
                if Drone.objects.get(pk=drone_id).operation == None or Drone.objects.get(pk=drone_id).is_connected_with_platform == False:
                    Drone.objects.filter(pk=drone_id).update(
                        operation=operation_instance)
        operation_instance.drones_to_operate.set(drone_allow_list)

        group_join_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        group_edit_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation edit")
        assign_perm('join_operation', group_join_operation, operation_instance)
        assign_perm('edit_operation', group_edit_operation, operation_instance)

        for user_id in form_list[1].data.getlist('Users in'):
            User.objects.filter(pk=user_id)[0].groups.add(group_join_operation)

        wizard_form["operator"].groups.add(group_edit_operation)
        logger.info('Operation with id {} is created successfully.'.format(
            operation_instance.pk))
        return redirect('manage_operations')


class EditOperationForm(LoginRequiredMixin, SessionWizardView):
    template_name = 'aiders/operation_edit_wizard.html'

    def get_form_initial(self, step):
        operation_name = self.kwargs['operation_name']
        operation = Operation.objects.get(operation_name=operation_name)
        if self.request.user.has_perm('edit_operation', operation):
            if 'operation_name' in self.kwargs and step == '0':
                operation_dict = model_to_dict(operation)
                return operation_dict
            else:
                return self.initial_dict.get(step, {})
        else:
            raise PermissionDenied(
                "You do not have permission to change the operation.")

    def get_context_data(self, form, **kwargs):
        context = super(EditOperationForm, self).get_context_data(
            form=form, **kwargs)
        if self.steps.current == '1':
            initial = {
                'users_in': [],
                'users_out': [],
            }

            operation_name = self.kwargs['operation_name']
            operation = Operation.objects.get(operation_name=operation_name)
            operation_drones_dict = model_to_dict(operation)
            all_drones = list(Drone.objects.all())

            for user in User.objects.all():
                # Don't display the 'AnonymousUser' on the user list. We don't care about anonymous users
                if not user.username == 'AnonymousUser':
                    if user.has_perm('join_operation', operation):
                        initial['users_in'].append(user)
                    else:
                        initial['users_out'].append(user)
            context.update({'drones_allow': set(all_drones) & set(
                operation_drones_dict['drones_to_operate'])})
            context.update({'drones_all': set(all_drones) ^ set(
                operation_drones_dict['drones_to_operate'])})
            context.update({'users_allow': initial['users_in']})
            context.update({'users_all': initial['users_out']})
            context.update({'edit_form': True})
        return context

    def done(self, form_list, form_dict, **kwargs):
        wizard_form = {k: v for form in form_list for k,
                       v in form.cleaned_data.items()}
        drone_allow_list = Drone.objects.none()
        operation_name = self.kwargs['operation_name']
        operation_instance = Operation.objects.get(
            operation_name=operation_name)
        for drone_id in form_list[1].data.getlist('Drones in'):
            drone_allow_list = drone_allow_list | Drone.objects.filter(
                pk=drone_id)
            if form_list[1].data.getlist('drone_operation') == ['True']:
                print(Drone.objects.get(pk=drone_id).operation)
                if Drone.objects.get(pk=drone_id).operation == None or Drone.objects.get(pk=drone_id).is_connected_with_platform == False:
                    Drone.objects.filter(pk=drone_id).update(
                        operation=operation_instance)
        operation_instance.location = wizard_form['location']
        operation_instance.description = wizard_form['description']
        operation_instance.drones_to_operate.set(drone_allow_list)
        operation_instance.save()

        Group.objects.get(
            name=operation_instance.operation_name+" operation join").delete()
        group = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        assign_perm('join_operation', group, operation_instance)

        for user_id in form_list[1].data.getlist('Users in'):
            User.objects.filter(pk=user_id)[0].groups.add(group)

        # Iterate over the drones that are NOT allowed on this operation.
        # If these drones were until now joined on this operation, kick them out
        notAllowedDrones = form_list[1].data.getlist('Drones out')
        for dronePK in notAllowedDrones:
            droneInstance = Drone.objects.get(pk=dronePK)
            if droneInstance.operation == operation_instance:
                Drone.objects.filter(
                    drone_name=droneInstance.drone_name).update(operation=None)
        return redirect('manage_operations')


class ExecuteAlgorithmAPIView(LoginRequiredMixin, APIView):
    def post(self, request, *args, **kwargs):
        operation_name = kwargs['operation_name']
        operation = Operation.objects.get(operation_name=operation_name)
        userPK = request.user.pk
        operationPK = operation.pk
        algorithmDetails = request.data
        algorithmName = algorithmDetails['algorithmName']
        input = algorithmDetails['input']
        canBeLoadedOnMap = algorithmDetails['canBeLoadedOnMap']
        output = utils.handleAlgorithmExecution(
            operationPK, input, canBeLoadedOnMap, algorithmName, userPK)
        return Response(output)


class ExecuteMissionAPIView(LoginRequiredMixin, APIView):
    def get(self, request, *args, **kwargs):
        operation_name = kwargs['operation_name']
        drone_name = kwargs['drone_name']
        user = request.user
        operation = Operation.objects.get(operation_name=operation_name)
        drone = Drone.objects.get(drone_name=drone_name)

        mission_log = MissionLog.objects.filter(
            action='START_MISSION', user=user.pk, drone=drone, operation=operation).last()
        return Response(mission_log.mission.mission_type)

    def post(self, request, *args, **kwargs):
        # print("Request of the Execute Mission:", request, "\nand kwargs:", kwargs)
        operation_name = kwargs['operation_name']
        drone_name = kwargs['drone_name']
        actionDetails = request.data
        user_name = request.user.username
        operation = Operation.objects.get(operation_name=operation_name)

        User = get_user_model()
        action = actionDetails['action']
        grid = actionDetails['grid']
        captureAndStoreImages = actionDetails['captureAndStoreImages']
        missionPath = actionDetails['mission_points']
        dronePK = Drone.objects.get(drone_name=drone_name).pk
        try:
            missionType = actionDetails['mission_type']
        except:
            missionType = None
        # if missionType == Mission.NORMAL_MISSION:
        mission_request_handler.publishMissionToRos(
            operation.pk, missionType, drone_name, grid, captureAndStoreImages, missionPath, action, request.user.pk, dronePK)
        # elif missionType == Mission.SEARCH_AND_RESCUE_MISSION:
        #     utils.handleAlgorithmExecution(operation.pk, input, canBeLoadedOnMap, userPK, algorithmName)
        # pass
        return Response(status=status.HTTP_200_OK)


class AlgorithmListView(LoginRequiredMixin, generic.ListView):
    model = Algorithm
    # fields = ('__all__')
    template_name = 'aiders/algorithms.html'
    queryset = Algorithm.objects.all()
    success_url = reverse_lazy('home')

    # def get(self, request, *args, **kwargs):
    #     context = self.get_context_data()
    #     return self.render_to_response(context)
    #
    #     # self.object = self.get_object()
    #     # context = self.get_context_data(object=self.object)
    #     # return self.render_to_response(context)

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(
            operation_name=self.kwargs.get('operation_name'))

        if not self.request.user.has_perm('join_operation', Operation.objects.filter(operation_name=self.kwargs.get('operation_name'))[0]):
            raise PermissionDenied(
                "You do not have permission to join the operation.")

        # User has to join the operation in order to view the operation's algorithms
        User.objects.filter(pk=self.request.user.id).update(
            joined_operation=operation)

        context = super(AlgorithmListView, self).get_context_data(**kwargs)
        context['algorithm_results'] = operation.algorithm_set.all()
        context['operation_name'] = self.kwargs.get('operation_name')
        # Create any data and add it to the context
        return context


@ login_required
@ csrf_protect
def stop_operation_view(request, operation_name):
    if request.method == 'GET':
        opQuery = Operation.objects.filter(operation_name=operation_name)

        if (opQuery.exists()):
            operation = get_object_or_404(opQuery)

            if (operation.active):
                operation.active = False
                operation.save()
                return redirect('manage_operations')


@ login_required
@ csrf_protect
def leave_operation_view(request):
    if request.method == 'GET':
        get_user_model().objects.filter(pk=request.user.id).update(joined_operation=None)
        return redirect('manage_operations')
        # if (userQuery.exists()):
        #     get_object_or_404(userQuery).update(joined_operation=None)
        #     user.joined_operation = None
        #     user.save()
        # return redirect('home')


@ login_required
@ csrf_protect
def join_operation_view(request, operation_name):
    if not request.user.has_perm('join_operation', Operation.objects.filter(operation_name=operation_name)[0]):
        raise PermissionDenied(
            "You do not have permission to join the operation.")
    if request.method == 'POST':
        opQuery = Operation.objects.filter(operation_name=operation_name)
        if (opQuery.exists()):
            operation = get_object_or_404(opQuery)
            if (operation.active):
                User.objects.filter(pk=request.user.id).update(
                    joined_operation=operation)
                # get_object_or_404(user_query)
                return redirect('home')
            else:
                raise Http404('Operation Not Found')
        else:
            raise Http404('Operation Not Found')
    return JsonResponse({'success': False})


@ csrf_protect
def register_request(request):
    if request.method == 'POST':
        form = NewUserForm(request.POST)
        if form.is_valid():
            user = form.save()
            login(request, user, backend='django.contrib.auth.backends.ModelBackend')
            return redirect('manage_operations')
    else:
        form = NewUserForm()
    return render(request=request, template_name='aiders/register.html', context={"register_form": form})


class DetectionAPIOperations():

    @ staticmethod
    def create_detection_session_on_db(user, operation, drone):
        return DetectionSession.objects.create(
            user=user,
            operation=operation,
            drone=drone
        )

    @ staticmethod
    def save_frame_to_db(frame_file, detection_session):
        detFrame = DetectionFrame.objects.create(
            frame=frame_file,
            detection_session=detection_session,
        )
        return detFrame

    @ staticmethod
    def update_detection_status_on_db(drone, detection_status, detection_type_str):
        qs = Detection.objects.filter(drone__drone_name=drone.drone_name).update(
            detection_status=detection_status, detection_type_str=detection_type_str)

    @ staticmethod
    def update_detection_session_end_time(detection_session):
        end_time = datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ)
        DetectionSession.objects.filter(pk=detection_session.id).update(
            end_time=end_time, is_active=False)

    @ staticmethod
    def update_latest_frame(detection_session, latest_frame_url):
        DetectionSession.objects.filter(pk=detection_session.id).\
            update(latest_frame_url=latest_frame_url)

    @ staticmethod
    def save_detected_object_to_db(detection_session, detectedObj, frame):
        DetectedObject.objects.create(
            track_id=detectedObj.trk_id,
            label=detectedObj.label,
            lat=detectedObj.lat,
            lon=detectedObj.lon,
            detection_session=detection_session,
            distance_from_drone=detectedObj.distFromDrone,
            frame=frame
        )


class LiveStreamAPIOperations(LoginRequiredMixin, generics.RetrieveAPIView):

    # def get(self, request, *args, **kwargs):
    #     operation_name=self.kwargs.get('operation_name')
    #     drone_name = self.kwargs.get('drone_name')

    @ staticmethod
    def create_live_stream_session_on_db(drone):
        return LiveStreamSession.objects.create(
            drone=drone
        )

    @ staticmethod
    def save_raw_frame_to_db(frame_file, drone_name, live_stream_session):
        detFrame = RawFrame.objects.create(
            frame=frame_file,
            drone=Drone.objects.get(drone_name=drone_name),
            live_stream_session=live_stream_session,
        )
        return detFrame

    @ staticmethod
    def update_latest_raw_frame(live_stream_session, latest_frame_url):
        LiveStreamSession.objects.filter(pk=live_stream_session.id).\
            update(latest_frame_url=latest_frame_url)


@ api_view(['GET'])
def objects_detected_on_last_frame_api_view(request, operation_name, drone_name):
    if request.method == 'GET':
        try:
            active_detection_session = DetectionSession.objects.filter(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name)
            active_detection_session = DetectionSession.objects.get(
                is_active=True, operation__operation_name=operation_name, drone__drone_name=drone_name)
            # Get the last frame object for the active detection session
            latest_frame = DetectionFrame.objects.filter(
                detection_session=active_detection_session).last()
            # Get the detected objects that appear on the last frame
            detected_objects = DetectedObject.objects.filter(
                frame=latest_frame)

        except DetectionSession.DoesNotExist:
            return Response({'error': Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE})
        if detected_objects == None:
            return Response({'error': "No objects detected on last frame"})
        serializer = DetectedObjectSerializer(detected_objects, many=True)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


@ api_view(['GET'])
def last_detection_frame_api_view(request, operation_name, drone_name):
    if request.method == 'GET':
        try:
            active_detection_session = DetectionSession.objects.get(
                is_active=True, drone__drone_name=drone_name)
        except DetectionSession.DoesNotExist:
            return Response({'latest_frame_url': Constants.NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE})
        serializer = DetectionSessionSerializer(active_detection_session)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


@ api_view(['GET'])
def last_raw_frame_api_view(request, operation_name, drone_name):
    if request.method == 'GET':
        try:
            active_detection_session = LiveStreamSession.objects.get(
                is_active=True, drone__drone_name=drone_name)
        except LiveStreamSession.DoesNotExist:
            return Response({'latest_frame_url': Constants.NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE})
        serializer = LiveStreamSessionSerializer(active_detection_session)
        return Response(serializer.data)

    return Response(status=status.HTTP_400_BAD_REQUEST)


@ api_view(['GET'])
def detection_types_api_view(request, operation_name):
    if request.method == 'GET':
        from logic.algorithms.object_detection.src.models.label import \
            get_labels_all
        return Response({'detection_types': list(get_labels_all())})

    return Response(status=status.HTTP_400_BAD_REQUEST)


@ api_view(['GET'])
def live_stream_status_api_view(request, operation_name, drone_name):
    if request.method == 'GET':
        liveStreamSession = LiveStreamSession.objects.get(
            drone__drone_name=drone_name)
        if (liveStreamSession.is_active):
            return Response({'is_live_stream_active': True})
        else:
            return Response({'is_live_stream_active': False})
    return Response(status=status.HTTP_400_BAD_REQUEST)


class WeatherLiveAPIView(LoginRequiredMixin, APIView):
    def post(self, request, *args, **kwargs):
        ThreadRunningPub = False
        ThreadRunningSub = False
        threadName = []
        for thread in threading.enumerate():
            if thread.name == 'MainWeatherPublisher':
                threadName.append(thread)
                ThreadRunningPub = True
            elif thread.name == 'MainWeatherSubscriber':
                threadName.append(thread)
                ThreadRunningSub = True
        if request.data['state'] == 'true':
            operation_name = self.kwargs.get('operation_name')
            operation_name = operation_name.replace(' ', '~')
            if ThreadRunningPub == False:
                publisherThread = MyThread(name='MainWeatherPublisher', target=weather_station_ros_publisher.main, args=(
                    operation_name, 'MainWeatherPublisher'))
                sys.argv = Constants.START_WEATHER_DATA_PUBLISHER_SCRIPT[1:]
                publisherThread.start()
            if ThreadRunningSub == False:
                subscriberThread = MyThread(name='MainWeatherSubscriber', target=weather_station_ros_subscriber.main, args=(
                    operation_name, 'MainWeatherSubscriber'))
                subscriberThread.start()
        else:
            for thread in threadName:
                thread.stop()
        return HttpResponse('Threads up', status=status.HTTP_200_OK)


class WeatherStationAPIView(LoginRequiredMixin, generics.RetrieveAPIView):
    queryset = WeatherStation.objects.all()
    serializer_class = WeatherStationSerializer

    def addWeatherStationDataToDB(data, object_name):
        object_name = object_name.replace('~', ' ')
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


def system_monitoring_save_to_db(cpu_usage, cpu_core_usage, cpu_temp, gpu_usage, gpu_memory, gpu_temp, ram_usage, swap_memory_usage, temp, mb_new_sent, mb_new_received, mb_new_total, disk_read, disk_write, battery_percentage):
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
        battery_percentage=battery_percentage
    )


class buildMapSessionsAPIView(LoginRequiredMixin, generic.ListView):
    model = BuildMapSession

    template_name = 'aiders/buildMapSession.html'
    queryset = BuildMapSession.objects.all()

    def get_context_data(self, **kwargs):
        # Call the base implementation first to get the context
        operation = Operation.objects.get(
            operation_name=self.kwargs.get('operation_name'))

        if not self.request.user.has_perm('join_operation', Operation.objects.filter(operation_name=self.kwargs.get('operation_name'))[0]):
            raise PermissionDenied(
                "You do not have permission to join the operation.")

        context = super(buildMapSessionsAPIView,
                        self).get_context_data(**kwargs)
        context['MapSession_results'] = list(
            operation.buildmapsession_set.all())
        index = 0
        urlList = []
        list_non_zero_images = list(BuildMapImage.objects.filter().values(
            'session').annotate(n=models.Count("pk")))
        while index < len(context['MapSession_results']):
            element = context['MapSession_results'][index]
            save = False
            for session_non_zero_images in list_non_zero_images:
                if session_non_zero_images['session'] == context['MapSession_results'][index].id:
                    context['MapSession_results'][index].images = session_non_zero_images['n']
                    save = True
            if save == False:
                context['MapSession_results'].remove(element)
            else:
                urlList.append(self.request.build_absolute_uri(reverse(
                    'build_map_session_share', args=[self.kwargs.get('operation_name'), element.id])))
                index += 1
        context['operation_name'] = self.kwargs.get('operation_name')
        context['urls'] = urlList
        return context


class buildMapSessionsShareAPIView(LoginRequiredMixin, View):

    def get(self, request, *args, **kwargs):

        self.kwargs.get('pk')
        buildMapSessionObject = BuildMapSession.objects.get(
            pk=self.kwargs.get('pk'))
        fileList = []
        with open('buildMapSession.csv', 'w') as csvFile:
            fileWriter = csv.writer(
                csvFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            fileWriter.writerow(
                [f.name for f in BuildMapSession._meta.get_fields()])
            dataList = []
            for key in [f.name for f in BuildMapSession._meta.get_fields()]:
                try:
                    dataList.append(getattr(buildMapSessionObject, key))
                except:
                    dataList.append("")
            fileWriter.writerow(dataList)

        with open('buildMapImages.csv', 'w') as csvFile2:
            fileWriter = csv.writer(
                csvFile2, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            fileWriter.writerow(
                [f.name for f in BuildMapImage._meta.get_fields()])
            for data in BuildMapImage.objects.filter(session=self.kwargs.get('pk')):
                dataList = []
                for key in [f.name for f in BuildMapImage._meta.get_fields()]:
                    try:
                        if isinstance(getattr(data, key), point.Point):
                            dataList.append(str(getattr(data, key).coords[0])+' '+str(
                                getattr(data, key).coords[1]))
                        else:
                            dataList.append(getattr(data, key))
                    except:
                        dataList.append("")
                fileWriter.writerow(dataList)

        try:
            if not os.path.exists(default_storage.path('')+'/temp/'):
                os.makedirs(default_storage.path('')+'/temp/')
            else:
                shutil.rmtree(default_storage.path('')+'/temp/')
                os.makedirs(default_storage.path('')+'/temp/')
            shutil.move('buildMapSession.csv', default_storage.path(
                '')+'/temp/buildMapSession.csv')
            shutil.move('buildMapImages.csv', default_storage.path(
                '')+'/temp/buildMapImages.csv')
            os.mkdir(default_storage.path('')+'/temp/' +
                     BuildMapImage.objects.filter(session=self.kwargs.get('pk')).last().path.split('/')[0])
            for data in BuildMapImage.objects.filter(session=self.kwargs.get('pk')):
                shutil.copyfile(default_storage.path(data.path),
                                default_storage.path('')+'/temp/'+data.path)
        except Exception as e:
            pass

        try:
            zip_file = zipfile.ZipFile(default_storage.path(
                'build_map_session_share.zip'), 'w')
            for root, dirs, files in os.walk(default_storage.path('temp')):
                for f in files:
                    zip_file.write(os.path.join(root, f), f)
            zip_file.close()
            zip_file = open(default_storage.path(
                'build_map_session_share.zip'), 'rb')
            return FileResponse(zip_file)
        except Exception as e:
            return HttpResponse(status=status.HTTP_404_NOT_FOUND)


class waterCollectionActivatedAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        drone_name = request.POST.get('drone_id')
        operation_name = kwargs.get('operation_name')
        if Drone.objects.get(drone_name=drone_name).water_sampler_available:
            try:
                water_collector.publish_message(drone_name, 1)
                WaterSampler.objects.create(
                    drone=Drone.objects.get(drone_name=drone_name),
                    operation=Operation.objects.get(
                        operation_name=operation_name),
                    user=User.objects.get(pk=request.user.pk),
                    telemetry=Telemetry.objects.filter(
                        drone=Drone.objects.get(drone_name=drone_name)).last(),
                )
                logger.info(
                    'Water sampler activated for drone {}.'.format(drone_name))
                return HttpResponse('Sending message to drone.', status=status.HTTP_200_OK)
            except Exception as e:
                logger.error(
                    'Water sampler encounter an error for drone {}. Error: {}'.format(drone_name, e))
        return HttpResponse('Water sampler encounter an error for drone {}.'.format(drone_name), status=status.HTTP_200_OK)


class ballisticActivatedAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        drone_name = request.POST.get('drone_id')
        operation_name = kwargs.get('operation_name')
        if Drone.objects.get(drone_name=drone_name).ballistic_available:
            try:
                ballistic.publish_message(drone_name, 1)
                Ballistic.objects.create(
                    drone=Drone.objects.get(drone_name=drone_name),
                    operation=Operation.objects.get(
                        operation_name=operation_name),
                    user=User.objects.get(pk=request.user.pk),
                    telemetry=Telemetry.objects.filter(
                        drone=Drone.objects.get(drone_name=drone_name)).last(),
                )
                logger.info(
                    'Ballistic activated for drone {}.'.format(drone_name))
                return HttpResponse('Sending message to drone.', status=status.HTTP_200_OK)
            except Exception as e:
                logger.error(
                    'Ballistic encounter an error for drone {}. Error: {}'.format(drone_name, e))
        return HttpResponse('Ballistic encounter an error for drone {}.'.format(drone_name), status=status.HTTP_200_OK)


class rangeFinderAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        drone_name = request.POST.get('drone_id')
        start_stop = request.POST.get('start_stop')
        operation_name = kwargs.get('operation_name')
        if Drone.objects.get(drone_name=drone_name).camera_model:
            try:
                range_detection.buildMapPublisherSingleMessage(
                    drone_name, start_stop)
                logger.info(
                    'Range Finder activated for drone {}.'.format(drone_name))
                return HttpResponse('Sending message to drone.', status=status.HTTP_200_OK)
            except Exception as e:
                logger.error(
                    'Range Finder encounter an error for drone {}. Error: {}'.format(drone_name, e))
        return HttpResponse('Range Finder an error for drone {}.'.format(drone_name), status=status.HTTP_200_OK)


class frontEndUserInputAPIView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        element = request.POST.get('elementId')
        value = None
        if request.POST.get('active') == "true":
            active = True
        elif request.POST.get('active') == "false":
            active = False
        else:
            active = True
            value = request.POST.get('active')
        operation_name = kwargs.get('operation_name')
        try:
            FrontEndUserInput.objects.create(
                operation=Operation.objects.get(operation_name=operation_name),
                element_name=element,
                active=active,
                value=value
            )
            return HttpResponse('Action Saved Successful.', status=status.HTTP_200_OK)
        except Exception as e:
            logger.error(e)
        return HttpResponse("Action Not Saved Successful.", status=status.HTTP_200_OK)


class SystemMonitoringView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        if request.user.is_superuser:
            return render(request, 'aiders/monitoring-platform.html', {})
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)


class ControlDevicesMonitoringView(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        if request.user.is_superuser:
            drones = Drone.objects.all()
            available_drones = list(ControlDevice.objects.filter().values(
                'drone').annotate(n=models.Count("pk")))
            temp = []
            for drones_temp in available_drones:
                temp.append(Drone.objects.get(id=drones_temp['drone']))
            available_drones = temp
            return render(request, 'aiders/monitoring-control-devices.html', {'drones': drones, 'available_drones': available_drones})
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)


class ControlDeviceMonitoringView(LoginRequiredMixin, View):
    def post(self, request, *args, **kwargs):
        print(kwargs.get('control_device'))
        if request.user.is_superuser:
            drone_name = kwargs.get('control_device')
            available_drones = list(ControlDevice.objects.filter().values(
                'drone').annotate(n=models.Count("pk")))
            temp = []
            for drones_temp in available_drones:
                temp.append(drones_temp['drone'])
            available_drones = temp
            if not Drone.objects.get(drone_name=drone_name).id in available_drones:
                return HttpResponse(status=status.HTTP_404_NOT_FOUND)
            return render(request, 'aiders/monitoring-control-device.html', {'drone_name': drone_name})
        return HttpResponse(status=status.HTTP_401_UNAUTHORIZED)

    def test_my_high_accuracy(self, lat, long, altitude, pitch, roll, heading):
        import math

        from geopy.distance import geodesic
        pitch = pitch
        roll = roll
        distance_pitch = altitude * math.tan(pitch*math.pi/180)  # lat
        distance_roll = altitude * math.tan(roll*math.pi/180)  # long
        destination_pitch = geodesic(
            kilometers=distance_pitch/1000).destination((0, 0), heading+0)
        destination_roll = geodesic(
            kilometers=distance_roll/1000).destination((0, 0), heading+270)
        newLat = lat+destination_pitch.latitude+destination_roll.latitude
        newLong = long+destination_pitch.longitude+destination_roll.longitude
        return(newLat, newLong)


class LoraTransmitterLocationRetrieveAPIView(LoginRequiredMixin, generics.RetrieveAPIView):
    queryset = LoraTransmitterLocation.objects.all()
    serializer_class = LoraTransmitterLocationSerializer
    lookup_field = 'tagName'

    def get_object(self):
        tag_name = self.kwargs.get("lora_device_name")
        qs = LoraTransmitterLocation.objects.filter(
            loraTransmitter__tagName=tag_name)
        if not qs.exists():
            raise Http404('Object not found')
        return qs.last()  # Return the most recent information about this lora device


class LoraTransmiterListAPIView(LoginRequiredMixin, generics.ListAPIView):
    queryset = LoraTransmitter.objects.all()
    serializer_class = LoraTransmitterSerializer


class Lidar3DMesh(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        lidar_session_list = list(LidarPointSession.objects.filter(operation=Operation.objects.get(
            operation_name=self.kwargs.get("operation_name")), is_process=True, is_active=False).values())
        for session in lidar_session_list:
            session['start_time'] = str(session['start_time'])
            if session['end_time'] != None:
                session['end_time'] = str(session['end_time'])
        lidar_session_list = json.dumps(lidar_session_list)
        return HttpResponse(lidar_session_list)

    def post(self, request, *args, **kwargs):
        mesh_id = request.POST.get('mesh_id')
        data_return = {}
        data_return['id'] = mesh_id
        data_return['file_path'] = 'triangle_mesh/'+str(mesh_id)+'.glb'
        lat = LidarPoint.objects.filter(lidar_point_session=mesh_id).aggregate(
            Avg('telemetry__lat'))
        lon = LidarPoint.objects.filter(
            lidar_point_session=mesh_id).aggregate(Avg('telemetry__lon'))
        data_return['long'] = lon['telemetry__lon__avg']
        data_return['lat'] = lat['telemetry__lat__avg']
        data_return['height'] = 0
        data_return['heading'] = LidarPoint.objects.filter(
            lidar_point_session=mesh_id)[0].telemetry.heading
        data_return = json.dumps(data_return)
        return HttpResponse(data_return)


class Lidar3DPoints(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        lidar_session_list = list(LidarPointSession.objects.filter(operation=Operation.objects.get(
            operation_name=self.kwargs.get("operation_name")), is_active=False).values())
        for session in lidar_session_list:
            session['start_time'] = str(session['start_time'])
            if session['end_time'] != None:
                session['end_time'] = str(session['end_time'])
        lidar_session_list = json.dumps(lidar_session_list)
        return HttpResponse(lidar_session_list)

    def post(self, request, *args, **kwargs):
        mesh_id = request.POST.get('mesh_id')
        list_lidar_points = list(LidarPoint.objects.filter(
            lidar_point_session=mesh_id))
        point_dict = self.point_db_to_json(list_lidar_points)
        point_dict = {'data': point_dict}
        lat = LidarPoint.objects.filter(lidar_point_session=mesh_id).aggregate(
            Avg('telemetry__lat'))
        lon = LidarPoint.objects.filter(
            lidar_point_session=mesh_id).aggregate(Avg('telemetry__lon'))
        point_dict['coordinates'] = [
            lat['telemetry__lat__avg'], lon['telemetry__lon__avg']]
        # print(point_dict['data']['0']['coordinates'])
        # lidar_points.lidar_points_to_long_lat(0, 0, 0, 0, 0, 0)
        point_dict = json.dumps(point_dict)
        return HttpResponse(point_dict)

    def point_db_to_json(self, points):
        point_id = 0
        point_dict = {}
        list_colors = []
        for point in points:
            data_point = point.points.split('|')
            for loop_data in data_point:
                data = loop_data.split(',')
                if data != ['']:
                    list_colors.append(int(data[3]))
                    list_colors.append(int(data[4]))
                    list_colors.append(int(data[5]))
        color_max = max(list_colors)
        color_min = min(list_colors)
        for point in points:
            data_point = point.points.split('|')
            for loop_data in data_point:
                data = loop_data.split(',')
                if data != ['']:
                    data = [float(x) for x in data]
                    point_dict[str(point_id)] = {}
                    point_dict[str(point_id)]['coordinates'] = data[0:3]
                    point_dict[str(point_id)]['color'] = [
                        (int(data[3]) - color_min), (int(data[4]) - color_min), (int(data[5]) - color_min)]
                    point_id = point_id+1
        return point_dict


class Lidar_process_cloud_points(LoginRequiredMixin, generics.ListAPIView):
    def post(self, request, *args, **kwargs):
        mesh_id = request.POST.get('mesh_id')
        mesh_object = LidarPointSession.objects.get(id=mesh_id)
        works = self.run_lidar_point_triangle(mesh_object)
        if works == True:
            mesh_object_update = LidarPointSession.objects.filter(id=mesh_id)
            mesh_object_update.update(is_process=True)
            return HttpResponse(200, status=status.HTTP_200_OK)
        return HttpResponse(500, status=status.HTTP_200_OK)

    def get(self, request, *args, **kwargs):
        lidar_session_list = list(LidarPointSession.objects.filter(operation=Operation.objects.get(
            operation_name=self.kwargs.get("operation_name")), is_process=False, is_active=False).values())
        for session in lidar_session_list:
            session['start_time'] = str(session['start_time'])
            if session['end_time'] != None:
                session['end_time'] = str(session['end_time'])
        lidar_session_list = json.dumps(lidar_session_list)
        return HttpResponse(lidar_session_list)

    def run_lidar_point_triangle(self, lidar_object):
        list_all_points = []
        list_all_color_points = []
        list_of_lidar_points = list(
            LidarPoint.objects.filter(lidar_point_session=lidar_object))
        list_colors = []
        for point in list_of_lidar_points:
            data_point = point.points.split('|')
            for loop_data in data_point:
                data = loop_data.split(',')
                if data != ['']:
                    list_colors.append(int(data[3]))
                    list_colors.append(int(data[4]))
                    list_colors.append(int(data[5]))
        color_max = max(list_colors)
        color_min = min(list_colors)
        for point in list_of_lidar_points:
            data_point = point.points.split('|')
            for loop_data in data_point:
                data = loop_data.split(',')
                if data != ['']:
                    data = [float(x) for x in data]
                    list_all_points.append(
                        [data[0], data[1], data[2]])
                    list_all_color_points.append([
                        (int(data[3]) - color_min) / (color_max-color_min), (int(data[4]) - color_min) / (color_max-color_min), (int(data[5]) - color_min) / (color_max-color_min)])

        for loop_data in list_all_points:
            loop_data = np.asarray(loop_data)
        list_all_points = np.asarray(list_all_points)

        for loop_data in list_all_color_points:
            loop_data = np.asarray(loop_data)
        list_all_color_points = np.asarray(list_all_color_points)

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(list_all_points)
            pcd.colors = o3d.utility.Vector3dVector(list_all_color_points)
            # o3d.visualization.draw_geometries([pcd], point_show_normal=True)

            alpha = 0.1
            tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(
                pcd)

            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
                pcd, alpha, tetra_mesh, pt_map)
            # mesh.vertex_colors = o3d.utility.Vector3dVector(
            #     list_all_color_points)
            if not mesh.has_vertex_normals():
                mesh.compute_vertex_normals()
            if not mesh.has_triangle_normals():
                mesh.compute_triangle_normals()
            # o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
            if not os.path.exists(default_storage.path('triangle_mesh')):
                os.makedirs(default_storage.path('triangle_mesh'))
            if os.path.exists(default_storage.path('triangle_mesh/'+str(lidar_object.id)+'.glb')):
                os.remove(default_storage.path(
                    'triangle_mesh/'+str(lidar_object.id)+'.glb'))
            o3d.io.write_triangle_mesh(
                default_storage.path('triangle_mesh/'+str(lidar_object.id)+'.glb'), mesh)
            return True
        except Exception as e:
            print(e)
            return False


class FlyingReportAPIView(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        AvailableDroneList = list(Drone.objects.filter(
            operation__operation_name=operation_name).values())
        listDrones = []
        for drone in AvailableDroneList:
            listDrones.append({"drone_name": drone['drone_name'],
                               "latitude": Telemetry.objects.filter(drone__drone_name=drone['drone_name']).last().lat,
                               "longitude": Telemetry.objects.filter(drone__drone_name=drone['drone_name']).last().lon})
        return render(request, 'aiders/flying_report.html', {'list_of_drones': listDrones, 'available_drones': json.dumps(listDrones), 'operation_name': operation_name, 'form': FlyingReportForm()})

    def post(self, request, *args, **kwargs):
        user = request.user.username
        if request.POST.get('form_selection') != 'custom':
            drone = request.POST.get('form_selection')
        else:
            drone = 'Unknown'
        operation_name = self.kwargs.get("operation_name")
        form = FlyingReportForm(request.POST)
        if form.is_valid():
            latitude = request.POST.get('latitude')
            longitude = request.POST.get('longitude')
            altitude = request.POST.get('altitude')
            radius = request.POST.get('radius')
            buffer_altitude = request.POST.get('buffer_altitude')
            buffer_radius = request.POST.get('buffer_radius')
            start_date = request.POST.get('start_date_time')
            end_date = request.POST.get('end_date_time')
            start_date = datetime.datetime.strptime(
                start_date, '%Y-%m-%dT%H:%M')
            end_date = datetime.datetime.strptime(
                end_date, '%Y-%m-%dT%H:%M')
            path = 'daily_fly_notams/notams' + \
                str(len(FlyingReport.objects.all()))+'.pdf'
            flying_report.main(user, drone, operation_name, latitude, longitude, altitude, radius,
                               buffer_altitude, buffer_radius, start_date, end_date, path)
            try:
                drone = Drone.objects.get(drone_name=drone)
            except Drone.DoesNotExist:
                drone = None

            FlyingReport.objects.create(user=request.user, drone=drone, operation=Operation.objects.get(operation_name=operation_name), latitude=latitude, longitude=longitude, altitude=altitude,
                                        radius=radius, buffer_altitude=buffer_altitude, buffer_radius=buffer_radius, start_date_time=start_date, end_date_time=end_date, file_path=path)
            response = open(default_storage.path(path), 'rb')
            return FileResponse(response)
        operation_name = self.kwargs.get("operation_name")
        AvailableDroneList = list(Drone.objects.filter(
            operation__operation_name=operation_name).values())
        listDrones = []
        for drone in AvailableDroneList:
            listDrones.append({"drone_name": drone['drone_name'],
                               "latitude": Telemetry.objects.filter(drone__drone_name=drone['drone_name']).last().lat,
                               "longitude": Telemetry.objects.filter(drone__drone_name=drone['drone_name']).last().lon})
        return render(request, 'aiders/flying_report.html', {'list_of_drones': listDrones, 'available_drones': json.dumps(listDrones), 'operation_name': operation_name, 'form': form})


class FlyingReportTableAPIView(LoginRequiredMixin, generics.ListAPIView):
    def get(self, request, *args, **kwargs):
        operation_name = self.kwargs.get("operation_name")
        fly_reports = FlyingReport.objects.filter(
            operation=Operation.objects.get(operation_name=operation_name))
        return render(request, 'aiders/flying_reports.html', {'flying_reports': fly_reports, 'operation_name': operation_name})


class DroneMovementAPIView(LoginRequiredMixin, generics.ListAPIView):
    def create_data_to_db(data, drone_name):
        if(DroneMovement.objects.get(seq=data.seq, uid=data.uid, time_stamp=data.time_stamp) != None):
            DroneMovement.objects.create(
                seq=data.seq,
                uid=data.uid,
                time_stamp=data.timestamp,
                drone=Drone.objects.get(drone_name=drone_name),
                flight_logic_state=data.flight_logic_state,
                wind_speed=data.wind_speed,
                wind_angle=data.wind_angle,
                battery_voltage=data.battery_voltage,
                battery_current=data.battery_current,
                position_x=data.position_x,
                position_y=data.position_y,
                position_z=data.position_z,
                altitude=data.altitude,
                orientation_x=data.orientation_x,
                orientation_y=data.orientation_y,
                orientation_z=data.orientation_z,
                orientation_w=data.orientation_w,
                velocity_x=data.velocity_x,
                velocity_y=data.velocity_y,
                velocity_z=data.velocity_z,
                angular_x=data.angular_x,
                angular_y=data.angular_y,
                angular_z=data.angular_z,
                linear_acceleration_x=data.linear_acceleration_x,
                linear_acceleration_y=data.linear_acceleration_y,
                linear_acceleration_z=data.linear_acceleration_z,
                payload=data.payload,
            )


def settings_view(request):
    if request.user.is_authenticated:
        if request.method == 'GET':
            use_online_map = UserPreferences.objects.get(
                user=request.user).use_online_map
            return render(request, 'aiders/settings.html', {'use_online_map': use_online_map})
        elif request.method == 'POST':
            selectedVal = request.POST.get('map_mode_dropdown')
            use_online_map = True if selectedVal == Constants.ONLINE_MAP_MODE else False
        UserPreferences.objects.filter(user=request.user).update(
            use_online_map=use_online_map)
        return render(request, 'aiders/settings.html', {'use_online_map': use_online_map})


# Delete later
class TestingBuildMap(LoginRequiredMixin, View):
    def get(self, request, *args, **kwargs):
        with open('aiders/buildmapimages_db.csv', newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
            active = True
            for row in spamreader:
                my_list = " ".join(row).split(",")
                # if my_list[1] == 'matrice300_5807/16650632878898530304.jpeg':
                #     active = True
                # if my_list[1] == 'matrice300_5807/16650634753086040064.jpeg':
                #     active = False
                # # print(active)
                print(my_list[1])
                print(my_list[1] ==
                      'matrice300_5807/16680725277336719360.jpeg')
                if my_list[1] == 'Build_Maps_matrice300_5807_2022-11-10_11.28.46/16680725277336719360.jpeg':
                    if active:
                        newBearing = (float(my_list[8])+float(my_list[14]))/2
                        long_lat = my_list[6].split(" ")
                        long_lat[1] = float(long_lat[1])
                        long_lat[0] = float(long_lat[0])
                        # long_lat[1], long_lat[0]=self.test_my_high_accuracy(float(long_lat[1]),float(long_lat[0]), float(my_list[7]), float(my_list[10]), float(my_list[9]), newBearing)
                        print(long_lat[1], long_lat[0])
                        destinations = img_georeference.calcPoints(float(long_lat[1]), float(
                            long_lat[0]), newBearing, float(my_list[7]), my_list[1], 'none', 'Zenmuse_H20T')
                        # print(destinations)
                        # print(float(my_list[8])+newBearing)
                        # print(float(my_list[10]), float(my_list[9]))
                        try:
                            print(
                                Point(float(long_lat[0]), float(long_lat[1])))
                            image = BuildMapImage.objects.create(
                                path=my_list[1],
                                top_left=Point(
                                    destinations[2].longitude, destinations[2].latitude),
                                top_right=Point(
                                    destinations[0].longitude, destinations[0].latitude),
                                bottom_left=Point(
                                    destinations[1].longitude, destinations[1].latitude),
                                bottom_right=Point(
                                    destinations[3].longitude, destinations[3].latitude),
                                centre=Point(
                                    float(long_lat[0]), float(long_lat[1])),
                                altitude=Decimal(my_list[7]),
                                bearing=Decimal(
                                    (float(my_list[8])+float(my_list[14]))/2),
                                d_roll=None,
                                d_pitch=None,
                                d_yaw=None,
                                g_roll=None,
                                g_pitch=None,
                                g_yaw=None,
                                session_id=1
                            )
                            print('working')
                            # active=False
                        except Exception as e:
                            print(e)
            return HttpResponse(status=status.HTTP_200_OK)
