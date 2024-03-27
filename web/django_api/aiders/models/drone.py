import contextlib
import datetime
import json

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone
from django.db.models import Count
from logic.Constants import Constants


from .detection import Detection
from .operation import Operation, OnlineSession

class Drone(models.Model):
    drone_name = models.CharField(max_length=100, unique=True)
    ip = models.CharField(max_length=100, blank=True, null=True)
    port = models.CharField(max_length=6, blank=True, null=True)
    live_stream_url = models.CharField(max_length=200, blank=True, null=True)
    model = models.CharField(max_length=200)
    type = models.CharField(max_length=200, null=True)
    camera_model = models.CharField(max_length=200)
    time = models.DateTimeField(null=True, blank=True)
    operation = models.ForeignKey("Operation", on_delete=models.SET_NULL, blank=True, null=True)
    is_connected_with_platform = models.BooleanField()
    mission = models.ForeignKey("Mission", on_delete=models.SET_NULL, blank=True, null=True)
    ballistic_available = models.BooleanField(default=False)
    build_map_activated = models.BooleanField(default=False)
    drone_movement_available = models.BooleanField(default=False)
    lidar_available = models.BooleanField(default=False)
    multispectral_available = models.BooleanField(default=False)
    water_sampler_available = models.BooleanField(default=False)
    weather_station_available = models.BooleanField(default=False)
    requested_collaboration = models.BooleanField(default=False, null=True)
    responding_to_collaboration = models.BooleanField(default=False, null=True)

    def save(self, *args, **kwargs):
        if not self.pk:
            super().save(*args, **kwargs)
            Detection.objects.create(
                detection_status=Detection.DetectionStatusChoices.DETECTION_INITIAL_STATUS,
                detection_type_str=Detection.DetectionTypeStrChoices.NO_ACTIVE_DETECTOR,
                drone=self,
            )
        else:
            super().save(*args, **kwargs)
        OnlineSession.update_drone_session(self)

    def getDroneNameById(pk):
        try:
            return Drone.objects.get(pk=pk).drone_name
        except Drone.DoesNotExist:
            return None

    def getDroneIdByName(drone_name):
        try:
            return Drone.objects.get(drone_name=drone_name).id
        except Drone.DoesNotExist:
            return None

    def getDroneFromIdToJsonFormat(droneId):
        deserialized_data = json.loads(coreDjangoSerializers.serialize(
            "json", [Drone.objects.get(pk=droneId)]))
        # Convert time, start_time, and end_time to timestamps
        for record in deserialized_data:
            fields = record["fields"]
            if "ip" in fields:
                del fields["ip"]
            if "time" in fields:
                del fields["time"]
            if "mission" in fields and fields["mission"] is None:
                fields["mission"] = ""
            if "operation" in fields and fields["operation"] is None:
                fields["operation"] = ""
            fields["pk"] = record["pk"]
        return [record["fields"] for record in deserialized_data]

    def __str__(self) -> str:
        return self.drone_name


class Telemetry(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    battery_percentage = models.FloatField()
    gps_signal = models.FloatField()
    satellites = models.IntegerField()
    heading = models.FloatField()
    velocity = models.FloatField()
    homeLat = models.FloatField()
    homeLon = models.FloatField()
    lat = models.FloatField()
    lon = models.FloatField()
    alt = models.FloatField()
    drone_state = models.CharField(max_length=100)
    secondsOn = models.FloatField()
    gimbal_angle = models.FloatField()
    water_sampler_in_water = models.BooleanField(default=False)
    vtol_state = models.CharField(max_length=50, null=True)
    fov_coordinates = models.CharField(max_length=250, null=True)
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    mission_log = models.ForeignKey(
        "MissionLog", on_delete=models.CASCADE, blank=True, null=True)

    def save(self, *args, **kwargs):
        with contextlib.suppress(Exception):
            if Telemetry.objects.filter(drone=self.drone).last().drone_state == "In_Mission" and self.drone_state == "Flying":
                MissionLog_object = MissionLog.objects.filter(
                    drone=self.drone, action="START_MISSION").last()
                MissionLog_object.pk = None
                MissionLog_object.time = None
                MissionLog_object.action = "FINISH_MISSION"
                MissionLog_object.save()
        self.operation = self.drone.operation
        super(Telemetry, self).save()

    def getAllTelemetriesOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return Telemetry.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def getAllDronesOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return (
            Telemetry.objects.filter(
                operation__operation_name=operationName,
                time__gte=startTime,
                time__lte=endTime,
                drone__isnull=False,
            )
            .values_list("drone", flat=True)
            .distinct()
        )

    def convertListTelemetryToJsonFormat(listOfTelemetries):
        listOfTelemetryJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfTelemetries))
        for recordOfTelemetriesJson in listOfTelemetryJson:
            fields = recordOfTelemetriesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "droneTelemetry"
            if fields["mission_log"] == None:
                fields["mission_log"] = ""
            fields["drone"] = Drone.getDroneNameById(fields["drone"])
        return [recordOfTelemetriesJson["fields"] for recordOfTelemetriesJson in listOfTelemetryJson]


class ErrorMessage(models.Model):
    time = models.DateTimeField(auto_now=True)
    message = models.CharField(max_length=255, blank=False)
    drone = models.ForeignKey(
        Drone, on_delete=models.CASCADE, blank=True, null=True)
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)

    def save(self, *args, **kwargs):
        from ..consumers import ErrorMsg

        if self.drone.operation != None:
            ErrorMsg.set_message(
                self.drone.operation.operation_name, self.message)
            self.operation = self.drone.operation
        super(ErrorMessage, self).save(*args, **kwargs)

    def getAllErrorMessageOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return ErrorMessage.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def getAllErrorMessageOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return ErrorMessage.objects.filter(
            drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListErrorMessageToJsonFormat(listOfErrorMessages):
        listOfErrorMessagesJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfErrorMessages))
        for recordOfErrorMessagesJson in listOfErrorMessagesJson:
            fields = recordOfErrorMessagesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            for field in fields:
                if fields[field] == None:
                    fields[field] = "None"
            fields["type"] = "errorMessage"
        return [recordOfErrorMessagesJson["fields"] for recordOfErrorMessagesJson in listOfErrorMessagesJson]


class ControlDevice(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    cpu_usage = models.FloatField()
    cpu_core_usage = models.CharField(max_length=255, blank=False)
    cpu_core_frequency = models.CharField(max_length=255, blank=False)
    cpu_temp = models.FloatField()
    cpu_fan_RPM = models.FloatField()
    gpu_usage = models.FloatField()
    gpu_frequency = models.FloatField()
    gpu_temp = models.FloatField()
    ram_usage = models.FloatField()
    swap_usage = models.FloatField()
    swap_cache = models.FloatField()
    emc_usage = models.FloatField()


class WaterSampler(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    user = models.ForeignKey("User", on_delete=models.CASCADE)
    telemetry = models.ForeignKey(Telemetry, on_delete=models.CASCADE)


class Ballistic(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    user = models.ForeignKey("User", on_delete=models.CASCADE)
    telemetry = models.ForeignKey(Telemetry, on_delete=models.CASCADE)


class BuildMapSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    folder_path = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        self.folder_path = self.folder_path + \
            datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
        super(BuildMapSession, self).save()

    def getLatestActiveSessionIdByDroneId(_droneId):
        return BuildMapSession.objects.filter(drone_id=_droneId, is_active=True).last()

    def getInactiveSessionAndNumberOfImagesByOperationId(_operationId):
        # Retrieve inactive sessions with end_time as None
        inactiveSessions = BuildMapSession.objects.filter(
            is_active=False, operation_id=_operationId, end_time__isnull=False
        )
        # Annotate the query with the count of related images for each session
        inactiveSessionsWithImageCount = inactiveSessions.annotate(image_count=Count("buildmapimage"))
        return inactiveSessionsWithImageCount

    def createBuildMapSessionByUserIdOperationNameDrone(_user, _operationName, _drone):
        buildMapSession = BuildMapSession.objects.create(
            user=_user,
            operation=Operation.objects.get(operation_name=_operationName),
            is_active=True,
            drone=_drone,
            folder_path=Constants.BUILD_MAP_DIR_PREFIX + _drone.drone_name + "_",
        )
        return buildMapSession.id

    def deactivateBuildMapSessionByOperationNameDrone(_operationName, _drone):
        buildMapSession = BuildMapSession.objects.filter(
            operation=Operation.objects.get(operation_name=_operationName), drone=_drone, is_active=True
        )
        buildMapSessionId = buildMapSession[0].id
        buildMapSession.update(end_time=datetime.datetime.now(tz=Constants.CYPRUS_TIMEZONE_OBJ), is_active=False)
        print(buildMapSession, flush=True)
        return buildMapSessionId

class BuildMapImage(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    path = models.CharField(max_length=255, null=False)
    top_left = models.PointField()
    top_right = models.PointField()
    bottom_left = models.PointField()
    bottom_right = models.PointField()
    centre = models.PointField()
    altitude = models.DecimalField(max_digits=5, decimal_places=1)
    bearing = models.DecimalField(max_digits=17, decimal_places=14)
    d_roll = models.FloatField(blank=True, null=True)
    d_pitch = models.FloatField(blank=True, null=True)
    d_yaw = models.FloatField(blank=True, null=True)
    g_roll = models.FloatField(blank=True, null=True)
    g_pitch = models.FloatField(blank=True, null=True)
    g_yaw = models.FloatField(blank=True, null=True)
    session = models.ForeignKey(BuildMapSession, on_delete=models.CASCADE)

    def all_fields(self):
        return [
            "time",
            "path",
            "top_left",
            "top_right",
            "bottom_left",
            "bottom_right",
            "centre",
            "altitude",
            "bearing",
            "d_roll",
            "d_pitch",
            "d_yaw",
            "g_roll",
            "g_pitch",
            "g_yaw",
            "session",
        ]

    def getAllBuildMapImagesOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return BuildMapImage.objects.filter(
            session__drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListBuildMapImagesToJsonFormat(listOfBuildMapImages):
        listOfBuildMapImagesJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfBuildMapImages))
        for recordOfBuildMapImagesJson in listOfBuildMapImagesJson:
            fields = recordOfBuildMapImagesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            for field in fields:
                if field == "top_left" or field == "top_right" or field == "bottom_left" or field == "bottom_right" or field == "centre":
                    coordinate_string = fields[field].replace(
                        "SRID=4326;POINT (", "").replace(")", "")
                    latitude, longitude = coordinate_string.split(" ")
                    fields[field] = [float(latitude), float(longitude)]
                if fields[field] == None:
                    fields[field] = "None"
            fields["id"] = str(recordOfBuildMapImagesJson["pk"])
            fields["type"] = "droneBuildMapImage"
        return [recordOfBuildMapImagesJson["fields"] for recordOfBuildMapImagesJson in listOfBuildMapImagesJson]


class BuildMapAdvanceSession(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    folder_path = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        self.folder_path = self.folder_path + \
            datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
        super(BuildMapAdvanceSession, self).save()


class BuildMapAdvanceImage(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    path = models.CharField(max_length=255, null=False)
    session = models.ForeignKey(
        "BuildMapAdvanceSession", on_delete=models.CASCADE)

class MissionPoint(models.Model):
    point = models.PointField()
    # mission = models.ForeignKey(Mission,on_delete=models.CASCADE)

    def getMissionPointsDataById(pk):
        missionPointData = MissionPoint.objects.get(pk=pk)
        return [float(missionPointData.point[0]), float(missionPointData.point[1])]


class Mission(models.Model):
    NORMAL_MISSION = "NORMAL_MISSION"
    SEARCH_AND_RESCUE_MISSION = "SEARCH_AND_RESCUE_MISSION"
    GRID_MISSION = "GRID_MISSION"

    MISSION_TYPE_CHOICES = [(NORMAL_MISSION, NORMAL_MISSION), (
        SEARCH_AND_RESCUE_MISSION, SEARCH_AND_RESCUE_MISSION), (GRID_MISSION, GRID_MISSION)]

    mission_type = models.CharField(
        max_length=150, choices=MISSION_TYPE_CHOICES)
    time = models.DateTimeField(auto_now_add=True)

    mission_completed = models.BooleanField(default=False)
    grid = models.BooleanField(default=False)
    captureAndStoreImages = models.BooleanField(default=False)
    mission_points = models.ManyToManyField(MissionPoint, blank=True)
    mission_speeds = models.CharField(max_length=400)
    mission_gimbal = models.CharField(max_length=400, blank=True)
    repeat = models.IntegerField(blank=True)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    user = models.ForeignKey("User", on_delete=models.CASCADE, blank=True)

    def fields(self):
        return [f.name for f in self._meta.fields + self._meta.many_to_many]

    def getMissionDataById(pk):
        try:
            mission_json = json.loads(coreDjangoSerializers.serialize(
                "json", [Mission.objects.get(pk=pk)]))[0]["fields"]
            for index, point in enumerate(mission_json["mission_points"]):
                mission_json["mission_points"][index] = MissionPoint.getMissionPointsDataById(
                    point)
            return mission_json
        except Mission.DoesNotExist:
            return None

    """The following constraint will currently remain as comment because it is wrong. The constraint
        should be: "Do not allow a mission to be active, under the same drone, under the same operation,
        more than once". However, the way it is currently implemented, it does not consider the drone.
    """
    # class Meta:
    #
    #     '''
    #     Do not allow two missions to be active (i.e running) under the same operation.
    #     When active=NULL that means mission is not active.
    #     In this way, we can have one active mission per operation due to the following constraint,
    #     but as many inactive as we want because constraint won't apply,
    #     since each NULL is different.
    #     Reference: https://stackoverflow.com/questions/47550176/mysql-unique-constraint-based-on-column-value
    #     '''
    #     constraints = [
    #         models.UniqueConstraint
    #         (fields=['active','operation'], name='user_operation_constraint')
    #     ]
    #
    # def save(self, *args, **kwargs):
    #
    #     '''
    #     Since we avoid the False value for inactive missions and replaced it with NULL,
    #     in case the "false" value is mistakenly inserted
    #     in the database, convert it to Null.
    #     '''
    #     if  not self.active:
    #         self.active = None
    #     return  super(Mission, self).save(*args, **kwargs)
    #


class MissionLog(models.Model):
    START_MISSION = "START_MISSION"
    PAUSE_MISSION = "PAUSE_MISSION"
    RESUME_MISSION = "RESUME_MISSION"
    CANCEL_MISSION = "CANCEL_MISSION"
    FINISH_MISSION = "FINISH_MISSION"

    MISSION_ACTION_CHOICES = [
        (START_MISSION, START_MISSION),
        (PAUSE_MISSION, PAUSE_MISSION),
        (RESUME_MISSION, RESUME_MISSION),
        (CANCEL_MISSION, CANCEL_MISSION),
        (FINISH_MISSION, FINISH_MISSION),
    ]
    mission = models.ForeignKey(Mission, on_delete=models.CASCADE)
    time = models.DateTimeField(auto_now_add=True)
    action = models.CharField(max_length=80, choices=MISSION_ACTION_CHOICES)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    user = models.ForeignKey("User", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    algorithm = models.ForeignKey(
        "Algorithm", on_delete=models.CASCADE, blank=True, null=True)

    def getAllMissionLogOfDroneBetweenTwoTime(droneName, startTime, endTime):
        return MissionLog.objects.filter(
            drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListMissionLogToJsonFormat(listOfMissionLogs):
        listOfMissionLogsJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfMissionLogs))
        for recordOfMissionLogsJson in listOfMissionLogsJson:
            fields = recordOfMissionLogsJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "mission"
            if fields["algorithm"] == None:
                fields["algorithm"] = "None"
            fields["drone"] = Drone.getDroneNameById(fields["drone"])
            fields["mission"] = Mission.getMissionDataById(fields["mission"])
        return [recordOfMissionLogsJson["fields"] for recordOfMissionLogsJson in listOfMissionLogsJson]



class MavlinkLog(models.Model):
    time = models.DateTimeField(auto_now=True)
    message = models.CharField(max_length=255, blank=False)
    drone = models.ForeignKey(
        Drone, on_delete=models.CASCADE, blank=True, null=True)
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
