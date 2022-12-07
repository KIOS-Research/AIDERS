import datetime
import os

from django.contrib.auth import get_user_model
from django.contrib.auth.models import AbstractUser, Group, Permission
from django.contrib.contenttypes.models import ContentType
from django.contrib.gis.db import models
from django.core.validators import (MaxValueValidator, MinValueValidator,
                                    RegexValidator)
from django.db import connection, transaction
from django.db.models import F, Q
from django.forms import ValidationError
from django.utils import timezone
from logic.Constants import Constants


class User(AbstractUser):
    # On which operation the current user is currently joined. It can be NULL
    joined_operation = models.ForeignKey(
        'Operation', on_delete=models.SET_NULL, blank=True, null=True)

    def update_permissions(pk, edit_data, value):
        user = User.objects.get(pk=pk)
        if edit_data == "permission_edit_permissions":
            my_group = Group.objects.get(name='edit_permissions')
            if value:
                my_group.user_set.add(user)
            else:
                my_group.user_set.remove(user)
        elif edit_data == "permission_create_operations":
            my_group = Group.objects.get(name='create_operations')
            if value:
                my_group.user_set.add(user)
            else:
                my_group.user_set.remove(user)
        else:
            print(edit_data)

    # class Meta:
    # constraints = [models.CheckConstraint(check=models.Q(age__gte=18), name='is_adult')]

    # class Meta:
    #
    #     '''User should not be allowed to join two operations at the same time.'''
    #     constraints = [
    #         models.UniqueConstraint
    #         (fields=['id', 'joined_operation'], name='user_operation_constraint')
    #     ]


class UserLog(models.Model):
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    time = models.DateTimeField(auto_now=True)
    event = models.CharField(max_length=50, blank=False)


class Operation(models.Model):
    alphanumeric = RegexValidator(r'^[0-9a-zA-Z !@#$%^&*()_+=/.,?:;| ]*$',
                                  'Only alphanumeric characters are allowed and some symbols.')

    created_at = models.DateTimeField(auto_now_add=True, auto_now=False)
    ended_at = models.DateTimeField(null=True, blank=True)
    active = models.BooleanField(default=True)
    operator = models.ForeignKey(get_user_model(), related_name='operations',
                                 on_delete=models.CASCADE)
    drones_to_operate = models.ManyToManyField('Drone', related_name='+')
    operation_name = models.CharField(max_length=100, validators=[
                                      alphanumeric], unique=True)
    location = models.CharField(max_length=100)
    # radius = models.FloatField()
    description = models.TextField(blank=True, null=True)

    def save(self, *args, **kwargs):
        just_created = False
        if self.operation_name and not self.active:  # Someone just stopped this operation
            # We have to know the time the operation finished
            self.ended_at = datetime.datetime.now(
                tz=Constants.CYPRUS_TIMEZONE_OBJ)
            User.objects.filter(joined_operation=self).update(
                joined_operation=None)  # If an operation is not active (i.e just stopped), NO ONE should be a member of this operation.
            super().save(*args, **kwargs)
        else:
            if not self.pk:
                # This code only happens if the object is not in the database yet. A.k.a just created
                just_created = True

            super().save(*args, **kwargs)  # First save the current operation

            if just_created:
                '''
                    Someone had to create this operation. Once it is created it is also automatically started.
                    For this reason the creator has to automatically join the newly created operation
                '''
                User.objects.filter(pk=self.operator.id).update(
                    joined_operation=self)  # Now assign to the user the just saved operation

    class Meta:
        permissions = (
            ('join_operation', 'Join operation'),
            ('edit_operation', 'Edit operation'),
        )

    def __str__(self) -> str:
        return self.operation_name


class Drone(models.Model):
    drone_name = models.CharField(max_length=100, unique=True)
    ip = models.CharField(max_length=100, blank=True, null=True)
    model = models.CharField(max_length=200)
    camera_model = models.CharField(max_length=200)
    joined_at = models.DateTimeField(null=True, blank=True)
    operation = models.ForeignKey(
        Operation, on_delete=models.SET_NULL, blank=True, null=True)
    is_connected_with_platform = models.BooleanField()
    mission = models.ForeignKey(
        'Mission', on_delete=models.SET_NULL, blank=True, null=True)
    ballistic_available = models.BooleanField(default=False)
    build_map_activated = models.BooleanField(default=False)
    drone_movement_available = models.BooleanField(default=False)
    lidar_available = models.BooleanField(default=False)
    multispectral_available = models.BooleanField(default=False)
    water_sampler_available = models.BooleanField(default=False)
    weather_station_available = models.BooleanField(default=False)

    def save(self, *args, **kwargs):
        if not self.pk:  # Drone just created
            # Create an initial detection object for this drone
            super().save(*args, **kwargs)
            Detection.objects.create(
                detection_status=Detection.DetectionStatusChoices.DETECTION_INITIAL_STATUS,
                detection_type_str=Detection.DetectionTypeStrChoices.NO_ACTIVE_DETECTOR,
                drone=self
            )
        else:
            super().save(*args, **kwargs)
        if not self.operation:
            DroneToOperationLog.objects.create(drone=self, operation=None)
        else:
            DroneToOperationLog.objects.create(
                drone=self, operation=self.operation)

    def __str__(self) -> str:
        return self.drone_name


class DroneToOperationLog(models.Model):
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    time = models.DateTimeField(auto_now=True)
    operation = models.ForeignKey(
        Operation, on_delete=models.CASCADE, null=True, blank=True)


class DetectionSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    latest_frame_url = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        # At any given time, there should be only one active Detection Session per
        # drone, because one drone cannot initiate two detection sessions at once
        if not self.is_active:
            return super(DetectionSession, self).save(*args, **kwargs)

        # If for some reason it happens that other Detection Sessions for this drone
        # are still true (there shouldn't be because starting a new detection session
        # means that the previous one was closed) make them all false
        with transaction.atomic():
            DetectionSession.objects.filter(
                is_active=True, drone=self.drone).update(is_active=False)
            return super(DetectionSession, self).save(*args, **kwargs)


class Detection(models.Model):
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)

    class DetectionStatusChoices(models.TextChoices):
        DETECTION_DISCONNECTED = 'DETECTION_DISCONNECTED', 'DETECTION_DISCONNECTED'
        DETECTION_CONNECTED = 'DETECTION_CONNECTED', 'DETECTION_CONNECTED'
        DETECTION_WANT_TO_CONNECT = 'DETECTION_WANT_TO_CONNECT', 'DETECTION_WANT_TO_CONNECT'
        DETECTION_WANT_TO_DISCONNECT = 'DETECTION_WANT_TO_DISCONNECT', 'DETECTION_WANT_TO_DISCONNECT'
        DETECTION_INITIAL_STATUS = 'DETECTION_INITIAL_STATUS', 'DETECTION_INITIAL_STATUS'

    detection_status = models.CharField(
        max_length=50,
        choices=DetectionStatusChoices.choices,
        default=DetectionStatusChoices.DETECTION_INITIAL_STATUS,
    )

    class DetectionTypeStrChoices(models.TextChoices):

        VEHICLE_DETECTOR = 'VEHICLE_DETECTOR', 'VEHICLE_DETECTOR'
        GENERAL_DETECTOR = 'GENERAL_DETECTOR', 'GENERAL_DETECTOR'
        PERSON_DETECTOR = 'PERSON_DETECTOR', 'PERSON_DETECTOR'
        PERSON_AND_VEHICLE_DETECTOR = 'VEHICLE_PERSON_DETECTOR', 'VEHICLE_PERSON_DETECTOR'
        NO_ACTIVE_DETECTOR = 'NO_ACTIVE_DETECTOR', 'NO_ACTIVE_DETECTOR'

    class DetectionModelChoices(models.TextChoices):
        # Detects cars from high alt. Works ONLY with GPU (faster on GPU only)
        YOLO = 'YOLO', 'YOLO'
        # for CPU (can also be ran in GPU). Detects cars from high alt
        YOLOCV = 'YOLOOCV', 'YOLOOCV'
        YOLOV5 = 'YOLOV5', 'YOLOV5'  # Detects cars from high alt. Works with GPU or CPU
        YOLO_BATCH = 'YOLO_BATCH', 'YOLO_BATCH'  # People + vehicles. Works on GPU only
        NO_ACTIVE_MODEL = 'NO_ACTIVE_MODEL', 'NO_ACTIVE_MODEL'

    detection_type_str = models.CharField(
        max_length=50,
        choices=DetectionTypeStrChoices.choices,
        default=DetectionTypeStrChoices.NO_ACTIVE_DETECTOR,
    )

    detection_model = models.CharField(
        max_length=50,
        choices=DetectionModelChoices.choices,
        default=DetectionModelChoices.NO_ACTIVE_MODEL,
    )

    gpuEnabled = True if os.environ.get(
        "NVIDIA_AVAILABLE", "0") == "1" else False

    # @property
    # def detection_type(self):
    #     if (self.detection_type_str == Detection.DetectionTypeStrChoices.VEHICLE_DETECTOR):
    #         return Detection.DetectionModelChoices.YOLOCV
    #     elif (self.detection_type_str == Detection.DetectionTypeStrChoices.PERSON_AND_VEHICLE_DETECTOR):
    #         return Detection.DetectionModelChoices.YOLOV5
    #     elif (self.detection_type_str == Detection.DetectionTypeStrChoices.NO_ACTIVE_DETECTOR):
    #         return Detection.DetectionModelChoices.NO_ACTIVE_MODEL


class LiveStreamSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    latest_frame_url = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        # At any given time, there should be only one active Live Stream Session per
        # drone, because one drone cannot initiate two live stream sessions at once
        if not self.is_active:
            return super(LiveStreamSession, self).save(*args, **kwargs)

        # If for some reason it happens that other Live Stream Sessions for this drone
        # are still true (there shouldn't be because starting a new live stream session
        # means that the previous one was closed) make them all false
        with transaction.atomic():
            LiveStreamSession.objects.filter(
                is_active=True, drone=self.drone).update(is_active=False)
            return super(LiveStreamSession, self).save(*args, **kwargs)


def get_upload_path_for_detection_frame(instance, filename):
    return os.path.join(Constants.DETECTION_FRAMES_DIR_NAME_PREFIX + instance.detection_session.drone.drone_name + "_" + str(instance.detection_session.start_time.strftime('%Y-%m-%d_%H.%M.%S')), filename)
    # return os.path.join(Constants.DETECTION_FRAMES_DIR_NAME_PREFIX, filename)


def get_upload_path_for_raw_frame(instance, filename):
    return os.path.join(Constants.RAW_FRAMES_DIR_NAME_PREFIX + instance.live_stream_session.drone.drone_name + "_" + str(instance.live_stream_session.start_time.strftime('%Y-%m-%d_%H.%M.%S')), filename)


class WeatherStation(models.Model):
    # started_at = models.DateTimeField(auto_now_add=True)
    # drone = models.OneToOneField(Drone, on_delete=models.CASCADE, primary_key=True)
    current_time = models.DateTimeField(auto_now_add=True)
    wind_speed = models.FloatField()
    wind_direction = models.FloatField()
    temperature = models.FloatField()
    pressure = models.FloatField()
    humidity = models.FloatField()
    heading = models.FloatField()
    operation = models.ForeignKey(
        Operation, on_delete=models.CASCADE, blank=True, null=True)
    drone = models.ForeignKey(
        Drone, on_delete=models.CASCADE, blank=True, null=True)


class Telemetry(models.Model):
    received_at = models.DateTimeField(auto_now_add=True)
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
    operation = models.ForeignKey(
        Operation, on_delete=models.CASCADE, blank=True, null=True)
    mission_log = models.ForeignKey(
        'MissionLog', on_delete=models.CASCADE, blank=True, null=True)

    def save(self, *args, **kwargs):
        try:
            if(Telemetry.objects.filter(drone=self.drone).last().drone_state == 'In_Mission' and self.drone_state == 'Flying'):
                MissionLog_object = MissionLog.objects.filter(
                    drone=self.drone, action='START_MISSION').last()
                MissionLog_object.pk = None
                MissionLog_object.executed_at = None
                MissionLog_object.action = 'FINISH_MISSION'
                MissionLog_object.save()
        except:
            pass
        self.operation = self.drone.operation
        super(Telemetry, self).save()

    def drone_name(self):
        return self.drone.name


class MissionPoint(models.Model):
    point = models.PointField()
    # mission = models.ForeignKey(Mission,on_delete=models.CASCADE)


class Mission(models.Model):

    NORMAL_MISSION = 'NORMAL_MISSION'
    SEARCH_AND_RESCUE_MISSION = 'SEARCH_AND_RESCUE_MISSION'
    GRID_MISSION = 'GRID_MISSION'

    MISSION_TYPE_CHOICES = [
        (NORMAL_MISSION, NORMAL_MISSION),
        (SEARCH_AND_RESCUE_MISSION, SEARCH_AND_RESCUE_MISSION),
        (GRID_MISSION, GRID_MISSION)
    ]

    mission_type = models.CharField(
        max_length=150, choices=MISSION_TYPE_CHOICES)
    executed_at = models.DateTimeField(auto_now_add=True)

    mission_completed = models.BooleanField(default=False)
    grid = models.BooleanField(default=False)
    captureAndStoreImages = models.BooleanField(default=False)
    mission_points = models.ManyToManyField(MissionPoint, blank=True)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    user = models.ForeignKey(User, on_delete=models.CASCADE, blank=True)

    def fields(self):
        return [f.name for f in self._meta.fields + self._meta.many_to_many]

    '''The following constraint will currently remain as comment because it is wrong. The constraint
        should be: "Do not allow a mission to be active, under the same drone, under the same operation,
        more than once". However, the way it is currently implemented, it does not consider the drone.
    '''
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
    START_MISSION = 'START_MISSION'
    PAUSE_MISSION = 'PAUSE_MISSION'
    RESUME_MISSION = 'RESUME_MISSION'
    CANCEL_MISSION = 'CANCEL_MISSION'
    FINISH_MISSION = 'FINISH_MISSION'

    MISSION_ACTION_CHOICES = [
        (START_MISSION, START_MISSION),
        (PAUSE_MISSION, PAUSE_MISSION),
        (RESUME_MISSION, RESUME_MISSION),
        (CANCEL_MISSION, CANCEL_MISSION),
        (FINISH_MISSION, FINISH_MISSION),
    ]
    mission = models.ForeignKey(Mission, on_delete=models.CASCADE)
    executed_at = models.DateTimeField(auto_now_add=True)
    action = models.CharField(max_length=80, choices=MISSION_ACTION_CHOICES)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    algorithm = models.ForeignKey(
        'Algorithm', on_delete=models.CASCADE, blank=True, null=True)


class RawFrame(models.Model):
    saved_at = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_raw_frame)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    # detection_session = models.ForeignKey(DetectionSession, on_delete=models.CASCADE,  blank=True, null=True)
    live_stream_session = models.ForeignKey(
        LiveStreamSession, on_delete=models.CASCADE)


class DetectionFrame(models.Model):
    saved_at = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_detection_frame)
    # detection_session = models.ForeignKey(DetectionSession, on_delete=models.CASCADE,  blank=True, null=True)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)


class DetectedObject(models.Model):
    detected_at = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)
    lat = models.FloatField()
    lon = models.FloatField()
    label = models.CharField(max_length=100)
    track_id = models.IntegerField()
    distance_from_drone = models.FloatField()
    frame = models.ForeignKey(DetectionFrame, on_delete=models.CASCADE)


class Algorithm(models.Model):

    FIRE_PROPAGATION_ALGORITHM = 'FIRE_PROPAGATION_ALGORITHM'
    CREATE_3D_OBJECT_ALGORITHM = 'CREATE_3D_OBJECT_ALGORITHM'
    CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM = 'CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM'

    ALGORITHM_NAMES = [
        (FIRE_PROPAGATION_ALGORITHM, FIRE_PROPAGATION_ALGORITHM),
        (CREATE_3D_OBJECT_ALGORITHM, CREATE_3D_OBJECT_ALGORITHM),
        (CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM,
         CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM)
    ]

    executed_at = models.DateTimeField(auto_now_add=True)
    algorithm_name = models.CharField(
        max_length=80,
        choices=ALGORITHM_NAMES
    )
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    input = models.JSONField(blank=True, null=True)
    output = models.JSONField(blank=True, null=True)
    canBeLoadedOnMap = models.BooleanField(default=False)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)


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
    session = models.ForeignKey('BuildMapSession', on_delete=models.CASCADE)

    def all_fields(self):
        return['time', 'path', 'top_left', 'top_right', 'bottom_left', 'bottom_right', 'centre', 'altitude', 'bearing', 'd_roll', 'd_pitch', 'd_yaw', 'g_roll', 'g_pitch', 'g_yaw', 'session']


class BuildMapSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    folder_path = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        self.folder_path = self.folder_path + \
            str(datetime.datetime.now().strftime('%Y-%m-%d_%H.%M.%S'))
        super(BuildMapSession, self).save()


class WaterSampler(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    telemetry = models.ForeignKey(Telemetry, on_delete=models.CASCADE)


class Ballistic(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    telemetry = models.ForeignKey(Telemetry, on_delete=models.CASCADE)

# class SnippetSerializer(serializers.Serializer):


class Terminal(models.Model):
    ip_address = models.GenericIPAddressField()
    logged_in = models.BooleanField(default=False)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    os = models.CharField(max_length=100)
    device = models.CharField(max_length=50)
    browser = models.CharField(max_length=100)


class ControlDevice(models.Model):
    received_at = models.DateTimeField(auto_now_add=True)
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


class ErrorMessage(models.Model):
    time = models.DateTimeField(auto_now=True)
    message = models.CharField(max_length=255, blank=False)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    operation = models.ForeignKey(
        Operation, on_delete=models.CASCADE, blank=True, null=True)

    def save(self, *args, **kwargs):
        from .consumers import ErrorMsg
        if(self.drone.operation != None):
            ErrorMsg.set_message(
                self.drone.operation.operation_name, self.message)
            self.operation = self.drone.operation
        super(ErrorMessage, self).save(*args, **kwargs)


class FrontEndUserInput(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    element_name = models.CharField(max_length=255)
    active = models.BooleanField()
    value = models.CharField(max_length=255, blank=True, null=True)


class SystemMonitoring(models.Model):
    time = models.DateTimeField(auto_now=True)

    cpu_usage = models.FloatField()
    cpu_core_usage = models.CharField(max_length=255, blank=False)
    cpu_temp = models.FloatField()

    gpu_usage = models.FloatField(blank=True, null=True)
    gpu_memory = models.FloatField(blank=True, null=True)
    gpu_temp = models.FloatField(blank=True, null=True)

    ram_usage = models.FloatField()
    swap_memory_usage = models.FloatField()
    temp = models.FloatField()
    # Network is in MB/s
    upload_speed = models.FloatField()
    download_speed = models.FloatField()
    total_network = models.FloatField()

    disk_read = models.FloatField()
    disk_write = models.FloatField()

    battery_percentage = models.FloatField(blank=True, null=True)


class LoraTransmitter(models.Model):
    joined_at = models.DateTimeField(auto_now_add=True)
    tagName = models.CharField(max_length=100)
    upTime = models.FloatField()
    operation = models.ForeignKey(
        Operation, on_delete=models.SET_NULL, blank=True, null=True)


class LoraTransmitterLocation(models.Model):
    received_at = models.DateTimeField(auto_now_add=True)
    lat = models.FloatField()
    lon = models.FloatField()
    loraTransmitter = models.ForeignKey(
        LoraTransmitter, on_delete=models.CASCADE)


class LidarPointSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    is_process = models.BooleanField(default=False)


class LidarPoint(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    points = models.TextField(null=False)
    telemetry = models.ForeignKey(Telemetry, on_delete=models.CASCADE)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    lidar_point_session = models.ForeignKey(
        LidarPointSession, on_delete=models.CASCADE)

    def save(self, *args, **kwargs):
        telemetry_data = Telemetry.objects.filter(drone=self.drone).last()
        self.telemetry = telemetry_data
        super(LidarPoint, self).save(*args, **kwargs)


class DroneMovement(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    seq = models.IntegerField()
    uid = models.CharField(max_length=100)
    drone = models.ForeignKey(Drone, on_delete=models.CASCADE)
    time_stamp = models.IntegerField()
    flight_logic_state = models.CharField(max_length=100)
    wind_speed = models.FloatField()
    wind_angle = models.FloatField()
    battery_voltage = models.FloatField()
    battery_current = models.FloatField()
    position_x = models.FloatField()
    position_y = models.FloatField()
    position_z = models.FloatField()
    altitude = models.FloatField()
    orientation_x = models.FloatField()
    orientation_y = models.FloatField()
    orientation_z = models.FloatField()
    orientation_w = models.FloatField()
    velocity_x = models.FloatField()
    velocity_y = models.FloatField()
    velocity_z = models.FloatField()
    angular_x = models.FloatField()
    angular_y = models.FloatField()
    angular_z = models.FloatField()
    linear_acceleration_x = models.FloatField()
    linear_acceleration_y = models.FloatField()
    linear_acceleration_z = models.FloatField()
    payload = models.FloatField()


class FlyingReport(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    drone = models.ForeignKey(
        Drone, on_delete=models.CASCADE, blank=True, null=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    altitude = models.FloatField(validators=[MinValueValidator(1.0), ])
    radius = models.FloatField(validators=[MinValueValidator(1.0), ])
    buffer_altitude = models.FloatField(validators=[MinValueValidator(1.0), ])
    buffer_radius = models.FloatField(validators=[MinValueValidator(1.0), ])
    start_date_time = models.DateTimeField()
    end_date_time = models.DateTimeField()
    file_path = models.CharField(max_length=255, null=False)

    def clean(self):
        now = timezone.now()
        if self.altitude < 0:
            raise ValidationError(
                'The altitude should not be bellow 0.')
        if self.radius < 0:
            raise ValidationError(
                'The radius should not be bellow 0.')
        if self.buffer_altitude < 0:
            raise ValidationError(
                'The buffer altitude should not be bellow 0.')
        if self.buffer_radius < 0:
            raise ValidationError(
                'The buffer radius should not be bellow 0.')
        if self.start_date_time.date() < now.date():
            raise ValidationError(
                'Start date & time of the operation should not be in the past.')
        if self.start_date_time > self.end_date_time:
            raise ValidationError(
                'Start date & time of the operation should be before the end date & time.')
        return super().clean()

    class Meta:
        constraints = [
            models.CheckConstraint(
                check=Q(start_date_time__lte=F('end_date_time')), name='start_before_end')]


def check_groups():
    # Create edit_permissions Group
    try:
        Group.objects.get(name='edit_permissions')
    except:
        new_group, created = Group.objects.get_or_create(
            name='edit_permissions')
        if Permission.objects.filter(codename='edit_permissions').exists():
            permission = Permission.objects.filter(codename='edit_permissions')
        else:
            ct = ContentType.objects.get_for_model(User)
            permission = Permission.objects.create(codename='edit_permissions',
                                                   name='edit_permissions',
                                                   content_type=ct)
        new_group.permissions.add(permission)

    # # Create create_operations Group
    try:
        Group.objects.get(name='create_operations')
    except:
        new_group, created = Group.objects.get_or_create(
            name='create_operations')
        if Permission.objects.filter(codename='create_operations').exists():
            permission = Permission.objects.filter(
                codename='create_operations')
        else:
            ct = ContentType.objects.get_for_model(Operation)
            permission = Permission.objects.create(codename='create_operations',
                                                   name='create_operations',
                                                   content_type=ct)
        new_group.permissions.add(permission)

    # Create join_operations Group
    try:
        Group.objects.get(name='join_operations')
    except:
        new_group, created = Group.objects.get_or_create(
            name='join_operations')
        if Permission.objects.filter(codename='join_operations').exists():
            permission = Permission.objects.filter(codename='join_operations')
        else:
            ct = ContentType.objects.get_for_model(Operation)
            permission = Permission.objects.create(codename='join_operations',
                                                   name='join_operations',
                                                   content_type=ct)
        new_group.permissions.add(permission)


if 'auth_group' in connection.introspection.table_names():
    check_groups()


class UserPreferences(models.Model):
    use_online_map = models.BooleanField(default=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
