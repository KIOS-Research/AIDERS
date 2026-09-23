import datetime
import json
import os

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.db import transaction
from django.db.models import Max
from django.utils import timezone
from logic.Constants import Constants


class DetectionSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    latest_frame_url = models.CharField(max_length=255, null=False)
    video_recording_path = models.CharField(max_length=500, null=True, blank=True)

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

    def getDroneFromDetectionSessionId(detectionSessionId):
        return DetectionSession.objects.get(id=detectionSessionId).drone


class Detection(models.Model):
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)

    class DetectionStatusChoices(models.TextChoices):
        DETECTION_DISCONNECTED = "DETECTION_DISCONNECTED", "DETECTION_DISCONNECTED"
        DETECTION_CONNECTED = "DETECTION_CONNECTED", "DETECTION_CONNECTED"
        DETECTION_WANT_TO_CONNECT = "DETECTION_WANT_TO_CONNECT", "DETECTION_WANT_TO_CONNECT"
        DETECTION_WANT_TO_DISCONNECT = "DETECTION_WANT_TO_DISCONNECT", "DETECTION_WANT_TO_DISCONNECT"
        DETECTION_INITIAL_STATUS = "DETECTION_INITIAL_STATUS", "DETECTION_INITIAL_STATUS"

    detection_status = models.CharField(
        max_length=50,
        choices=DetectionStatusChoices.choices,
        default=DetectionStatusChoices.DETECTION_INITIAL_STATUS,
    )

    class DetectionTypeStrChoices(models.TextChoices):
        VEHICLE_DETECTOR = "VEHICLE_DETECTOR", "VEHICLE_DETECTOR"
        GENERAL_DETECTOR = "GENERAL_DETECTOR", "GENERAL_DETECTOR"
        PERSON_DETECTOR = "PERSON_DETECTOR", "PERSON_DETECTOR"
        PERSON_AND_VEHICLE_DETECTOR = "VEHICLE_PERSON_DETECTOR", "VEHICLE_PERSON_DETECTOR"
        NO_ACTIVE_DETECTOR = "NO_ACTIVE_DETECTOR", "NO_ACTIVE_DETECTOR"
        DISASTER_CLASSIFICATION = "DISASTER_CLASSIFICATION", "DISASTER_CLASSIFICATION"
        CROWD_LOCALIZATION = "CROWD_LOCALIZATION", "CROWD_LOCALIZATION"

    class DetectionModelChoices(models.TextChoices):
        # Detects cars from high alt. Works ONLY with GPU (faster on GPU only)
        YOLO = "YOLO", "YOLO"
        # for CPU (can also be ran in GPU). Detects cars from high alt
        YOLOCV = "YOLOOCV", "YOLOOCV"
        YOLOV5 = "YOLOV5", "YOLOV5"  # Detects cars from high alt. Works with GPU or CPU
        YOLO_BATCH = "YOLO_BATCH", "YOLO_BATCH"  # People + vehicles. Works on GPU only
        # HRNet backbone + ESPCN upsampling head, density-map person localization
        # with LMDS post-processing. Runs in PyTorch eager mode (not onnxruntime).
        # Active model for CROWD_LOCALIZATION.
        HRNET_ESPCN = "HRNET_ESPCN", "HRNET_ESPCN"
        # torchvision EfficientNet-B0 backbone with a custom 4-class classifier
        # head (Earthquake/Fire/Flood/Normal), fine-tuned via transfer learning.
        # Runs in PyTorch eager mode. Active model for DISASTER_CLASSIFICATION.
        EFFICIENTNET_B0 = "EFFICIENTNET_B0", "EFFICIENTNET_B0"
        NO_ACTIVE_MODEL = "NO_ACTIVE_MODEL", "NO_ACTIVE_MODEL"

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

    gpuEnabled = os.environ.get("NVIDIA_AVAILABLE", "0") == "1"


def get_upload_path_for_detection_frame(instance, filename):
    return os.path.join(
        Constants.DETECTION_FRAMES_DIR_NAME_PREFIX
        + instance.detection_session.drone.drone_name
        + "_"
        + str(instance.detection_session.start_time.strftime("%Y-%m-%d_%H.%M.%S")),
        filename,
    )

class DetectionInfo(models.Model):
    msg_identifier = models.CharField(max_length=100)
    uav_status = models.CharField(max_length=20)
    sent_utc = models.DateTimeField()
    district = models.CharField(max_length=100)
    # mission_id = models.CharField(max_length=100)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)
    
    def getAllDetectionInfoByDroneIdAndSessionId(drone_id, session_id):

        return DetectionInfo.objects.filter(
                drone_id=drone_id, detection_session_id=session_id
        )
    
   

class DetectionFrame(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_detection_frame)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)


    def getAllDetectionFrameOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return DetectionFrame.objects.filter(
            detection_session__drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListDetectionFramesToJsonFormat(listOfDetectionFrames):
        listOfDetectionFramesJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfDetectionFrames))
        for recordOfDetectionFramesJson in listOfDetectionFramesJson:
            fields = recordOfDetectionFramesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "droneDetectionVideoFrame"
            fields["drone"] = DetectionSession.getDroneFromDetectionSessionId(
                fields["detection_session"]).drone_name
        return [recordOfDetectionFramesJson["fields"] for recordOfDetectionFramesJson in listOfDetectionFramesJson]


class DetectedObject(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)
    lat = models.FloatField()
    lon = models.FloatField()

    #optional
    # alt = models.FloatField(null=True)
    bounding_boxes = models.JSONField(null=True)
    confidence = models.FloatField(null=True)
    #object_id = models.IntegerField(null=True)

    label = models.CharField(max_length=100)
    track_id = models.IntegerField(null=True)
    distance_from_drone = models.FloatField(null=True)
    frame = models.ForeignKey(DetectionFrame, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)

    def getAllDetectionObjectsOfDroneBetweenTwoTimes(_droneName, _startTime, _endTime):
        return DetectedObject.objects.filter(
            detection_session__drone__drone_name=_droneName,
            time__gte=_startTime,
            time__lte=_endTime,
        )

    def convertListDetectionObjectsToJsonFormat(listOfDetectionObjects):
        listOfDetectionObjectsJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfDetectionObjects))
        for recordOfDetectionObjectsJson in listOfDetectionObjectsJson:
            fields = recordOfDetectionObjectsJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "droneDetectionVideoFrame"
            fields["drone"] = DetectionSession.getDroneFromDetectionSessionId(
                fields["detection_session"]).drone_name
        return [recordOfDetectionObjectsJson["fields"] for recordOfDetectionObjectsJson in listOfDetectionObjectsJson]

    def getAllActiveDetectionObjectsByOperationId(_operationId):
        # Subquery to get the latest time for each track_id
        latest_times = DetectedObject.objects.filter(
            detection_session__operation__id=_operationId,
            detection_session__is_active=True
        ).values('track_id').annotate(latest_time=Max('time'))

        # Filter the DetectedObject instances with the latest time for each track_id
        latest_records = DetectedObject.objects.filter(
            detection_session__operation__id=_operationId,
            detection_session__is_active=True,
            time__in=[entry['latest_time'] for entry in latest_times]
        )

        # Return the list of values
        return list(
            latest_records.values(
                "id",
                "time",
                "lat",
                "lon",
                "label",
                "track_id",
                "distance_from_drone",
                "detection_session",
                "frame",
                "operation",
                "drone",
                "bounding_boxes",
                "confidence"
            )
        )


# DetectionDescription!

class DetectedObjectDescription(models.Model):
    track_id = models.IntegerField()
    description = models.CharField(max_length=300, null=True, blank=True)
    detection_session = models.ForeignKey(DetectionSession, on_delete=models.CASCADE)
    is_suspicious = models.BooleanField(default=False)
    should_follow = models.BooleanField(default=False)
    updated_by = models.ForeignKey(get_user_model(), on_delete=models.DO_NOTHING)
    updated_at = models.DateTimeField(auto_now_add=True)

    def getAllActiveDetectionDescriptionsByOperationId(_operationId):
        # Subquery to get the latest updated_at for each track_id
        latest_updates = DetectedObjectDescription.objects.filter(
            detection_session__operation__id=_operationId,
            detection_session__is_active=True
        ).values('track_id').annotate(latest_update=Max('updated_at'))

        # Filter the DetectedObjectDescription instances with the latest update for each track_id
        latest_records = DetectedObjectDescription.objects.filter(
            detection_session__operation__id=_operationId,
            detection_session__is_active=True,
            updated_at__in=[entry['latest_update'] for entry in latest_updates]
        )

        # Return the list of values
        return list(
            latest_records.values(
                "id",
                "track_id",
                "description",
                "is_suspicious",
                "should_follow",
                "updated_by",
                "updated_at",
                "detection_session",
            )
        )

    def updateDescriptionByTrackIdAndSessionId(
        _detectionSession, _trackId, _description, _isSuspicious, _shouldFollow, _user
    ):
        DetectedObjectDescription.objects.create(
            track_id=_trackId,
            description=_description,
            detection_session_id=_detectionSession,
            is_suspicious=_isSuspicious,
            should_follow=_shouldFollow,
            updated_by=_user,
        )

# Disaster Classification detections
class DetectedDisaster(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    lat = models.FloatField()
    lon = models.FloatField()
    earthquake_probability = models.FloatField()
    fire_probability = models.FloatField()
    flood_probability = models.FloatField()
    frame = models.ForeignKey(DetectionFrame, on_delete=models.CASCADE)


# Crowd Localization detections
class DetectionCrowdLocalizationResults(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(
        DetectionSession, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    frame = models.ForeignKey(DetectionFrame, on_delete=models.CASCADE)
    coordinates = models.TextField()
    count = models.IntegerField(default=0)

    # lat = models.FloatField()
    # lon = models.FloatField()
    # label = models.CharField(max_length=100)
    # track_id = models.IntegerField()
    # distance_from_drone = models.FloatField()