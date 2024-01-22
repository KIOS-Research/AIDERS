import datetime
import json
import os

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.db import transaction
from django.utils import timezone
from logic.Constants import Constants


def get_upload_path_for_detection_frame(instance, filename):
    return os.path.join(
        Constants.DETECTION_FRAMES_DIR_NAME_PREFIX
        + instance.detection_session.drone.drone_name
        + "_"
        + str(instance.detection_session.start_time.strftime("%Y-%m-%d_%H.%M.%S")),
        filename,
    )


class SafemlDetectionSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    latest_frame_url = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        # At any given time, there should be only one active Detection Session per
        # drone, because one drone cannot initiate two detection sessions at once
        if not self.is_active:
            return super(SafemlDetectionSession, self).save(*args, **kwargs)

        # If for some reason it happens that other Detection Sessions for this drone
        # are still true (there shouldn't be because starting a new detection session
        # means that the previous one was closed) make them all false
        with transaction.atomic():
            SafemlDetectionSession.objects.filter(is_active=True, drone=self.drone).update(is_active=False)
            return super(SafemlDetectionSession, self).save(*args, **kwargs)

    def getDroneFromDetectionSessionId(detectionSessionId):
        return SafemlDetectionSession.objects.get(id=detectionSessionId).drone


class SafemlDetectionFrame(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_detection_frame)
    detection_session = models.ForeignKey(SafemlDetectionSession, on_delete=models.CASCADE)

    def getAllDetectionFrameOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return SafemlDetectionFrame.objects.filter(
            detection_session__drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListDetectionFramesToJsonFormat(listOfDetectionFrames):
        listOfDetectionFramesJson = json.loads(coreDjangoSerializers.serialize("json", listOfDetectionFrames))
        for recordOfDetectionFramesJson in listOfDetectionFramesJson:
            fields = recordOfDetectionFramesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(
                    datetime.datetime.fromisoformat(fields["time"][:-1]), timezone.utc
                ).timestamp()
            fields["type"] = "droneDetectionVideoFrame"
            fields["drone"] = SafemlDetectionSession.getDroneFromDetectionSessionId(
                fields["detection_session"]
            ).drone_name
        return [recordOfDetectionFramesJson["fields"] for recordOfDetectionFramesJson in listOfDetectionFramesJson]


class SafemlOutput(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(SafemlDetectionSession, on_delete=models.CASCADE)
    scue = models.FloatField(null=True, blank=True)
    frame = models.ForeignKey(SafemlDetectionFrame, on_delete=models.CASCADE)

class DeepKnowledgeDetectionSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    latest_frame_url = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        # At any given time, there should be only one active Detection Session per
        # drone, because one drone cannot initiate two detection sessions at once
        if not self.is_active:
            return super(DeepKnowledgeDetectionSession, self).save(*args, **kwargs)

        # If for some reason it happens that other Detection Sessions for this drone
        # are still true (there shouldn't be because starting a new detection session
        # means that the previous one was closed) make them all false
        with transaction.atomic():
            DeepKnowledgeDetectionSession.objects.filter(is_active=True, drone=self.drone).update(is_active=False)
            return super(DeepKnowledgeDetectionSession, self).save(*args, **kwargs)

    def getDroneFromDetectionSessionId(detectionSessionId):
        return DeepKnowledgeDetectionSession.objects.get(id=detectionSessionId).drone


class DeepKnowledgeDetectionFrame(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_detection_frame)
    detection_session = models.ForeignKey(DeepKnowledgeDetectionSession, on_delete=models.CASCADE)

    def getAllDetectionFrameOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return DeepKnowledgeDetectionFrame.objects.filter(
            detection_session__drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListDetectionFramesToJsonFormat(listOfDetectionFrames):
        listOfDetectionFramesJson = json.loads(coreDjangoSerializers.serialize("json", listOfDetectionFrames))
        for recordOfDetectionFramesJson in listOfDetectionFramesJson:
            fields = recordOfDetectionFramesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(
                    datetime.datetime.fromisoformat(fields["time"][:-1]), timezone.utc
                ).timestamp()
            fields["type"] = "droneDetectionVideoFrame"
            fields["drone"] = DeepKnowledgeDetectionSession.getDroneFromDetectionSessionId(
                fields["detection_session"]
            ).drone_name
        return [recordOfDetectionFramesJson["fields"] for recordOfDetectionFramesJson in listOfDetectionFramesJson]


class DeepKnowledgeOutput(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    detection_session = models.ForeignKey(DeepKnowledgeDetectionSession, on_delete=models.CASCADE)
    uncertainty = models.FloatField(null=True, blank=True)
    frame = models.ForeignKey(DeepKnowledgeDetectionFrame, on_delete=models.CASCADE)