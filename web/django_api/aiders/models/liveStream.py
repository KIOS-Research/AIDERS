import datetime
import json
import os

from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.db import transaction
from django.utils import timezone

from logic.Constants import Constants
from .drone import Drone


class LiveStreamSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
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


def get_upload_path_for_raw_frame(instance, filename):
    return os.path.join(
        Constants.RAW_FRAMES_DIR_NAME_PREFIX
        + instance.live_stream_session.drone.drone_name
        + "_"
        + str(instance.live_stream_session.start_time.strftime("%Y-%m-%d_%H.%M.%S")),
        filename,
    )


class RawFrame(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    frame = models.ImageField(upload_to=get_upload_path_for_raw_frame)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    live_stream_session = models.ForeignKey(
        LiveStreamSession, on_delete=models.CASCADE)

    def getAllFramesOfDroneBetweenTwoTimes(droneName, startTime, endTime):
        return RawFrame.objects.filter(
            drone__drone_name=droneName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListFramesToJsonFormat(listOfFrames):
        listOfFramesJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfFrames))
        for recordOfFramesJson in listOfFramesJson:
            fields = recordOfFramesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "droneVideoFrame"
            fields["drone"] = Drone.getDroneNameById(fields["drone"])
        return [recordOfFramesJson["fields"] for recordOfFramesJson in listOfFramesJson]
