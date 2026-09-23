import datetime
import json
import os

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.core.files.storage import default_storage
from django.utils import timezone
from logic.Constants import Constants

from .operation import OnlineSession

class StaticCamera(models.Model):
    name = models.CharField(max_length=100, unique=True)
    model = models.CharField(max_length=200)
    live_stream_url = models.CharField(max_length=200, blank=True, null=True)
    latitude = models.FloatField(null=True, blank=True)
    longitude = models.FloatField(null=True, blank=True)
    time = models.DateTimeField(null=True, blank=True)
    operation = models.ForeignKey("Operation", on_delete=models.SET_NULL, blank=True, null=True)
    is_connected_with_platform = models.BooleanField()

    def save(self, *args, **kwargs):
        super(StaticCamera, self).save()
        OnlineSession.update_static_camera_session(self)
        if self.operation != None and StaticCameraSession.objects.filter(static_camera=self, is_active=True).last() is None:
            StaticCameraSession.objects.create(
                operation=self.operation,
                static_camera=self,
                is_active=True,
            )
        if self.is_connected_with_platform == False and StaticCameraSession.objects.filter(static_camera=self, is_active=True).last() is not None:
            if static_camera_session := StaticCameraSession.objects.filter(static_camera=self, is_active=True).last():
                static_camera_session.is_active = False
                static_camera_session.end_time = datetime.datetime.now()
                static_camera_session.save()

    def __str__(self) -> str:
        return self.name

    def getStaticCameraNameById(pk):
        try:
            return StaticCamera.objects.get(pk=pk).name
        except StaticCamera.DoesNotExist:
            return None

    def getStaticCameraFromIdToJsonFormat(static_cameraId):
        deserialized_data = json.loads(coreDjangoSerializers.serialize(
            "json", [StaticCamera.objects.get(pk=static_cameraId)]))
        # Convert time, start_time, and end_time to timestamps
        for record in deserialized_data:
            fields = record["fields"]
            if "ip" in fields:
                del fields["ip"]
            if "time" in fields:
                del fields["time"]
            fields["pk"] = record["pk"]
        return [record["fields"] for record in deserialized_data]


class StaticCameraSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    static_camera = models.ForeignKey(StaticCamera, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)

    def save(self, *args, **kwargs):
        super(StaticCameraSession, self).save()
