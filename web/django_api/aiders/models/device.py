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

class Device(models.Model):
    name = models.CharField(max_length=100, unique=True)
    operator = models.CharField(max_length=100)
    ip = models.CharField(max_length=100, blank=True, null=True)
    model = models.CharField(max_length=200)
    time = models.DateTimeField(null=True, blank=True)
    operation = models.ForeignKey("Operation", on_delete=models.SET_NULL, blank=True, null=True)
    is_connected_with_platform = models.BooleanField()

    def save(self, *args, **kwargs):
        super(Device, self).save()
        OnlineSession.update_device_session(self)
        if self.operation != None and DeviceSession.objects.filter(device=self, is_active=True).last() is None:
            DeviceSession.objects.create(
                user=self.operation.operator,
                operation=self.operation,
                device=self,
                is_active=True,
                folder_path=Constants.DEVICE_IMAGE_DIR_PREFIX + self.name + "_",
            )
        if self.is_connected_with_platform == False and DeviceSession.objects.filter(device=self, is_active=True).last() is not None:
            if device_session := DeviceSession.objects.filter(device=self, is_active=True).last():
                device_session.is_active = False
                device_session.end_time = datetime.datetime.now()
                device_session.save()

    def __str__(self) -> str:
        return self.name

    def getDeviceNameById(pk):
        try:
            return Device.objects.get(pk=pk).name
        except Device.DoesNotExist:
            return None

    def getDeviceFromIdToJsonFormat(deviceId):
        deserialized_data = json.loads(coreDjangoSerializers.serialize(
            "json", [Device.objects.get(pk=deviceId)]))
        # Convert time, start_time, and end_time to timestamps
        for record in deserialized_data:
            fields = record["fields"]
            if "ip" in fields:
                del fields["ip"]
            if "time" in fields:
                del fields["time"]
            fields["pk"] = record["pk"]
        return [record["fields"] for record in deserialized_data]


class DeviceTelemetry(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    device = models.ForeignKey(Device, on_delete=models.CASCADE)
    latitude = models.FloatField()
    longitude = models.FloatField()
    altitude = models.FloatField()
    heading = models.FloatField()
    orientation_x = models.FloatField()
    orientation_y = models.FloatField()
    orientation_z = models.FloatField()
    accelerometer_x = models.FloatField()
    accelerometer_y = models.FloatField()
    accelerometer_z = models.FloatField()
    gyroscope_x = models.FloatField()
    gyroscope_y = models.FloatField()
    gyroscope_z = models.FloatField()
    geomagnetic_x = models.FloatField()
    geomagnetic_y = models.FloatField()
    geomagnetic_z = models.FloatField()
    light = models.FloatField()
    step = models.FloatField()
    pressure = models.FloatField()
    proximity = models.FloatField()
    battery_percentage = models.FloatField()
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    secondsOn = models.FloatField()

    def save(self, *args, **kwargs):
        self.operation = self.device.operation
        super(DeviceTelemetry, self).save()

    def getAllDevicesOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return (
            DeviceTelemetry.objects.filter(
                operation__operation_name=operationName,
                time__gte=startTime,
                time__lte=endTime,
                device__isnull=False,
            )
            .values_list("device", flat=True)
            .distinct()
        )

    def getAllTelemetriesOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return DeviceTelemetry.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListDeviceTelemetryToJsonFormat(listOfDeviceTelemetries):
        listOfDeviceTelemetryJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfDeviceTelemetries))
        for recordOfDeviceTelemetriesJson in listOfDeviceTelemetryJson:
            fields = recordOfDeviceTelemetriesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "deviceTelemetry"
            fields["device"] = Device.getDeviceNameById(fields["device"])
        return [recordOfDeviceTelemetriesJson["fields"] for recordOfDeviceTelemetriesJson in listOfDeviceTelemetryJson]


class DeviceSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    device = models.ForeignKey(Device, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    folder_path = models.CharField(max_length=255, null=False)

    def save(self, *args, **kwargs):
        if self.folder_path:
            # Set a default value for folder_path if it is not provided
            self.folder_path = self.folder_path + \
                datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
            os.mkdir(default_storage.path(self.folder_path))
            super(DeviceSession, self).save()


class DeviceImage(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    path = models.CharField(max_length=255, null=False)
    device = models.ForeignKey(Device, on_delete=models.CASCADE)
    latitude = models.FloatField()
    longitude = models.FloatField()
    session = models.ForeignKey(DeviceSession, on_delete=models.CASCADE)

    def getAllDeviceImagesOfDeviceBetweenTwoTimes(deviceName, startTime, endTime):
        return DeviceImage.objects.filter(
            device__name=deviceName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListDeviceImagesToJsonFormat(listOfDeviceImages):
        listOfDeviceImageJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfDeviceImages))
        for recordOfDeviceimagesJson in listOfDeviceImageJson:
            fields = recordOfDeviceimagesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "deviceImage"
            fields["device"] = Device.getDeviceNameById(fields["device"])
            fields["id"] = recordOfDeviceimagesJson["pk"]
        return [recordOfDeviceimagesJson["fields"] for recordOfDeviceimagesJson in listOfDeviceImageJson]
