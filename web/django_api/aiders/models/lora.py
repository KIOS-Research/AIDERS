
import datetime
import json

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone

from .operation import OnlineSession


class BaloraMaster(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    name = models.CharField(max_length=100, unique=True)
    operation = models.ForeignKey("Operation", on_delete=models.SET_NULL, blank=True, null=True)
    is_connected_with_platform = models.BooleanField()

    def save(self, *args, **kwargs):
        super(BaloraMaster, self).save()
        OnlineSession.update_balora_master_session(self)
        if self.operation != None and BaloraSession.objects.filter(baloraMaster=self, is_active=True).last() is None:
            BaloraSession.objects.create(
                user=self.operation.operator,
                operation=self.operation,
                baloraMaster=self,
                is_active=True,
            )
        if self.is_connected_with_platform == False:
            if balora_session := BaloraSession.objects.filter(baloraMaster=self, is_active=True).last():
                balora_session.is_active = False
                balora_session.end_time = datetime.datetime.now()
                balora_session.save()

    def __str__(self) -> str:
        return self.name


class Balora(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    baloraMaster = models.ForeignKey(BaloraMaster, on_delete=models.CASCADE)
    name = models.CharField(max_length=100)

    def save(self, *args, **kwargs):
        super(Balora, self).save()

    def getBaloraFromIdToJsonFormat(baloraIds):
        deserialized_data = json.loads(coreDjangoSerializers.serialize(
            "json", [Balora.objects.get(pk=baloraIds)]))
        # Convert time, start_time, and end_time to timestamps
        for record in deserialized_data:
            fields = record["fields"]
            if "ip" in fields:
                del fields["ip"]
            if "time" in fields:
                del fields["time"]
            fields["pk"] = record["pk"]
        return [record["fields"] for record in deserialized_data]

    def getBaloraNameById(pk):
        try:
            return Balora.objects.get(pk=pk).name
        except Balora.DoesNotExist:
            return None


class BaloraSession(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    baloraMaster = models.ForeignKey(BaloraMaster, on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)


class BaloraTelemetry(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    baloraMaster = models.ForeignKey(BaloraMaster, on_delete=models.CASCADE)
    balora = models.ForeignKey(Balora, on_delete=models.CASCADE)
    latitude = models.FloatField(blank=True, null=True)
    longitude = models.FloatField(blank=True, null=True)
    pm1 = models.FloatField(blank=True, null=True)
    pm25 = models.FloatField(blank=True, null=True)
    acc_x = models.FloatField(blank=True, null=True)
    acc_y = models.FloatField(blank=True, null=True)
    acc_z = models.FloatField(blank=True, null=True)
    received_signal_strength_indication = models.FloatField()
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    secondsOn = models.FloatField()

    def save(self, *args, **kwargs):
        self.operation = self.baloraMaster.operation
        super(BaloraTelemetry, self).save()

    def getAllBalorasOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return (
            BaloraTelemetry.objects.filter(
                operation__operation_name=operationName,
                time__gte=startTime,
                time__lte=endTime,
                balora__isnull=False,
            )
            .values_list("balora", flat=True)
            .distinct()
        )

    def getAllTelemetriesOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return BaloraTelemetry.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListBaloraTelemetryToJsonFormat(listOfBaloraTelemetries):
        listOfBaloraTelemetryJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfBaloraTelemetries))
        for recordOfBaloraTelemetriesJson in listOfBaloraTelemetryJson:
            fields = recordOfBaloraTelemetriesJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "baloraTelemetry"
            fields["balora"] = Balora.getBaloraNameById(fields["balora"])
        return [recordOfBaloraTelemetriesJson["fields"] for recordOfBaloraTelemetriesJson in listOfBaloraTelemetryJson]


class BaloraMonitor(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    baloraMaster = models.ForeignKey(BaloraMaster, on_delete=models.CASCADE)
    balora = models.ForeignKey(Balora, on_delete=models.CASCADE)
    cpu_usage = models.FloatField()
    heap_memory = models.FloatField()
    battery_percentage = models.FloatField()
    received_signal_strength_indication = models.FloatField()
    operation = models.ForeignKey(
        "Operation", on_delete=models.CASCADE, blank=True, null=True)
    secondsOn = models.FloatField()

    def save(self, *args, **kwargs):
        self.operation = self.baloraMaster.operation
        super(BaloraTelemetry, self).save()
