import datetime

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core.exceptions import ObjectDoesNotExist
from django.core.validators import RegexValidator
from django.utils import timezone
from logic.Constants import Constants

from .user import User


class Operation(models.Model):
    alphanumeric = RegexValidator(
        r"^[0-9a-zA-Z !@#$%^&*()_+=/.,?:;| ]*$",
        "Only alphanumeric characters are allowed and some symbols.",
    )

    active = models.BooleanField(default=True)
    created_at = models.DateTimeField(auto_now_add=True, auto_now=False)
    description = models.TextField(blank=True, null=True)

    drones_to_operate = models.ManyToManyField("Drone", related_name="+")
    devices_to_operate = models.ManyToManyField("Device", related_name="+")
    baloras_to_operate = models.ManyToManyField("BaloraMaster", related_name="+")
    ended_at = models.DateTimeField(null=True, blank=True)
    location = models.CharField(max_length=100)

    operation_name = models.CharField(max_length=100, validators=[alphanumeric], unique=True)
    operator = models.ForeignKey(get_user_model(), related_name="operations", on_delete=models.CASCADE)

    disaster_epicenter_latitude = models.FloatField(null=True, blank=True)
    disaster_epicenter_longitude = models.FloatField(null=True, blank=True)

    def save(self, *args, **kwargs):
        just_created = False
        if self.operation_name and not self.active:  # Someone just stopped this operation
            # We have to know the time the operation finished
            self.ended_at = datetime.datetime.now(
                tz=Constants.CYPRUS_TIMEZONE_OBJ)
            User.objects.filter(joined_operation=self).update(
                joined_operation=None
            )  # If an operation is not active (i.e just stopped), NO ONE should be a member of this operation.
            super().save(*args, **kwargs)
        else:
            if not self.pk:
                # This code only happens if the object is not in the database yet. A.k.a just created
                just_created = True

            super().save(*args, **kwargs)  # First save the current operation

            if just_created:
                """
                Someone had to create this operation. Once it is created it is also automatically started.
                For this reason the creator has to automatically join the newly created operation
                """
                User.objects.filter(pk=self.operator.id).update(
                    joined_operation=self)  # Now assign to the user the just saved operation

    def getOperationIdByName(_operationName):
        try:
            return Operation.objects.get(operation_name=_operationName).id
        except ObjectDoesNotExist:
            return None  # Return None if the operation name is not found
    def getDisasterEpicenterGPSByOperationId(_operationId):
        try:
            operationObject = Operation.objects.get(id=_operationId)
            if operationObject.disaster_epicenter_latitude != None and operationObject.disaster_epicenter_longitude != None:
                return {"latitude":operationObject.disaster_epicenter_latitude, "longitude":operationObject.disaster_epicenter_longitude}
        except ObjectDoesNotExist:
            return None  # Return None if the operation name is not found
        return None
    class Meta:
        permissions = (
            ("join_operation", "Join operation"),
            ("edit_operation", "Edit operation"),
        )

    def __str__(self) -> str:
        return self.operation_name


class OnlineSession(models.Model):
    operation = models.ForeignKey(Operation, on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", null=True, blank=True, on_delete=models.CASCADE)
    device = models.ForeignKey("Device", null=True, blank=True, on_delete=models.CASCADE)
    balora_master = models.ForeignKey("BaloraMaster", null=True, blank=True, on_delete=models.CASCADE)
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(auto_now_add=False, null=True, blank=True)

    def update_drone_session(self):
        # Checks if Drone is connected and is in an operation
        if self.operation != None and self.is_connected_with_platform:
            # Checks if there is not a Session currently running
            if OnlineSession.objects.filter(drone=self, end_time__isnull=True).last() is None:
                OnlineSession.objects.create(
                    operation=self.operation, drone=self)
            elif OnlineSession.objects.filter(drone=self, end_time__isnull=True).last().operation != self.operation:
                OnlineSession.objects.filter(
                    drone=self, end_time__isnull=True).update(end_time=timezone.now())
                OnlineSession.objects.create(
                    operation=self.operation, drone=self)
        elif OnlineSession.objects.filter(drone=self, end_time__isnull=True).last() != None:
            OnlineSession.objects.filter(
                drone=self, end_time__isnull=True).update(end_time=timezone.now())

    def update_device_session(self):
        if self.operation != None and self.is_connected_with_platform:
            if OnlineSession.objects.filter(device=self, end_time__isnull=True).last() is None:
                OnlineSession.objects.create(
                    operation=self.operation, device=self)
            elif OnlineSession.objects.filter(device=self, end_time__isnull=True).last().operation != self.operation:
                OnlineSession.objects.filter(
                    device=self, end_time__isnull=True).update(end_time=timezone.now())
                OnlineSession.objects.create(
                    operation=self.operation, device=self)
        elif OnlineSession.objects.filter(device=self, end_time__isnull=True).last() != None:
            OnlineSession.objects.filter(
                device=self, end_time__isnull=True).update(end_time=timezone.now())

    def update_balora_master_session(self):
        if self.operation != None and self.is_connected_with_platform:
            if OnlineSession.objects.filter(balora_master=self, end_time__isnull=True).last() is None:
                OnlineSession.objects.create(
                    operation=self.operation, balora_master=self)
            elif OnlineSession.objects.filter(balora_master=self, end_time__isnull=True).last().operation != self.operation:
                OnlineSession.objects.filter(
                    balora_master=self, end_time__isnull=True).update(end_time=timezone.now())
                OnlineSession.objects.create(
                    operation=self.operation, balora_master=self)
        elif OnlineSession.objects.filter(balora_master=self, end_time__isnull=True).last() != None:
            OnlineSession.objects.filter(
                balora_master=self, end_time__isnull=True).update(end_time=timezone.now())
