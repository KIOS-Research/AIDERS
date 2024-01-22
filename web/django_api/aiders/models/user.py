import datetime
import json

from django.contrib.auth import get_user_model
from django.contrib.auth.models import AbstractUser, Group
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone


class User(AbstractUser):
    # On which operation the current user is currently joined. It can be NULL
    joined_operation = models.ForeignKey(
        "Operation", on_delete=models.SET_NULL, blank=True, null=True)

    def update_permissions(self, edit_data, value):
        user = User.objects.get(id=self)
        if edit_data == "permission_edit_permissions":
            my_group = Group.objects.get(name="edit_permissions")
            if value:
                my_group.user_set.add(user)
            else:
                my_group.user_set.remove(user)
        elif edit_data == "permission_create_operations":
            my_group = Group.objects.get(name="create_operations")
            if value:
                my_group.user_set.add(user)
            else:
                my_group.user_set.remove(user)
        else:
            print(edit_data)

    def getUserNameById(pk):
        try:
            return User.objects.get(pk=pk).username
        except User.DoesNotExist:
            return None


class UserPreferences(models.Model):
    use_online_map = models.BooleanField(default=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)


class Terminal(models.Model):
    ip_address = models.GenericIPAddressField()
    logged_in = models.BooleanField(default=False)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    os = models.CharField(max_length=100)
    device = models.CharField(max_length=50)
    browser = models.CharField(max_length=100)


class FrontEndUserInput(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    element_name = models.CharField(max_length=255)
    value = models.CharField(max_length=255, blank=True, null=True)

    def getUserInputDataOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return FrontEndUserInput.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListUserInputDataToJsonFormat(listOfUserInputs):
        listOfUserInputsJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfUserInputs))
        for recordOfUserInputsJson in listOfUserInputsJson:
            fields = recordOfUserInputsJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(
                    fields["time"][:-1]), timezone.utc).timestamp()
            for field in fields:
                if fields[field] == None:
                    fields[field] = "None"
            fields["type"] = "userInput"
            fields["user"] = User.getUserNameById(fields["user"])
        return [recordOfUserInputsJson["fields"] for recordOfUserInputsJson in listOfUserInputsJson]
