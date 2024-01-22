import datetime
import json

from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.core import serializers as coreDjangoSerializers
from django.utils import timezone


class Algorithm(models.Model):
    FIRE_PROPAGATION_ALGORITHM = "FIRE_PROPAGATION_ALGORITHM"
    CREATE_3D_OBJECT_ALGORITHM = "CREATE_3D_OBJECT_ALGORITHM"
    CREATE_ORTHOPHOTO_ALGORITHM = "CREATE_ORTHOPHOTO_ALGORITHM"
    CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM = "CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM"

    ALGORITHM_NAMES = [
        (FIRE_PROPAGATION_ALGORITHM, FIRE_PROPAGATION_ALGORITHM),
        (CREATE_3D_OBJECT_ALGORITHM, CREATE_3D_OBJECT_ALGORITHM),
        (CREATE_ORTHOPHOTO_ALGORITHM, CREATE_ORTHOPHOTO_ALGORITHM),
        (CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM, CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM),
    ]

    time = models.DateTimeField(auto_now_add=True)
    algorithm_name = models.CharField(max_length=80, choices=ALGORITHM_NAMES)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    input = models.JSONField(blank=True, null=True)
    output = models.JSONField(blank=True, null=True)
    canBeLoadedOnMap = models.BooleanField(default=False)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)

    def getAllAlgorithmOfOperationBetweenTwoTimes(operationName, startTime, endTime):
        return Algorithm.objects.filter(
            operation__operation_name=operationName,
            time__gte=startTime,
            time__lte=endTime,
        )

    def convertListAlgorithmToJsonFormat(listOfAlgorithm):
        listOfAlgorithmJson = json.loads(
            coreDjangoSerializers.serialize("json", listOfAlgorithm))
        for recordOfAlgorithmJson in listOfAlgorithmJson:
            fields = recordOfAlgorithmJson["fields"]
            if "time" in fields:
                fields["time"] = timezone.make_aware(datetime.datetime.fromisoformat(fields["time"][:-1]), timezone.utc).timestamp()
            fields["type"] = "algorithm"
            fields["id"] = recordOfAlgorithmJson["pk"]
        return [recordOfAlgorithmJson["fields"] for recordOfAlgorithmJson in listOfAlgorithmJson]
