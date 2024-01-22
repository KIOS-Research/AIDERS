from django.contrib.auth import get_user_model
from drf_extra_fields.geo_fields import PointField
from rest_framework import serializers

from .models import *

User = get_user_model()


class OperationSerializer(serializers.ModelSerializer):
    # created = serializers.DateTimeField()
    # test_column = serializers.CharField(max_length=100)
    operator = serializers.ReadOnlyField(source="operator.username")

    class Meta:
        model = Operation
        fields = "__all__"


class DroneSerializer(serializers.ModelSerializer):
    # flight_id = serializers.PrimaryKeyRelatedField(allow_null=True, queryset=Flight.objects.all(), required=False, source='flight')
    class Meta:
        model = Drone
        fields = "__all__"

    def create(self, validated_data):
        return Drone.objects.create(**validated_data)

    def update(self, instance, validated_data):
        """
        Update and return an existing `Drone` instance, given the validated data.
        :param instance:
        :param validated_data:
        """
        instance.mission = validated_data.get(
            "mission"
        )  # This is in case user starts a mission. We have to update this field for the involved drones
        instance.mission = validated_data.get("detection")
        instance.save()
        return instance


class DeviceSerializer(serializers.ModelSerializer):
    class Meta:
        model = Device
        fields = "__all__"


class DeviceTelemetrySerializer(serializers.ModelSerializer):
    class Meta:
        model = DeviceTelemetry
        fields = "__all__"


class LoraSerializer(serializers.ModelSerializer):
    class Meta:
        model = BaloraMaster
        fields = "__all__"


class LoraTelemetrySerializer(serializers.ModelSerializer):
    class Meta:
        model = BaloraTelemetry
        fields = "__all__"


class DetectionDroneSerializer(serializers.ModelSerializer):
    """
    A serializer that serializes and displayes the detection status of a drone
    """

    drone_name = serializers.CharField()
    detection_status = serializers.CharField(source="detection.detection_status")
    detection_type = serializers.CharField(source="detection.detection_type")
    id = serializers.IntegerField(source="detection.id")

    class Meta:
        model = Detection
        fields = ("drone_name", "detection_status", "detection_type", "id")


class DetectionSerializer(serializers.ModelSerializer):
    class Meta:
        model = Detection
        fields = "__all__"  # , 'detection_status', 'detection_type')

    # def create(self, validated_data):
    #     detection = Detection.objects.create(**validated_data)


class DetectedObjectSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectedObject
        fields = "__all__"


class DetectionSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectionSession
        fields = "__all__"


class LiveStreamSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = LiveStreamSession
        fields = "__all__"


class ControlDeviceSerializer(serializers.ModelSerializer):
    class Meta:
        model = ControlDevice
        fields = "__all__"


class TelemetrySerializer(serializers.ModelSerializer):
    class Meta:
        model = Telemetry
        fields = "__all__"


class ErrorMessageSerializer(serializers.ModelSerializer):
    class Meta:
        model = ErrorMessage
        fields = "__all__"


class AlgorithmSerializer(serializers.ModelSerializer):
    class Meta:
        model = Algorithm
        fields = "__all__"


class MissionPointSerializer(serializers.ModelSerializer):
    class Meta:
        model = MissionPoint
        fields = ("point",)

    point = PointField(required=False)


class MissionSerializer(serializers.ModelSerializer):

    # drone = serializers.PrimaryKeyRelatedField(queryset=Drone.objects.filter(name="kios_drone0"))
    # drone = serializers.StringRelatedField()

    """
    Creating a nested relationship that will enable us to read (read-only)
    which drones correspond to this mission
    """
    # drones = DroneSerializer(many=True, read_only=True)
    """
    Creating a nested relationship that will enable us to WRITE
    the mission path (the location points) for this mission
    """
    mission_points = MissionPointSerializer(many=True)
    # time = serializers.DateTimeField(format='%Y-%m-%d %H:%M:%S')
    class Meta:
        model = Mission
        fields = [
            "mission_type",
            "time",
            "mission_completed",
            "grid",
            "captureAndStoreImages",
            "mission_points",
            "operation",
            "user",
            "mission_speeds",
            "mission_gimbal",
            "repeat",
        ]

        # fields = ('__all__')

    def create(self, validated_data):
        """
        Have to override the create method of this serializer, due to the following special cases:
        1) We have a  ManytoMany MissionPoints field on this table that relates to MissionPoint.
           This is to be able to have multiple location points and create the mission path for this mission
        2) A mission can have multiple drones. For this reason, for each mentioned drone that participates in this
           mission, we update its 'mission' field
        """

        mission_points_data = validated_data.pop(
            "mission_points"
        )  # When you receive the data via POST request, subtract from this data the mission path, which is an array of locations
        # drone_pks =  validated_data.pop('drones')
        mission = Mission.objects.create(**validated_data)  # Create the mission without the mission_path
        # Add each way point the mission_points field to construct the mission path
        for mission_point_data in mission_points_data:
            mission_point = MissionPoint.objects.create(**mission_point_data)
            mission.mission_points.add(mission_point)

        # Update the "mission" field for each drone that is going to participate on this mission
        # for drone_pk in drone_pks:
        #     obj = get_object_or_404(Drone.objects.filter(pk=drone_pk))
        #     serializer = DroneSerializer(obj, data={'mission':mission.id}, partial=True)
        #     if (serializer.is_valid()):
        #         print("SERIALIZER FOR DRONE PK IS VALID********")
        #         serializer.save()
        return mission


class MissionLoggerSerializer(serializers.ModelSerializer):
    class Meta:
        model = MissionLog
        fields = "__all__"


class UserSerializer(serializers.ModelSerializer):
    operations = serializers.PrimaryKeyRelatedField(many=True, queryset=Operation.objects.all())

    class Meta:
        model = User
        fields = ["id", "username", "operations"]
        # fields = ('__all__')


class DetectionFrameSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectionFrame
        fields = "__all__"


class WeatherStationSerializer(serializers.ModelSerializer):
    class Meta:
        model = WeatherStation
        fields = "__all__"


class LidarPointSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = LidarPointSession
        fields = "__all__"


class LidarPointSerializer(serializers.ModelSerializer):
    class Meta:
        model = LidarPoint
        fields = "__all__"


class UserPreferencesSerializer(serializers.ModelSerializer):
    class Meta:
        model = UserPreferences
        fields = "__all__"


## Manually detected object 

class DetectedObjectDescriptionSerializer(serializers.ModelSerializer):

    updated_by_username = serializers.CharField(source='received_by.username', read_only=True)

    class Meta:
        model = DetectedObjectDescription
        fields = [ 
                            "id",
                            "track_id",
                            "description",
                            "updated_by" ,
                            "updated_at",
                            "updated_by_username",
                        ]
        read_only_fields = [ 
                            "id",
                            "updated_by" ,
                            "updated_at",
                            "updated_by_username",
                        ]

    def create(self, validated_data ):
        updated_by = validated_data.pop("updated_by")
        Detected_Object_Description = DetectedObjectDescription.objects.create(    **validated_data, updated_by = updated_by) 
        return Detected_Object_Description

    def update(self, instance, validated_data):
        updated_by = validated_data.pop("updated_by")
        instance.description = validated_data.get('description', instance.description)
        instance.updated_by = updated_by
        instance.save()
        return instance

class ManuallySetObjectSerializer(serializers.ModelSerializer):

    created_by_username = serializers.CharField(source='created_by.username', read_only=True)

    class Meta:
        model = ManuallySetObject
        fields = [ 
                            "id",
                            "operation",
                            "created_at",
                            "description",
                            "label",
                            "created_by" ,
                            "created_by_username",
                        ]
        read_only_fields = [ 
                            "id",
                            "created_by" ,
                            "created_by_username"
                        ]
                        
    def create(self, validated_data ):
        created_by = validated_data.pop("created_by")
        Manually_Set_Object = ManuallySetObject.objects.create( **validated_data, created_by = created_by) 
        return Manually_Set_Object

    def update(self, instance, validated_data):
        instance.description = validated_data.get('description', instance.description)
        instance.save()
        return instance


class ManuallySetObjectLocationSerializer(serializers.ModelSerializer):

    coords_set_by_username = serializers.CharField(source='received_by.username', read_only=True)
    coords_set_by= serializers.PrimaryKeyRelatedField(source='received_by', read_only=True)

    class Meta:
        model = ManuallySetObjectLocation
        fields = [ 
                    "id",
                    "received_by" ,
                    "received_at",
                    "lon",
                    "lat",
                    "manually_set_object",
                    "coords_set_by",
                    "coords_set_by_username",
                    ]
        read_only_fields = [ 
                            "id",
                            "received_by" ,
                            "coords_set_by",
                            "coords_set_by_username",
                        ]

    def create(self, validated_data ):
        received_by = validated_data.pop("received_by")
        Manually_Set_Object_Location = ManuallySetObjectLocation.objects.create(    **validated_data, 
                                                                                    received_by = received_by) 
        return Manually_Set_Object_Location

class ManuallySetObjectDescriptionSerializer(serializers.ModelSerializer):

    updated_by_username = serializers.CharField(source='updated_by.username', read_only=True)

    class Meta:
        model = ManuallySetObjectDescription
        fields = [ 
                            "id",
                            "description",
                            "updated_by" ,
                            "updated_at",
                            "manually_set_object",
                            "updated_by_username",
                        ]
        read_only_fields = [ 
                            "id",
                            "updated_by" ,
                            "updated_by_username",
                        ]

    def create(self, validated_data ):
        updated_by = validated_data.pop("updated_by")
        Manually_Set_Object_Description = ManuallySetObjectDescription.objects.create(  **validated_data, 
                                                                                        updated_by = updated_by) 

        return Manually_Set_Object_Description
