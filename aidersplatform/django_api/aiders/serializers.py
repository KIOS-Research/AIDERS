from django.contrib.auth import get_user_model
from drf_extra_fields.geo_fields import PointField
from rest_framework import serializers

from .models import *

User = get_user_model()


class OperationSerializer(serializers.ModelSerializer):
    # created = serializers.DateTimeField()
    # test_column = serializers.CharField(max_length=100)
    operator = serializers.ReadOnlyField(source='operator.username')

    class Meta:
        model = Operation
        fields = ('__all__')



class DroneSerializer(serializers.ModelSerializer):
    # flight_id = serializers.PrimaryKeyRelatedField(allow_null=True, queryset=Flight.objects.all(), required=False, source='flight')
    class Meta:
        model = Drone
        fields = ('__all__')

    def create(self, validated_data):
        return Drone.objects.create(**validated_data)

    def update(self, instance, validated_data):
        '''
        Update and return an existing `Drone` instance, given the validated data.
        :param instance:
        :param validated_data:
        '''
        instance.mission = validated_data.get('mission') #This is in case user starts a mission. We have to update this field for the involved drones
        instance.mission = validated_data.get('detection')
        instance.save()
        return instance

class DetectionDroneSerializer(serializers.ModelSerializer):
    '''
    A serializer that serializes and displayes the detection status of a drone
    '''
    drone_name = serializers.CharField()
    detection_status = serializers.CharField(source='detection.detection_status')
    detection_type = serializers.CharField(source='detection.detection_type')
    id = serializers.IntegerField(source='detection.id')
    class Meta:
        model = Detection
        fields = ('drone_name', 'detection_status', 'detection_type','id')

class DetectionSerializer(serializers.ModelSerializer):
    class Meta:
        model = Detection
        fields = ('__all__')#, 'detection_status', 'detection_type')

    # def create(self, validated_data):
    #     detection = Detection.objects.create(**validated_data)

class DetectedObjectSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectedObject
        fields = ('__all__')

class DetectionSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectionSession
        fields = ('__all__')
class LiveStreamSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = LiveStreamSession
        fields = ('__all__')
class ControlDeviceSerializer(serializers.ModelSerializer):
    class Meta:
        model = ControlDevice
        fields = ('__all__')
class TelemetrySerializer(serializers.ModelSerializer):
    class Meta:
        model = Telemetry
        fields = ('__all__')

class ErrorMessageSerializer(serializers.ModelSerializer):
    class Meta:
        model = ErrorMessage
        fields = ('__all__')

class AlgorithmSerializer(serializers.ModelSerializer):
    class Meta:
        model = Algorithm
        fields = ('__all__')



class MissionPointSerializer(serializers.ModelSerializer):
    class Meta:
        model = MissionPoint
        fields = ('point',)

    point = PointField(required=False)
    # point = models.PointField(geography=True, srid=4326)
    
    #
    # def validate(self, attrs):
    #     '''
    #     Before giving the user the Mission Points for this mission,
    #     we first have to make sure that mission exists and the
    #     drone is participating on this mission
    #     Args:
    #         attrs:
    #
    #     Returns:
    #
    #     '''
    #     print("ATTRIBUTES: ", attrs)
    #     return attrs

class MissionSerializer(serializers.ModelSerializer):

    # drone = serializers.PrimaryKeyRelatedField(queryset=Drone.objects.filter(name="kios_drone0"))
    # drone = serializers.StringRelatedField()

    '''
    Creating a nested relationship that will enable us to read (read-only)
    which drones correspond to this mission
    '''
    # drones = DroneSerializer(many=True, read_only=True)
    '''
    Creating a nested relationship that will enable us to WRITE
    the mission path (the location points) for this mission
    '''
    mission_points = MissionPointSerializer(many=True)
    # executed_at = serializers.DateTimeField(format='%Y-%m-%d %H:%M:%S')
    class Meta:
        model = Mission
        fields =  ['mission_type',
                   'executed_at', 'mission_completed',
                   'grid', 'captureAndStoreImages',
                   'mission_points', 'operation','user']

        # fields = ('__all__')
    def create(self,validated_data):
        '''
         Have to override the create method of this serializer, due to the following special cases:
         1) We have a  ManytoMany MissionPoints field on this table that relates to MissionPoint.
            This is to be able to have multiple location points and create the mission path for this mission
         2) A mission can have multiple drones. For this reason, for each mentioned drone that participates in this
            mission, we update its 'mission' field
         '''

        mission_points_data = validated_data.pop('mission_points') #When you receive the data via POST request, subtract from this data the mission path, which is an array of locations
        # drone_pks =  validated_data.pop('drones')
        mission = Mission.objects.create(**validated_data) #Create the mission without the mission_path
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
        fields = ('__all__')

class UserSerializer(serializers.ModelSerializer):
    operations = serializers.PrimaryKeyRelatedField(many=True, queryset=Operation.objects.all())
    class Meta:
        model = User
        fields = ['id', 'username', 'operations']
        # fields = ('__all__')

class DetectionFrameSerializer(serializers.ModelSerializer):
    class Meta:
        model = DetectionFrame
        fields = ('__all__')

class WeatherStationSerializer(serializers.ModelSerializer):
    class Meta:
        model = WeatherStation
        fields = ('__all__')

class LoraTransmitterLocationSerializer(serializers.ModelSerializer):
    class Meta:
        model = LoraTransmitterLocation
        fields = ('__all__')

class LoraTransmitterSerializer(serializers.ModelSerializer):
    class Meta:
        model = LoraTransmitter
        fields = ('__all__')


class UserPreferencesSerializer(serializers.ModelSerializer):
    class Meta:
        model = UserPreferences
        fields = ('__all__')