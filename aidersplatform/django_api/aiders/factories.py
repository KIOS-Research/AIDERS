# -----------------------------------------------------------
# Creates the factories for the tables used for the database.
# These factories are responsible for creating dummy data for the database
# Reference: https://factoryboy.readthedocs.io/en/stable/examples.html
# -----------------------------------------------------------

import datetime
import random

import factory
import factory.fuzzy
import pytz
from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.contrib.gis.geos import Point
from django.core.files.base import ContentFile
from factory.fuzzy import BaseFuzzyAttribute

from .models import *

User = get_user_model()


class FuzzyPoint(factory.fuzzy.BaseFuzzyAttribute):
    def fuzz(self):
        '''
        Custom class using the Fuzzy super class to create a location point field.
        This is the same as the 'models.PointField()' of geo django
        '''
        return Point(random.uniform(-180.0, 180.0),
                     random.uniform(-90.0, 90.0))


class UserLogFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = UserLog
    user = factory.Iterator(User.objects.all())
    event = factory.fuzzy.FuzzyText(length=50)


class OperationFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Operation
    # created_at = factory.fuzzy.FuzzyDateTime(datetime.datetime.now().replace(tzinfo=datetime.timezone.utc))
    operation_name = factory.Sequence(lambda n: 'operation_code%s' % n)
    # pilot_ip = factory.Faker('ipv4')
    active = True
    operator = factory.Iterator(User.objects.all())
    location = factory.fuzzy.FuzzyText(length=50)
    description = factory.fuzzy.FuzzyText(length=200)


class DroneFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Drone

    drone_name = factory.Sequence(lambda n: 'kios_drone%s' % n)
    model = factory.Sequence(lambda n: 'model%s' % n)
    # joined_at = factory.fuzzy.FuzzyDateTime(datetime.datetime.now().replace(tzinfo=datetime.timezone.utc))
    operation = factory.Iterator(Operation.objects.all())
    is_connected_with_platform = factory.Faker('pybool')
    build_map_activated = factory.Faker('pybool')
    mission = None
    water_sampler_available = False


class DroneToOperationLogFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = DroneToOperationLog
    drone = factory.Iterator(Drone.objects.all())
    operation = factory.Iterator(Operation.objects.all())


class DetectionSessionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = DetectionSession
    user = factory.Iterator(User.objects.all())
    operation = factory.Iterator(Operation.objects.all())
    drone = factory.Iterator(Drone.objects.all())
    is_active = factory.Faker('pybool')
    latest_frame_url = "url for the latest frame file"


class DetectionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Detection
    # DETECTION_STATUS_CHOICES = [x[0] for x in Detection.DETECTION_STATUS_CHOICES]
    # print("CHOICE:s ", DETECTION_STATUS_CHOICES)
    # DETECTION_TYPE_CHOICES = [x[0] for x in Detection.DETECTION_TYPE_CHOICES]
    detection_status = 'DETECTION_INITIAL_STATUS'
    detection_type_str = 'VEHICLE_DETECTOR'
    detection_model = 'YOLO'
    drone = factory.Iterator(Drone.objects.all())


class DetectionFrameFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = DetectionFrame
    frame = factory.LazyAttribute(
        lambda _: ContentFile(
            factory.django.ImageField()._make_data(
                {'width': 1024, 'height': 768}
            ), 'example.jpg'
        )
    )
    detection_session = factory.Iterator(DetectionSession.objects.all())


class LiveStreamSessionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = LiveStreamSession
    drone = factory.Iterator(Drone.objects.all())
    is_active = factory.Faker('pybool')
    latest_frame_url = "url for the latest frame file"


class RawFrameFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = RawFrame
    frame = factory.LazyAttribute(
        lambda _: ContentFile(
            factory.django.ImageField()._make_data(
                {'width': 1024, 'height': 768}
            ), 'example.jpg'
        )
    )
    live_stream_session = factory.Iterator(LiveStreamSession.objects.all())


class DetectedObjectFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = DetectedObject
    detection_session = factory.Iterator(DetectionSession.objects.all())
    lat = factory.fuzzy.FuzzyFloat(low=0, high=100)
    lon = factory.fuzzy.FuzzyFloat(low=0, high=100)
    label = factory.fuzzy.FuzzyText(length=50)
    track_id = factory.fuzzy.FuzzyInteger(low=0, high=100)
    distance_from_drone = factory.fuzzy.FuzzyFloat(low=0, high=100)
    frame = factory.Iterator(DetectionFrame.objects.all())


class WeatherStationFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = WeatherStation
    # started_at = factory.fuzzy.FuzzyDateTime(datetime.datetime.now().replace(tzinfo=datetime.timezone.utc))
    wind_speed = factory.fuzzy.FuzzyFloat(low=0, high=100)
    wind_direction = factory.fuzzy.FuzzyFloat(low=0, high=360)
    temperature = factory.fuzzy.FuzzyFloat(low=0, high=100)
    pressure = factory.fuzzy.FuzzyFloat(low=0, high=100)
    humidity = factory.fuzzy.FuzzyFloat(low=0, high=100)
    heading = factory.fuzzy.FuzzyFloat(low=0, high=360)
    operation = factory.Iterator(Operation.objects.all())


class TelemetryFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Telemetry
    # received_at = factory.fuzzy.FuzzyDateTime(datetime.datetime.now().replace(tzinfo=datetime.timezone.utc))
    drone = factory.Iterator(Drone.objects.all())
    battery_percentage = factory.fuzzy.FuzzyFloat(low=0)
    gps_signal = factory.fuzzy.FuzzyFloat(low=0)
    satellites = factory.fuzzy.FuzzyInteger(low=0)
    heading = factory.fuzzy.FuzzyInteger(low=0, high=10)
    velocity = factory.fuzzy.FuzzyFloat(low=0)
    homeLat = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    homeLon = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    lat = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    lon = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    alt = factory.fuzzy.FuzzyFloat(low=0)
    drone_state = factory.fuzzy.FuzzyText()
    secondsOn = factory.fuzzy.FuzzyInteger(low=0)
    gimbal_angle = factory.fuzzy.FuzzyFloat(low=-90, high=90)


class MissionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Mission
    mission_type = factory.Faker('random_choices', elements=[
                                 'NORMAL_MISSION', 'SEARCH_AND_RESCUE_MISSION', 'GRID_MISSION'])
    mission_completed = factory.Faker('pybool')
    grid = factory.Faker('pybool')
    captureAndStoreImages = factory.Faker('pybool')
    operation = factory.Iterator(Operation.objects.all())
    user = factory.Iterator(User.objects.all())
    # telemetry = factory.Iterator(Telemetry.objects.all())
    # drone = factory.Iterator(Drone.objects.all())

    # @factory.post_generation
    # def drones(self, create, extracted, **kwargs):
    #     if not create:
    #         # Simple build, do nothing.
    #         return
    #
    #     if extracted:
    #         # A list of groups were passed in, use them
    #         for drone in extracted:
    #             self.drones.add(drone)

    @factory.post_generation
    def mission_points(self, create, extracted, **kwargs):
        if not create:
            # Simple build, do nothing.
            return

        if extracted:
            # A list of groups were passed in, use them
            for point in extracted:
                self.mission_points.add(point)


class MissionLogFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = MissionLog

    mission = factory.Iterator(Mission.objects.all())
    action = factory.Faker('random_choices', elements=[
                           'START_MISSION', 'PAUSE_MISSION', 'RESUME_MISSION', 'CANCEL_MISSION'])
    operation = factory.Iterator(Operation.objects.all())
    user = factory.Iterator(User.objects.all())
    drone = factory.Iterator(Drone.objects.all())
    algorithm = None


class MissionPointFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = MissionPoint
    # lat = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    # lon = factory.fuzzy.FuzzyFloat(low=-90, high=90)
    point = FuzzyPoint()


class UserFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = get_user_model()

    joined_operation = None
    username = factory.Sequence(lambda n: 'admin%s' % n)
    password = factory.LazyFunction(lambda: make_password('pass'))
    first_name = factory.Faker('first_name')
    last_name = factory.Faker('last_name')
    last_login = factory.fuzzy.FuzzyDateTime(
        datetime.datetime(2008, 1, 1, tzinfo=pytz.UTC))
    is_active = True
    date_joined = factory.fuzzy.FuzzyDateTime(datetime.datetime(
        2005, 1, 1,  tzinfo=pytz.UTC), datetime.datetime(2008, 1, 1, tzinfo=pytz.UTC))
    is_staff = True
    is_superuser = True


class WaterSamplerFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = WaterSampler
    drone = factory.Iterator(Drone.objects.all())
    operation = factory.Iterator(Operation.objects.all())
    user = factory.Iterator(User.objects.all())
    telemetry = factory.Iterator(Telemetry.objects.all())


class AlgorithmFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = Algorithm
    algorithm_name = factory.fuzzy.FuzzyChoice(
        ['FIRE_PROPAGATION_ALGORITHM', 'CREATE_3D_OBJECT_ALGORITHM'])
    input = {
        'type': 'object',
        'properties': {
            'result': {
                'type': 'string'
            }
        }
    }
    output = {"features": [{"geometry": {"coordinates": [[[33.34427, 35.062242], [33.344249, 35.062245], [33.343437, 35.062523], [33.343367, 35.06255], [33.34319, 35.06272], [33.343153, 35.062769], [33.343035, 35.062974], [33.343024, 35.063009], [33.343016, 35.063068], [33.343009, 35.063234], [33.343012, 35.063338], [33.343234, 35.063882], [33.34325, 35.0639], [33.343767, 35.064009], [33.343789, 35.06401], [33.343863, 35.064007], [33.343961, 35.063996], [33.344099, 35.06396], [33.344253, 35.063767], [33.344412, 35.063478], [33.344448, 35.063183], [33.344454, 35.062943], [33.344336, 35.062277], [33.344321, 35.062268], [33.34427, 35.062242]]], "type": "Polygon"}, "properties": {"Fire Fronts": 395, "Fire Speed (m/s)": 5.0, "ID": 1, "Time Step": "250 seconds", "Wind Speed (m/s)": 5.0}, "type": "Feature"}, {"geometry": {"coordinates": [[[33.345724, 35.063028], [33.344915, 35.062278], [33.344508, 35.062186], [33.344464, 35.062185], [33.344445, 35.062187], [33.343612, 35.062313], [33.343556, 35.062341], [33.34319, 35.06272], [33.343153, 35.062769], [33.343035, 35.062974], [33.343024, 35.063009], [33.343016, 35.063068], [33.343009, 35.063234], [33.343012, 35.063338], [33.343324, 35.06421], [33.343993, 35.064571], [33.344846, 35.064353], [33.344889, 35.064341], [33.34498, 35.064296], [33.345474, 35.063764], [33.345724, 35.063028]]], "type": "Polygon"}, "properties": {"Fire Fronts": 749, "Fire Speed (m/s)": 5.0, "ID": 2, "Time Step": "500 seconds", "Wind Speed (m/s)": 5.0}, "type": "Feature"}, {"geometry": {"coordinates": [
        [[33.346626, 35.063472], [33.346341, 35.062512], [33.345795, 35.061861], [33.345441, 35.061847], [33.345301, 35.061866], [33.343612, 35.062313], [33.343556, 35.062341], [33.34319, 35.06272], [33.343153, 35.062769], [33.343035, 35.062974], [33.343024, 35.063009], [33.343016, 35.063068], [33.343009, 35.063234], [33.343012, 35.063338], [33.343337, 35.064262], [33.343996, 35.064633], [33.345549, 35.065422], [33.345556, 35.065419], [33.345974, 35.064886], [33.346451, 35.064166], [33.34646, 35.064138], [33.346626, 35.063472]]], "type": "Polygon"}, "properties": {"Fire Fronts": 1123, "Fire Speed (m/s)": 5.0, "ID": 3, "Time Step": "750 seconds", "Wind Speed (m/s)": 5.0}, "type": "Feature"}, {"geometry": {"coordinates": [[[33.345301, 35.061866], [33.343612, 35.062313], [33.343556, 35.062341], [33.34319, 35.06272], [33.343153, 35.062769], [33.343035, 35.062974], [33.343024, 35.063009], [33.343016, 35.063068], [33.343009, 35.063234], [33.343012, 35.063338], [33.343337, 35.064262], [33.345067, 35.065478], [33.345996, 35.065761], [33.346102, 35.065706], [33.346806, 35.065291], [33.347004, 35.065144], [33.347216, 35.064672], [33.347395, 35.064202], [33.347366, 35.063274], [33.347349, 35.063217], [33.346754, 35.062299], [33.346359, 35.061849], [33.346286, 35.061839], [33.345828, 35.061825], [33.345441, 35.061847], [33.345301, 35.061866]]], "type": "Polygon"}, "properties": {"Fire Fronts": 1496, "Fire Speed (m/s)": 5.0, "ID": 4, "Time Step": "1000 seconds", "Wind Speed (m/s)": 5.0}, "type": "Feature"}], "type": "FeatureCollection"}

    user = factory.Iterator(User.objects.all())
    operation = factory.Iterator(Operation.objects.all())
    canBeLoadedOnMap = factory.Faker('pybool')


class BuildMapSessionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = BuildMapSession
    user = factory.Iterator(User.objects.all())
    operation = factory.Iterator(Operation.objects.all())
    drone = factory.Iterator(Drone.objects.all())
    images = factory.Iterator(BuildMapImage.objects.all())

    @factory.post_generation
    def images(self, create, extracted, **kwargs):
        if not create:
            # Simple build, do nothing.
            return

        if extracted:
            # A list of groups were passed in, use them
            for image in extracted:
                self.images.add(image)
    # images.set(factory.Iterator(BuildMapImage.objects.all()))


class BuildMapImageFactory1(factory.django.DjangoModelFactory):
    class Meta:
        model = BuildMapImage
    path = 'kios_drone0/sample_image_3.jpeg'
    top_left = Point(33.4123936719513, 35.1438787570235)
    top_right = Point(33.4121756358786, 35.1444096257444)
    bottom_left = Point(33.4119518766497, 35.144205501458)
    bottom_right = Point(33.4124605894798, 35.1440828805052)
    centre = Point(33.4124605894798, 35.144205501458)
    altitude = 40.5
    bearing = 35.1441441914389


class BuildMapImageFactory2(factory.django.DjangoModelFactory):
    class Meta:
        model = BuildMapImage
    path = 'kios_drone0/sample_image_1.jpeg'
    top_left = Point(33.4123160833058, 35.1440878264679)
    top_right = Point(33.4118495276815, 35.1436791288294)
    bottom_left = Point(33.4121742760475, 35.1436136096365)
    bottom_right = Point(33.4119913331698, 35.1441533460357)
    centre = Point(33.4120828049106, 35.1438834778766)
    altitude = 40.5
    bearing = -76.19999694822


class FuzzyPoint(BaseFuzzyAttribute):
    def fuzz(self):
        return Point(random.uniform(-180.0, 180.0),
                     random.uniform(-90.0, 90.0))


class BuildMapImageFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = BuildMapImage
    path = 'kios_drone0/sample_image_1.jpeg'
    top_left = FuzzyPoint()
    top_right = FuzzyPoint()
    bottom_left = FuzzyPoint()
    bottom_right = FuzzyPoint()
    centre = FuzzyPoint()
    altitude = factory.fuzzy.FuzzyDecimal(low=0, high=500)
    bearing = factory.fuzzy.FuzzyDecimal(low=-360, high=360)


class ControlDeviceFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = ControlDevice
    drone = factory.Iterator(Drone.objects.all())
    cpu_usage = factory.fuzzy.FuzzyFloat(low=0, high=100)
    cpu_core_usage = factory.fuzzy.FuzzyText(length=200)
    cpu_core_frequency = factory.fuzzy.FuzzyText(length=200)
    cpu_temp = factory.fuzzy.FuzzyFloat(low=0, high=100)
    cpu_fan_RPM = factory.fuzzy.FuzzyFloat(low=0, high=2000)
    gpu_usage = factory.fuzzy.FuzzyFloat(low=0, high=100)
    gpu_frequency = factory.fuzzy.FuzzyFloat(low=0, high=2000)
    gpu_temp = factory.fuzzy.FuzzyFloat(low=0, high=100)
    ram_usage = factory.fuzzy.FuzzyFloat(low=0, high=100)
    swap_usage = factory.fuzzy.FuzzyFloat(low=0, high=100)
    swap_cache = factory.fuzzy.FuzzyFloat(low=0, high=100)
    emc_usage = factory.fuzzy.FuzzyFloat(low=0, high=2000)


class ErrorMessageFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = ErrorMessage
    message = "Error message"
    drone = factory.Iterator(Drone.objects.all())
    operation = None


class FrontEndUserInputFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = FrontEndUserInput
    operation = factory.Iterator(Operation.objects.all())
    element_name = factory.fuzzy.FuzzyText(length=50)
    active = factory.Faker('pybool')
    value = factory.fuzzy.FuzzyText(length=50)


class LoraTransmitterFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = LoraTransmitter
    tagName = factory.fuzzy.FuzzyText(length=100)
    upTime = factory.fuzzy.FuzzyFloat(low=0, high=100)
    operation = factory.Iterator(Operation.objects.all())


class LoraTransmitterLocationFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = LoraTransmitterLocation
    lat = factory.fuzzy.FuzzyFloat(low=0, high=100)
    lon = factory.fuzzy.FuzzyFloat(low=0, high=100)
    loraTransmitter = factory.Iterator(LoraTransmitter.objects.all())


class LidarPointSessionFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = LidarPointSession
    user = factory.Iterator(User.objects.all())
    operation = factory.Iterator(Operation.objects.all())
    drone = factory.Iterator(Drone.objects.all())
    is_active = factory.Faker('pybool')


class LidarPointFactory(factory.django.DjangoModelFactory):
    class Meta:
        model = LidarPoint
    points = factory.fuzzy.FuzzyText(length=100)
    lat = factory.fuzzy.FuzzyFloat(low=0, high=100)
    lon = factory.fuzzy.FuzzyFloat(low=0, high=100)
    drone = factory.Iterator(Drone.objects.all())
    lidar_point_session = factory.Iterator(LidarPointSession.objects.all())


def create_groups():
    new_group, created = Group.objects.get_or_create(name='edit_permissions')
    ct = ContentType.objects.get_for_model(User)
    permission = Permission.objects.create(codename='edit_permissions',
                                           name='edit_permissions',
                                           content_type=ct)
    new_group.permissions.add(permission)

    new_group, created = Group.objects.get_or_create(name='create_operations')
    ct = ContentType.objects.get_for_model(User)
    permission = Permission.objects.create(codename='create_operations',
                                           name='create_operations',
                                           content_type=ct)
    new_group.permissions.add(permission)

    new_group, created = Group.objects.get_or_create(name='join_operations')
    ct = ContentType.objects.get_for_model(User)
    permission = Permission.objects.create(codename='join_operations',
                                           name='join_operations',
                                           content_type=ct)
    new_group.permissions.add(permission)
