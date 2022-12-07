
from aiders import models
from aiders.factories import OperationFactory
from django.contrib.staticfiles import finders
from logic import utils

from .test_setup import TestSetUp


class TestPlatformFeatures(TestSetUp):
    def test_fire_prediction_algorithm(self):
        op = OperationFactory.create(operation_name=self.operation_name, operator=self.user)
        operation_pk=op.pk
        input_data={'fire_speed': 10, 'fire_fronts': 10, 'wind_speed': 10, 'wind_angle': 0.17453292519943295, 'time_steps': {'value': 101, 'units': 'seconds'}, 'location': {'lon': 33.00113031193169, 'lat': 35.072905230383654}, 'time_intervals': [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]}
        can_be_loaded_on_map=True
        algorithm_name="FIRE_PROPAGATION_ALGORITHM"
        user_pk=self.user.pk
        output = utils.handleAlgorithmExecution(operation_pk,input_data,can_be_loaded_on_map,algorithm_name,user_pk)
        try:
            algorithm_data=models.Algorithm.objects.get(input=input_data)
            assert(True)
        except:
            assert(False)

    def test_map_layer_cyprus_eca_lines(self):
        cyprus_eca_lines = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_aikpilwnes_lines.geojson')
        if cyprus_eca_lines == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_eca_poles(self):
        cyprus_eca_poles = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_aikpilwnes.geojson')
        if cyprus_eca_poles == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_buildings(self):
        cyprus_buildings = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_buildings.geojson')
        if cyprus_buildings == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_dams(self):
        cyprus_dams = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_cyprus_dams.geojson')
        if cyprus_dams == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_hospitals(self):
        cyprus_hospitals = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_cyprus_hospitals.geojson')
        if cyprus_hospitals == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_roads_network(self):
        cyprus_roads_network = finders.find('aiders/cyprus_geolocation/aidersplatform_geojson_files_roadnetwork_original.geojson')
        if cyprus_roads_network == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_dams_icon(self):
        cyprus_dams_icon = finders.find('aiders/cyprus_geolocation_icons/aidersplatform_geojson_files_dam_icon.png')
        if cyprus_dams_icon == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_hospitals_icon(self):
        cyprus_hospitals_icon = finders.find('aiders/cyprus_geolocation_icons/aidersplatform_geojson_files_hospital_icon.png')
        if cyprus_hospitals_icon == None:
            assert(False)
        else:
            assert(True)

    def test_map_layer_cyprus_eca_poles_icon(self):
        cyprus_eca_poles_icon = finders.find('aiders/cyprus_geolocation_icons/aidersplatform_geojson_files_mv_pole.png')
        if cyprus_eca_poles_icon == None:
            assert(False)
        else:
            assert(True)
