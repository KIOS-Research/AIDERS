from aiders.views import (
    AlgorithmListView,
    AlgorithmRetrieveView,
    BuildMapImageView,
    BuildMapLoadAPIView,
    BuildMapStartOrStopSession,
    ControlDeviceDataAPIView,
    ControlDeviceMonitoringView,
    ControlDevicesMonitoringView,
    DatabaseFiller,
    DetectionRetrieveAPIView,
    DroneList,
    DroneListCreateAPIView,
    DroneModifyOperationView,
    DroneRetrieveAPIView,
    ExecuteAlgorithmAPIView,
    ExecuteMissionAPIView,
    FirePredictionCreateAPIView,
    ManageOperationsView,
    ManagePermissionsView,
    MissionListCreateAPIView,
    MissionLoggerListCreateAPIView,
    MissionPointsListCreateAPIView,
    MissionRetrieveAPIView,
    OperationListCreateAPIView,
    ReplayMissionOnlineAPIView,
    SystemMonitoringView,
    TelemetryListCreateAPIView,
    TelemetryRetrieveAPIView,
    UserDetail,
    UserList,
    WeatherStationAPIView,
    buildMapSessionsAPIView,
    buildMapSessionsShareAPIView,
    detection_types_api_view,
    frontEndUserInputAPIView,
    index,
    join_operation_view,
    leave_operation_view,
    login_view,
    logout_view,
    register_request,
    stop_operation_view,
)
from django.test import SimpleTestCase
from django.urls import resolve, reverse


class TestWebAppUrls(SimpleTestCase):
    def test_list_operations_url_resolves(self):
        url = reverse("operations")
        self.assertEquals(resolve(url).func.view_class, OperationListCreateAPIView)
        # assert 1==2

    def test_join_operation_url_resolves(self):
        url = reverse("join_operation", args=["some_operation_name"])
        self.assertEquals(resolve(url).func, join_operation_view)

    def test_stop_operation_url_resolves(self):
        url = reverse("stop_operation", args=["some_operation_name"])
        self.assertEquals(resolve(url).func, stop_operation_view)

    def test_leave_operation_url_resolves(self):
        url = reverse("leave_operation")
        self.assertEquals(resolve(url).func, leave_operation_view)

    def test_fill_dummy_url_resolves(self):
        url = reverse("fill_dummy_data")
        self.assertEquals(resolve(url).func.view_class, DatabaseFiller)

    def test_list_drones_url_resolves(self):
        url = reverse("drones_list")
        self.assertEquals(resolve(url).func.view_class, DroneList)

    def test_change_drones_operation(self):
        url = reverse("algorithms", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, AlgorithmListView)

    def test_algorithm_result_operation(self):
        some_algorithm_id = 2
        url = reverse("algorithm_result", args=["some_operation_name", some_algorithm_id, "input"])
        self.assertEquals(resolve(url).func.view_class, AlgorithmRetrieveView)

    def test_algorithm_execute_operation(self):
        url = reverse("algorithm_execute", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, ExecuteAlgorithmAPIView)

    def test_change_drones_operation(self):
        url = reverse("drone_modify_operation", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, DroneModifyOperationView)

    def test_list_users_url_resolves(self):
        url = reverse("users")
        self.assertEquals(resolve(url).func.view_class, UserList)

    def test_detail_user_url_resolves(self):
        some_user_id = 2
        url = reverse("user_detail", args=[some_user_id])
        self.assertEquals(resolve(url).func.view_class, UserDetail)

    def test_list_drones_url_resolves(self):
        url = reverse("drones_list")
        self.assertEquals(resolve(url).func.view_class, DroneList)

    def test_manage_operations_url_resolves(self):
        url = reverse("manage_operations")
        self.assertEquals(resolve(url).func.view_class, ManageOperationsView)

    def test_manage_permissions_url_resolves(self):
        url = reverse("manage_permissions")
        self.assertEquals(resolve(url).func.view_class, ManagePermissionsView)

    def test_platform_monitoring_url_resolves(self):
        url = reverse("platform_monitoring")
        self.assertEquals(resolve(url).func.view_class, SystemMonitoringView)

    def test_control_devices_monitoring_url_resolves(self):
        url = reverse("control_devices_monitoring")
        self.assertEquals(resolve(url).func.view_class, ControlDevicesMonitoringView)

    def test_control_device_monitoring_url_resolves(self):
        url = reverse("control_device_monitoring", args=["some_control_device"])
        self.assertEquals(resolve(url).func.view_class, ControlDeviceMonitoringView)

    def test_login_url_resolves(self):
        url = reverse("login")
        self.assertEquals(resolve(url).func, login_view)

    def test_logout_url_resolves(self):
        url = reverse("logout")
        self.assertEquals(resolve(url).func, logout_view)

    def test_register_url_resolves(self):
        url = reverse("register")
        self.assertEquals(resolve(url).func, register_request)

    def test_home_url_resolves(self):
        url = reverse("home")
        self.assertEquals(resolve(url).func, index)

    def test_fire_prediction_url_resolves(self):
        url = reverse("fire_prediction", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, FirePredictionCreateAPIView)

    def test_start_build_map_url_resolves(self):
        url = reverse("start_build_map", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, BuildMapStartOrStopSession)

    def test_missions_logger_url_resolves(self):
        url = reverse("missions_logger", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, MissionLoggerListCreateAPIView)

    def test_replay_mission_url_resolves(self):
        url = reverse("replay_mission", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, MissionRetrieveAPIView)

    def test_replay_mission_url_resolves(self):
        url = reverse("replay_mission_engine", args=["some_operation_name", 1])
        self.assertEquals(resolve(url).func.view_class, ReplayMissionOnlineAPIView)

    def test_start_build_map_url_resolves(self):
        url = reverse("start_build_map", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, BuildMapStartOrStopSession)

    def test_control_device_url_resolves(self):
        url = reverse("control_device", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, ControlDeviceDataAPIView)

    def test_weather_station_url_resolves(self):
        url = reverse("weather_station", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, WeatherStationAPIView)

    def test_build_map_sessions_url_resolves(self):
        url = reverse("build_map_sessions", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, buildMapSessionsAPIView)

    def test_build_map_sessions_shared_url_resolves(self):
        url = reverse("build_map_session_share", args=["some_operation_name", 0])
        self.assertEquals(resolve(url).func.view_class, buildMapSessionsShareAPIView)

    def test_water_collection_activated_url_resolves(self):
        url = reverse("water_collection_activated", args=["some_operation_name", 0])
        self.assertEquals(resolve(url).func.view_class, buildMapSessionsShareAPIView)

    def test_water_collection_activated_url_resolves(self):
        url = reverse("front_end_actions", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, frontEndUserInputAPIView)

    def test_detection_types_url_resolves(self):
        url = reverse("detection_types", args=["some_operation_name"])
        self.assertEquals(resolve(url).func, detection_types_api_view)

    def test_get_load_build_map_url_resolves(self):
        url = reverse("load_build_map", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, BuildMapLoadAPIView)

    def test_get_build_map_image_url_resolves(self):
        url = reverse("build_map_img")
        self.assertEquals(resolve(url).func, BuildMapImageView)


class TestAPIUrls(SimpleTestCase):
    def test_list_drones_url_resolves(self):
        some_operation_id = 3
        url = reverse("drones", args=[some_operation_id])
        self.assertEquals(resolve(url).func.view_class, DroneListCreateAPIView)

    def test_telemetries_url_resolves(self):
        url = reverse("telemetries", args=["some_operation_name"])
        self.assertEquals(resolve(url).func.view_class, TelemetryListCreateAPIView)

    def test_retrieve_drone_url_resolves(self):
        some_operation_id = 3
        some_drone_id = "kios_mavic2h"
        url = reverse("drone", args=[some_operation_id, some_drone_id])
        self.assertEquals(resolve(url).func.view_class, DroneRetrieveAPIView)

    def test_retrieve_mission_url_resolves(self):
        some_operation_id = 4
        some_drone_id = "kios_mavic2g"
        url = reverse("mission", args=[some_operation_id, some_drone_id])
        self.assertEquals(resolve(url).func.view_class, ExecuteMissionAPIView)

    def test_retrieve_mission_points_url_resolves(self):
        some_operation_id = 5
        some_drone_id = "kios_mavic2e"
        url = reverse("mission_points", args=[some_operation_id, some_drone_id])
        self.assertEquals(resolve(url).func.view_class, MissionPointsListCreateAPIView)

    def test_retrieve_telemetry_url_resolves(self):
        some_operation_id = 6
        some_drone_id = "kios_mavic2j"
        url = reverse("telemetry", args=[some_operation_id, some_drone_id])
        self.assertEquals(resolve(url).func.view_class, TelemetryRetrieveAPIView)

    def test_list_missions_url_resolves(self):
        some_operation_id = 6
        url = reverse("missions", args=[some_operation_id])
        self.assertEquals(resolve(url).func.view_class, MissionListCreateAPIView)

    def test_list_detection_drones_url_resolves(self):
        some_operation_id = 6
        some_drone_id = "kios_mavic2j"
        url = reverse("detection", args=[some_operation_id, some_drone_id])
        self.assertEquals(resolve(url).func.view_class, DetectionRetrieveAPIView)
