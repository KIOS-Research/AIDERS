from django.conf import settings
from django.conf.urls.static import static
from django.contrib.auth import get_user_model
from django.urls import include, path, re_path
from django.views.generic import RedirectView
from drf_yasg import openapi
from drf_yasg.views import get_schema_view

from .views import *

User = get_user_model()
schema_view = get_schema_view(
    openapi.Info(
        title="AIDERS API",
        default_version="v1",
        description="Test description",
        terms_of_service="https://www.google.com/policies/terms/",
        contact=openapi.Contact(email="contact@snippets.local"),
        license=openapi.License(name="BSD License"),
    ),
    public=True,
)


droneUrls = [
    path("", DroneRetrieveAPIView.as_view(), name="drone"),
    path("detection", DetectionRetrieveAPIView.as_view(), name="detection"),
    path("detectionStartOrStop", DetectionStartOrStopAPIView.as_view(), name="detection_start_or_stop"),
    
    # Lidar
    path("lidarStartOrStop", LidarStartOrStopAPIView.as_view(), name="lidar_start_or_stop"),

    path("live_stream_status", live_stream_status_api_view, name="live_stream_status"),
    path("mission_points", MissionPointsListCreateAPIView.as_view(), name="mission_points"),
    path("mission", ExecuteMissionAPIView.as_view(), name="mission"),
    path("telemetry", TelemetryRetrieveAPIView.as_view(), name="telemetry"),
]

operationUrls = [
    path("algorithm/execute", ExecuteAlgorithmAPIView.as_view(), name="algorithm_execute"),
    path("algorithms/", AlgorithmListView.as_view(), name="algorithms"),
    path("algorithms/<int:pk>/<str:attr>", AlgorithmRetrieveView.as_view(), name="algorithm_result"),
    path("ballistic", ballisticActivatedAPIView.as_view(), name="ballistic"),
    path("balora_pm25", BaloraPM25APIView.as_view(), name="balora_pm25"),

    # Build Map Urls
    path("buildMapSessions", buildMapSessionsAPIView.as_view(), name="build_map_sessions"),
    path("buildMapSessions/<int:pk>/", buildMapSessionsShareAPIView.as_view(), name="build_map_session_share"),
    path("getBuildMapSessions", BuildMapLoadAPIView.as_view(), name="getBuildMapSessions"),
    path("buildMapStartOrStop", BuildMapStartOrStopSession.as_view(), name="buildMapStartOrStop"),
    path("getActiveBuildMapSessionImages", buildMapGetLatestImages, name="getActiveBuildMapSessionImages"),
    path("getBuildMapImagesBySessionId", buildMapGetLatestImagesBySessionId, name="getBuildMapImagesBySessionId"),

    path("control_device/", ControlDeviceDataAPIView.as_view(), name="control_device"),
    path("detection_types", detection_types_api_view, name="detection_types"),
    path("drones/", DroneListCreateAPIView.as_view(), name="drones"),
    path("drones/<drone_name>/", include(droneUrls)),
    path("external_api", ExternalAPI.as_view(), name="external_api"),
    path("fire_prediction", FirePredictionCreateAPIView.as_view(), name="fire_prediction"),
    path("flying_report", FlyingReportAPIView.as_view(), name="flying_report"),
    path("flying_reports", FlyingReportTableAPIView.as_view(), name="flying_reports"),
    path("front_end_actions", frontEndUserInputAPIView.as_view(), name="front_end_actions"),

    # Lidar Urls
    path("getLidarSessionOfPoints", getLidarSessionOfPoints, name="getLidarSessionOfPoints"),
    path("getLidarSessionPointsBySessionId", getLidarPointsBySessionId, name="getLidarSessionPointsBySessionId"),
    path("getLidarSessionPointsThatAreNotProcessed", getLidarSessionOfPointsThatAreNotProcessed, name="getLidarSessionPointsThatAreNotProcessed"),
    path("processLidarSessionPointsBySessionId", processLidarSessionPointsBySessionId, name="processLidarSessionPointsBySessionId"),
    path("getLidarSessionWithProcessMesh", getLidarSessionWithProcessMesh, name="getLidarSessionWithProcessMesh"),
    path("getLidarMeshDataNeededForVisualizationBySessionId", getLidarMeshDataNeededForVisualizationBySessionId, name="getLidarMeshDataNeededForVisualizationBySessionId"),

    path("missions_logger/", MissionLoggerListCreateAPIView.as_view(), name="missions_logger"),
    path("missions/", MissionListCreateAPIView.as_view(), name="missions"),
    path("range_finder", rangeFinderAPIView.as_view(), name="range_finder"),
    path("replay_mission", MissionRetrieveAPIView.as_view(), name="replay_mission"),
    path("replay_mission/<replay_session_id>", ReplayMissionOnlineAPIView.as_view(), name="replay_mission_engine"),
    path("telemetries", TelemetryListCreateAPIView.as_view(), name="telemetries"),
    path("water_collection_activated", waterCollectionActivatedAPIView.as_view(), name="water_collection_activated"),
    path("weather_station", WeatherStationAPIView.as_view(), name="weather_station"),

    path("set_detected_object_description/<track_id>" , DetectedObjectDescriptionSetPIView.as_view(), name="set_detected_object_description"),


    path("getLatestManualObjects", getLatestManuallySetObject, name="getLatestManualObjects"),
    path("add_manual_object", ManuallySetObjectAddAPIView.as_view(), name="add_manual_object"),
    path("add_manual_object_location", ManuallySetObjectLocationAddAPIView.as_view(), name="add_manual_object_location"),
    path("update_manual_object/<id>", ManuallySetObjectUpdateAPIView.as_view(), name="update_manual_object"),

    # session replays - get available sessions
    path("getAvailableDroneSessions/<stream_type>", getAvailableDroneSessions, name="getAvailableDroneSessions"),
    path("getAvailableDeviceSessions", getAvailableDeviceSessions, name="getAvailableDeviceSessions"),

]

urlpatterns = [
    path("api/operations/", OperationListCreateAPIView.as_view(), name="operations"),
    path("api/operations/<operation_name>/", include(operationUrls)),
    path("baloras/", BaloraList.as_view(), name="baloras_list"),
    path("devices/", DeviceList.as_view(), name="devices_list"),
    path("devices/<device_name>/new_session", DeviceNewSessionView.as_view(), name="device_new_session"),
    path("devices/<device_name>/operation", DeviceModifyOperationView.as_view(), name="device_modify_operation"),
    path("devices/<device_name>/stop_session", DeviceStopSessionView.as_view(), name="device_stop_session"),
    path("devices/getActiveDeviceSessionImages", getActiveDeviceSessionImagesByDeviceId, name="active_device_session_images"),
    path("drones/", DroneList.as_view(), name="drones_list"),
    path("drones/<drone_name>/operation", DroneModifyOperationView.as_view(), name="drone_modify_operation"),
    path("baloras/<lora_name>/operation", BaloraModifyOperationView.as_view(), name="balora_modify_operation"),
    path("fillDummy/", DatabaseFiller.as_view(), name="fill_dummy_data"),
    path("Health-Check/Monitoring_Control_Devices", ControlDevicesMonitoringView.as_view(), name="control_devices_monitoring"),
    path("Health-Check/Monitoring_Control_Devices/<control_device>", ControlDeviceMonitoringView.as_view(), name="control_device_monitoring"),
    path("Health-Check/Monitoring_Platform", SystemMonitoringView.as_view(), name="platform_monitoring"),
    path("home", index, name="home"),
    path("login/", login_view, name="login"),
    path("logout/", logout_view, name="logout"),
    path("operations/", ManageOperationsView.as_view(), name="manage_operations"),
    path("operations/edit/<operation_name>", edit_operation_form_view, name="edit_operation"),
    path("operations/join/<operation_name>", join_operation_view, name="join_operation"),
    path("operations/leave", leave_operation_view, name="leave_operation"),
    path("operations/new", new_operation_form_view, name="new_operation"),
    path("operations/stop/<operation_name>", stop_operation_view, name="stop_operation"),
    path("permissions/", ManagePermissionsView.as_view(), name="manage_permissions"),
    path("permissions/<user_name>", ManageUserPermissionsView.as_view(), name="manage_permissions_user"),
    path("postBuildMapImg/", BuildMapImageView, name="build_map_img"),
    path("postDataImg/", DataImageView, name="data_img"),
    path("postDeviceImg/", DeviceImageView, name="device_img"),
    path("register/", register_request, name="register"),
    path("settings/", settings_view, name="settings"),
    path("safeDronesStart", safeDronesStart, name="safeDronesStart"),
    path("safeDronesStop", safeDronesStop, name="safeDronesStop"),
    path("safeDronesResults", safeDronesResults, name="safeDronesResults"),
    path("users/", UserList.as_view(), name="users"),
    path("users/<int:pk>/", UserDetail.as_view(), name="user_detail"),
    path("", include("django.contrib.auth.urls")),
    path("", RedirectView.as_view(pattern_name="login", permanent=False)),
    path("api-auth/", include("rest_framework.urls"), name="api_auth"),
    re_path(r"^redoc/$", schema_view.with_ui("redoc", cache_timeout=0), name="schema-redoc"),
    re_path(r"^swagger(?P<format>\.json|\.yaml)$", schema_view.without_ui(cache_timeout=0), name="schema-json"),
    re_path(r"^swagger/$", schema_view.with_ui("swagger", cache_timeout=0), name="schema-swagger-ui"),

    # TOKEN VALIDATION
    # path('validate-token/', validate_token, name='validate-token'),

    # MAVLINK
    path('mavlinkAdd', mavlinkAddFormView, name='mavlinkAdd'),
    path('mavlinkEdit/<pk>', mavlinkEditFormView, name='mavlinkEdit'),
    path('mavlinkManage/<pk>', mavlinkManageView, name='mavlinkManage'),
    path('mavlinkCheckConnection/<pk>', mavlinkCheckConnection, name='mavlinkCheckConnection'),
    path('mavlinkGetLogs/<operation_id>/<last_log_id>', mavlinkGetLogs, name='mavlinkGetLogs'),

    path("mavlinkConnect", mavlinkConnect, name="mavlinkConnect"),
    path("mavlinkDisconnect", mavlinkDisconnect, name="mavlinkDisconnect"),

    path("mavlinkTakeoff", mavlinkTakeoff, name="mavlinkTakeoff"),
    path("mavlinkLand", mavlinkLand, name="mavlinkLand"),
    path("mavlinkReturnHome", mavlinkReturnHome, name="mavlinkReturnHome"),
    path("mavlinkTransition", mavlinkTransition, name="mavlinkTransition"),
    path("mavlinkSetSpeed", mavlinkSetSpeed, name="mavlinkSetSpeed"),
    path("mavlinkArm", mavlinkArm, name="mavlinkArm"),
    path("mavlinkDisarm", mavlinkDisarm, name="mavlinkDisarm"),
    path("mavlinkKill", mavlinkKill, name="mavlinkKill"),

    # operation coverage points
    path("coverage_points/<operation_name>", operation_coverage_points, name="coverage-points"),

    # drone session replay
    path("drone_session_replay/<stream_type>/<session_id>", drone_session_replay, name="drone-session-replay"),
    path("device_session_replay/<session_id>", device_session_replay, name="device-session-replay"),
]

urlpatterns += static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)
urlpatterns += static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
