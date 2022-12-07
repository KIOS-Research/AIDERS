from django.conf import settings
from django.conf.urls.static import static
from django.contrib.auth import get_user_model
from django.urls import include, path, re_path
from django.views.generic import RedirectView
from drf_yasg import openapi
from drf_yasg.views import get_schema_view

from .forms import (OperationFormEditStep1, OperationFormStep1,
                    OperationFormStep2)
from .views import *

User = get_user_model()
schema_view = get_schema_view(
    openapi.Info(
        title="AIDERS API",
        default_version='v1',
        description="Test description",
        terms_of_service="https://www.google.com/policies/terms/",
        contact=openapi.Contact(email="contact@snippets.local"),
        license=openapi.License(name="BSD License"),
    ),
    public=True
)


dronepatterns = [
    path('', DroneRetrieveAPIView.as_view(), name='drone'),
    path('detection', DetectionRetrieveAPIView.as_view(), name='detection'),
    path('live_stream_status', live_stream_status_api_view,
         name='live_stream_status'),
    path('mission_points', MissionPointsListCreateAPIView.as_view(),
         name='mission_points'),
    path('mission', ExecuteMissionAPIView.as_view(), name='mission'),
    path('telemetry', TelemetryRetrieveAPIView.as_view(), name='telemetry'),

]

lorapatterns = [
    path('live_info', LoraTransmitterLocationRetrieveAPIView.as_view(),
         name='lora_live_info')
]
apipatterns = [
    path('algorithm/execute', ExecuteAlgorithmAPIView.as_view(),
         name='algorithm_execute'),
    path('algorithms/', AlgorithmListView.as_view(), name='algorithms'),
    path('algorithms/<int:pk>/<str:attr>',
         AlgorithmRetrieveView.as_view(), name='algorithm_result'),
    path('buildMapSessions', buildMapSessionsAPIView.as_view(),
         name='build_map_sessions'),
    path('buildMapSessions/<int:pk>/',
         buildMapSessionsShareAPIView.as_view(), name='build_map_session_share'),
    path('control_device/', ControlDeviceDataAPIView.as_view(),
         name='control_device'),
    path('detection_types', detection_types_api_view, name='detection_types'),
    path('drones/', DroneListCreateAPIView.as_view(), name='drones'),
    path('drones/<drone_name>/',  include(dronepatterns)),
    path('fire_prediction', FirePredictionCreateAPIView.as_view(),
         name='fire_prediction'),
    path('front_end_actions', frontEndUserInputAPIView.as_view(),
         name='front_end_actions'),
    path('flying_report', FlyingReportAPIView.as_view(), name='flying_report'),
    path('flying_reports', FlyingReportTableAPIView.as_view(),
         name='flying_reports'),
    path('get_last_build_map_image', BuildMapGetLastImageAPIView.as_view(),
         name='get_last_build_map_image'),
    path('get_last_build_map', BuildMapGetLastAPIView.as_view(),
         name='get_last_build_map'),
    path('lidar_3d_meshes', Lidar3DMesh.as_view(), name='lidar_3d_meshes'),
    path('lidar_3d_points', Lidar3DPoints.as_view(), name='lidar_3d_points'),
    path('lidar_process', Lidar_process_cloud_points.as_view(), name='lidar_process'),
    path('load_build_map', BuildMapLoadAPIView.as_view(), name='load_build_map'),
    path('lora_devices', LoraTransmiterListAPIView.as_view(), name='lora_devices'),
    path('lora_devices/<lora_device_name>/', include(lorapatterns)),
    path('missions_logger/', MissionLoggerListCreateAPIView.as_view(),
         name='missions_logger'),
    path('missions/', MissionListCreateAPIView.as_view(), name='missions'),
    path('replay_mission', MissionRetrieveAPIView.as_view(), name='replay_mission'),
    path('replay_mission/<mission_id>',
         ReplayMissionOnlineAPIView.as_view(), name='replay_mission_engine'),
    path('start_build_map', BuildMapAPIView.as_view(), name='start_build_map'),
    path('telemetries', TelemetryListCreateAPIView.as_view(), name='telemetries'),
    path('water_collection_activated', waterCollectionActivatedAPIView.as_view(
    ), name='water_collection_activated'),
    path('ballistic', ballisticActivatedAPIView.as_view(
    ), name='ballistic'),
    path('range_finder', rangeFinderAPIView.as_view(
    ), name='range_finder'),
    path('weather_live', WeatherLiveAPIView.as_view(), name='weather_live'),
    path('weather_station', WeatherStationAPIView.as_view(), name='weather_station'),
]

operation_form_new = [OperationFormStep1, OperationFormStep2]
operation_form_edit = [OperationFormEditStep1, OperationFormStep2]

urlpatterns = [
    path('api/operations/<operation_name>/', include(apipatterns)),
    path('api/operations/', OperationListCreateAPIView.as_view(), name='operations'),

    path('operations/new', NewOperationForm.as_view(operation_form_new),
         name='new_operation'),
    path('operations/edit/<operation_name>',
         EditOperationForm.as_view(operation_form_edit), name='edit_operation'),
    path('operations/join/<operation_name>',
         join_operation_view, name='join_operation'),
    path('operations/stop/<operation_name>',
         stop_operation_view, name='stop_operation'),
    path('operations/leave', leave_operation_view, name='leave_operation'),

    path('fillDummy/', DatabaseFiller.as_view(), name='fill_dummy_data'),

    path('manage/users/', UserList.as_view(), name='users'),
    path('users/<int:pk>/', UserDetail.as_view(), name='user_detail'),
    path('manage/drones/', DroneList.as_view(), name='drones_list'),
    path('manage/drones/<drone_name>/operation',
         DroneModifyOperationView.as_view(), name='drone_modify_operation'),

    path('login/', login_view, name='login'),
    path('logout/', logout_view, name='logout'),
    path('register/', register_request, name='register'),


    path('home', index, name='home'),

    path('postBuildMapImg/', BuildMapImageView, name='build_map_img'),

    path('manage/operations/', ManageOperationsView.as_view(),
         name='manage_operations'),
    path('manage/permissions/', ManagePermissionsView.as_view(),
         name='manage_permissions'),
    path('manage/permissions/<user_name>', ManageUserPermissionsView.as_view(),
         name='manage_permissions_user'),
    path('settings/', settings_view, name='settings'),

    path('Health-Check/Monitoring_Platform',
         SystemMonitoringView.as_view(), name='platform_monitoring'),
    path('Health-Check/Monitoring_Control_Devices',
         ControlDevicesMonitoringView.as_view(), name='control_devices_monitoring'),
    path('Health-Check/Monitoring_Control_Devices/<control_device>',
         ControlDeviceMonitoringView.as_view(), name='control_device_monitoring'),

    path('', include('django.contrib.auth.urls')),
    path('', RedirectView.as_view(pattern_name='login', permanent=False)),
    path('api-auth/', include('rest_framework.urls'), name='api_auth'),
    path('testing', TestingBuildMap.as_view(), name='testing'),
    re_path(r'^redoc/$', schema_view.with_ui('redoc',
            cache_timeout=0), name='schema-redoc'),
    re_path(r'^swagger(?P<format>\.json|\.yaml)$',
            schema_view.without_ui(cache_timeout=0), name='schema-json'),
    re_path(r'^swagger/$', schema_view.with_ui('swagger',
            cache_timeout=0), name='schema-swagger-ui'),

]

urlpatterns += static(settings.MEDIA_URL, document_root=settings.MEDIA_ROOT)
urlpatterns += static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
