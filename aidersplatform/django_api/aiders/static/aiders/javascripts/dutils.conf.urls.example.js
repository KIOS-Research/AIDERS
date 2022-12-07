/*
 * This file contains all of the names of the url end points from django API
 * and their respective full path url
 * This is used by utilizing the 'dutils.js' file in this directory
 * */

// TODO: Add a test that checks the following: Every url route on django should be present
//  in the following object
let operation_route = '/api/operations/<operation_name>';
let drone_route = operation_route + '/drones/<drone_name>';
let lora_route = operation_route + '/lora_devices/<lora_device_name>';
let media = '/media/';
let static_ = '/static/';

dutils.conf.urls = {
    // WebSockets
    ws_index: 'ws/index/',
    ws_monitoring: 'ws/monitoring/',
    // Paths without Arguments
    drone_modify_operation: '<drone_name>/operation',
    join_operation: '/operations/join/',
    leave_operation: '/operations/leave/',
    new_operation: '/operations/new/',
    odm: '',
    stop_operation: '/operations/stop/<operation_name>/',
    manage_permissions_user: '/manage/permissions/<user_name>',

    // Paths start with drone_route argument
    last_detection_frame: drone_route + '/last_detection_frame',
    mission_points: drone_route + '/mission_points',
    mission: drone_route + '/mission',
    objects_detected_on_last_frame: drone_route + '/objects_detected_on_last_frame',
    start_stop_detection: drone_route + '/start_stop_detection',
    telemetry: drone_route + '/telemetry',

    // Paths start with media argument
    build_map_image_path: media + '<image_path>/',
    detection_frame_path: media + '<frame_path>/',
    live_frame_path: media + '<frame_path>/',
    media: media + '<path>/',

    // Paths start with operation_route argument
    algorithm_execute: operation_route + '/algorithm/execute',
    algorithms: operation_route + '/algorithms/',
    detection_types: operation_route + '/detection_types',
    drone: operation_route + '/drones/<drone_name>/',
    drones: operation_route + '/drones/',
    fire_prediction: operation_route + '/fire_prediction',
    front_end_actions: operation_route + '/front_end_actions',
    get_last_build_map_image: operation_route + '/get_last_build_map_image',
    get_last_build_map: operation_route + '/get_last_build_map',
    lidar_3d_meshes: operation_route + '/lidar_3d_meshes',
    lidar_3d_points: operation_route + '/lidar_3d_points',
    lidar_process: operation_route + '/lidar_process',
    load_build_map: operation_route + '/load_build_map',
    operations: operation_route,
    replay_mission: operation_route + '/replay_mission',
    start_build_map: operation_route + '/start_build_map',
    water_collection_activated: operation_route + '/water_collection_activated',
    weather_live: operation_route + '/weather_live',
    weather_station: operation_route + '/weather_station',

    // Paths start with static argument
    cyprus_geolocation_icons: static_ + 'aiders/cyprus_geolocation_icons/<icon_path>',
    cyprus_geolocation: static_ + 'aiders/cyprus_geolocation/<geolocation_path>',
    drone_glb_model: static_ + '/aiders/models/drone.glb',
    home: '/home',
    last_raw_frame: drone_route + '/last_raw_frame',
    live_stream_status: drone_route + '/live_stream_status',
    lora_devices: operation_route + '/lora_devices',
    lora_live_info: lora_route + '/live_info',
};
