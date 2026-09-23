

-- static cameras -- TODO: stream rebroadcast to mtx
-- planes ADSB -- TODO: trigger calls to the API from the back-end
-- weather
-- population density layer
government vehicles
rescueaid



# REACTION

## General
- switch from WebRTC to LSC when LEGACY_LSC is true
- switch from Keycloak to Django authentication when KEYCLOAK_ACTIVE is false
- secure MAVLINK views functions to require user authentication
- ~~stop brodcasting and subscribing when KAFKA_ACTIVE is false~~
- ~~KC stops receiving messages after a while (fixed?)~~
- ensure (and display?) validity of homeLatitude and homeLongitude
- ~~fix operation coverage (based on drone fov?)~~
- set all timestamps to UTC (Unix?)
- ~~Operation: user input for base of operations location~~
- ~~simplify new operation form~~
- WSI: attach correct operation_id on each telemetry entry
- ~~WSI: handle incoming notifications (save to DB)~~
- 3d model tiling
- import orthophoto and 3d models


## Live Stream
- for drones who are not streaming directly to the platform rebroadcast using ffmpeg:
    - call the ffrb API to start rebroadcasting when the RTMP stream is external
- ~~find a way to get the name of the video recording mp4 and store its filename in the drone's live stream session table~~
- ~~make session replays to work with video files instead of images~~
- ~~adjust CV to capture stream frames instead of reading them from the live session table~~
- ~~stop capturing live stream frames with LSC~~
- trigger ffrb container to rebroadcast RTMP stream if the original stream has an external URL
- TESTING from Raspberry Pi webcam: 
    ffmpeg -hwaccel auto -re -i /dev/video0 -c:v libx264 -preset veryfast -bf 0 -f flv rtmp://192.168.0.17/live/tete
- ~~create admin page for live sessions list for each operation~~
- ~~create admin page for detection sessions list for each operation~~

## MAVLINK
- ~~add "configuration" field in drones table (Quad, VTOL, FW, etc.)~~
- ~~check which mode the UAV needs to be in for takeoff and mission (based on configuration?)~~
- find a way to get the VTOL state
- monitor mission progress/end
- pause/cancel mission for uav2.py
- ~~test functionality for PX4 flight controllers~~ (functional excpect VTOL state)
- PX4 stuck when uploading a mission after the first one is completed (works with MAVSDK)
- test functionality for ArduPilot flight controllers

## Crisis Classification
- ~~handle receiving crisis event with the same ID (copy description from previous instance)~~
- ~~save crisis event description~~
- ~~update event in other browser windows after it's edited~~

## Object Detections
- ~~adjust consumer for new message format~~
- ~~check detection visualization~~
- ~~mark detections as suspicious~~
- receive end_session flag in message

## Path Planning
- identify the user that requested a mission action

## Mobile Apps / DJI Drones
- check build map for v4
- fix build map for v5 (M300)
- ~~merge changes for sending pilot notifications from the platform~~
- change cameras (normal, multi-spectral)
- ~~show drone orientation on map~~
- ~~show drone home point on map~~
- receive pins from the platform

## Front-end
- ~~Fix "Vertical Separation Warning" (compares altitude with disconnected drones)~~
- ~~separate websocket connections for telemetry and stream frames~~
- get stream frames only when detection is active
- ~~show the drone's home location on map~~
- ~~show the base of operations' location on map~~
- ~~display incoming pilot notifications~~

## Simulator
- WS Devices
- Balora without ROS

## Operation Report
- ~~when a drone takes off from different locations, don't connect the paths~~
- only take in mind telemetry values that belong to the operation


#######################################


## General
- create image upload container
- ~~create algorithms container~~
- ~~create mavlink container~~
- create a go service that checks for client disconnects (ccd)
- adjust docker networking in docker-compose.yml
- have a commont time-zone in all containers
- fix geo container start-up error (map files missing?)
- ~~make a volume in all containers to import the connection.py file~~
- ~~migrate web container to this project~~
- ~~transfer odm container~~
- ~~transfer geoserver container~~
- ~~add `runtime: nvidia` for the web container if Nvidia GPU is available~~
- ~~create computer vision container~~
- remove "aiders." from all database queries


## Web Container
- ~~flag all clients as disconnected on start-up~~
- ~~sometimes missions have multiple heights:~~
    - ~~[33.44956963002767, 35.096895941629114, 100]~~,
    - ~~[33.451953029524205, 35.09738059659769, [100, 100, 100, 100]]~~,
- ~~FIX: views.py: NameError: name 'weather_station_ros_publisher' is not defined~~
- ~~FIX: HTTP POST /api/operations/xz/weather_live 500 [0.05, 192.168.0.10:47884]~~
- check /AIDERS.postman_collection.json for example POST requests
    - ~~post request to cv container to start detection~~
    - ~~post request to cv container to stop detection~~
    - post buildMapRequest to ROS container
    - ~~post start or stop lidar to ROS container~~
    - post missions to ROS container
- IMPORTANT: Check and fix timestamps to work with dynamic timezone

## ROS Container
- test drone error messages
- test drone weather station data
- test lora communication through ROS serial
- save lora monitor data
- bulk save lidar data


## CV Container
- limit detection to 5 FPS
- Error when running on GPU:
    - [TensorRT] ERROR: INVALID_CONFIG: The engine plan file is generated on an incompatible device, expecting compute 6.1 got compute 7.5, please rebuild.
    - File: "/app/object_detection/src/utils/inference.py", line 64
    - ENGINE FILE: /app/object_detection/src/models/tiny_yolov4_3l_custom_best.trt

## Launcher 
- ~~needs to wait until django is available~~


## Sim Launcher
- disable "live stream" option when nvidia container runtime is not enabled


## Sim Container
- handle incoming missions
- post photos from drones if the command is received through ROS (buildMap)
- ~~post photos from devices~~
- api to trigger client disconnects

## Geo Container
- set up run this first time before running geo container
'''bash
docker run \            
    -e DOWNLOAD_PBF=https://download.geofabrik.de/europe/cyprus-latest.osm.pbf \
    -e DOWNLOAD_POLY=https://download.geofabrik.de/europe/cyprus.poly \
    -v ./geo/osm-data:/data/database/ \
    overv/openstreetmap-tile-server \
    import
'''

### websocket queries:
    SELECT `aiders_operation`.`id`, `aiders_operation`.`active`, `aiders_operation`.`created_at`, `aiders_operation`.`description`, `aiders_operation`.`ended_at`, `aiders_operation`.`location`, `aiders_operation`.`operation_name`, `aiders_operation`.`operator_id`
    FROM `aiders_operation`
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg' LIMIT 21

    SELECT `aiders_drone`.`id`, `aiders_drone`.`drone_name`, `aiders_drone`.`ip`, `aiders_drone`.`model`, `aiders_drone`.`camera_model`, `aiders_drone`.`time`, `aiders_drone`.`operation_id`, `aiders_drone`.`is_connected_with_platform`, `aiders_drone`.`mission_id`, `aiders_drone`.`ballistic_available`, `aiders_drone`.`build_map_activated`, `aiders_drone`.`drone_movement_available`, `aiders_drone`.`lidar_available`, `aiders_drone`.`multispectral_available`, `aiders_drone`.`water_sampler_available`, `aiders_drone`.`weather_station_available`
    FROM `aiders_drone` 
    INNER JOIN `aiders_operation` ON (`aiders_drone`.`operation_id` = `aiders_operation`.`id`)
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg'

    SELECT `aiders_device`.`id`, `aiders_device`.`name`, `aiders_device`.`operator`, `aiders_device`.`ip`, `aiders_device`.`model`, `aiders_device`.`time`, `aiders_device`.`operation_id`, `aiders_device`.`is_connected_with_platform`
    FROM `aiders_device` 
    INNER JOIN `aiders_operation` ON (`aiders_device`.`operation_id` = `aiders_operation`.`id`)
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg'

    SELECT `aiders_balora`.`id`, `aiders_balora`.`time`, `aiders_balora`.`baloraMaster_id`, `aiders_balora`.`name`
    FROM `aiders_balora` 
    INNER JOIN `aiders_baloramaster` ON (`aiders_balora`.`baloraMaster_id` = `aiders_baloramaster`.`id`) 
    INNER JOIN `aiders_operation` ON (`aiders_baloramaster`.`operation_id` = `aiders_operation`.`id`)
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg'

    SELECT `aiders_weatherstation`.`id`, `aiders_weatherstation`.`time`, `aiders_weatherstation`.`wind_speed`, `aiders_weatherstation`.`wind_direction`, `aiders_weatherstation`.`temperature`, `aiders_weatherstation`.`pressure`, `aiders_weatherstation`.`humidity`, `aiders_weatherstation`.`heading`, `aiders_weatherstation`.`operation_id`, `aiders_weatherstation`.`drone_id`
    FROM `aiders_weatherstation` 
    INNER JOIN `aiders_operation` ON (`aiders_weatherstation`.`operation_id` = `aiders_operation`.`id`)
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg' 
    ORDER BY `aiders_weatherstation`.`id` DESC LIMIT 1

    SELECT `aiders_operation`.`id`, `aiders_operation`.`active`, `aiders_operation`.`created_at`, `aiders_operation`.`description`, `aiders_operation`.`ended_at`, `aiders_operation`.`location`, `aiders_operation`.`operation_name`, `aiders_operation`.`operator_id`
    FROM `aiders_operation`
    WHERE `aiders_operation`.`operation_name` = 'gdfdfg' LIMIT 21

    SELECT `aiders_manuallysetobject`.`id`, `aiders_manuallysetobject`.`created_by_id`, `aiders_manuallysetobject`.`created_at`, `aiders_manuallysetobject`.`operation_id`, `aiders_manuallysetobject`.`description`, `aiders_manuallysetobject`.`label`
    FROM `aiders_manuallysetobject`
    WHERE `aiders_manuallysetobject`.`operation_id` = 1





SIDEBAR STRUCTURE
-----------------

Load build map

Ground Weather Data

Custom Pins

Trajectories
Operation Coverage 

Stream replay

Vertical Separation Warning

Object Detection


View or Load Algorithms
    - fire spread
    - ortho
    - 3d object
    - mission path


Calculate Fire Spread

3D Model & Orthophoto

3D Points & Meshes
    - load 3d points & meshes
    - process 3d meshes