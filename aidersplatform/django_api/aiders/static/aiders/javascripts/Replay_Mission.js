let currentVisualizationValue;
let droneTelemetryOnChange = {};
let droneTimer;
let new_time_data = true;
visualization_player = document.createElement('INPUT');
visualization_player.setAttribute('type', 'range');
visualization_player.min = start_time.getTime();
visualization_player.max = end_time.getTime();
let list_user_action_menu = [
    'uav-missions-ui',
    'map-tools-ui',
    'enable-detections-ui',
    'algorithms-ui',
    'uavs-trajectory-ui',
    'detections-ui',
    'video-feeds-ui',
    'detection-video-feeds-ui',
    'map-styles-ui',
    'map-layers-ui',
];
let element_user_action_menu;

visualization_player.addEventListener('change', function () {
    check_timer();
    show_timer();
    $('#videoLoading').hide();
});
visualization_player.addEventListener(
    'input',
    function () {
        check_timer();
        show_timer();
        edit_timer();
        $('#videoLoading').show();
    },
    false
);

function edit_timer() {
    timer1 = parseInt(currentVisualizationValue);
    timer2 = parseInt(visualization_player.value);
    if (timer1 < timer2) {
        for (let i = timer1; i < timer2; i = i + 10) {
            if (data.time_series_data[i.toString()]) {
                Object.keys(data.time_series_data[i.toString()]).forEach(function (key) {
                    if (key.split(' ')[0] === 'build_map_image') {
                        run_data({ build_map_image: data.time_series_data[i.toString()][key] });
                    } else if (key.split(' ')[0] === 'detected_object') {
                        run_data({ detected_object: data.time_series_data[i.toString()][key] });
                    } else if (key.split(' ')[0] === 'detection_frame') {
                        run_data({ detection_frame: data.time_series_data[i.toString()][key] });
                    } else if (key.split(' ')[0] === 'telemetry') {
                        droneTelemetryOnChange[data.time_series_data[i.toString()][key]['drone_id']] =
                            data.time_series_data[i.toString()][key];
                    } else if (key.split(' ')[0] === 'user_input') {
                        if (
                            document.getElementById(data.time_series_data[i.toString()][key].element_name).checked !==
                            data.time_series_data[i.toString()][key].active
                        ) {
                            run_data({ user_input: data.time_series_data[i.toString()][key] });
                        }
                    } else if (key.split(' ')[0] === 'algorithm') {
                        run_data({ algorithm: data.time_series_data[i.toString()][key] });
                    } else if (key.split(' ')[0] === 'error') {
                        run_data({ error: data.time_series_data[i.toString()][key] });
                    }
                });
            }
        }
    } else {
        for (let i = timer1; i > timer2; i = i - 10) {
            if (data.time_series_data[i.toString()]) {
                Object.keys(data.time_series_data[i.toString()]).forEach(function (key) {
                    if (key.split(' ')[0] === 'build_map_image') {
                        reverse_data({ build_map_image: data.time_series_data[i.toString()][key] });
                    } else if (key.split(' ')[0] === 'detected_object') {
                        run_data({ detected_object: data.time_series_data[i.toString()] });
                    } else if (key.split(' ')[0] === 'telemetry') {
                        droneTelemetryOnChange[data.time_series_data[i.toString()][key]['drone_id']] =
                            data.time_series_data[i.toString()][key];
                    } else if (key.split(' ')[0] === 'user_input') {
                        if (
                            document.getElementById(data.time_series_data[i.toString()][key].element_name).checked !==
                            data.time_series_data[i.toString()][key].active
                        ) {
                            reverse_data({ user_input: data.time_series_data[i.toString()][key] });
                        }
                    } else if (key.split(' ')[0] === 'algorithm') {
                        reverse_data({ algorithm: data.time_series_data[i.toString()][key] });
                    }
                });
            }
        }
    }
    Object.keys(droneTelemetryOnChange).forEach(function (key) {
        run_data({ telemetry: droneTelemetryOnChange[key] });
    });
}

$('#videoLoading').show();
visualization_player.value = visualization_player.min;
visualization_player.style.width = '100%';
document.getElementById('INPUT').replaceWith(visualization_player);

var isPaused = false;
let pan_to_drone_only_once = true;

var allDroneInfoArray = [];
var new_drone_names = [];
var drone_names_and_ids = [];
var webSocketMessage = {};
var fireAlgorithmData = {};
webSocketMessage['weather_station'] = {};
webSocketMessage['drones'] = [];
frame_obj = {};

Object.keys(data.time_series_data).forEach(function (series) {
    data.time_series_data[new Date(series).getTime()] = data.time_series_data[series];
    delete data.time_series_data[series];
});

console.log(data);

const { MapboxLayer, LineLayer, COORDINATE_SYSTEM } = deck;
var map = main_map();

setTimeout(() => {
    create_drones(data.drone_available);
    dronesLines();

    live_visualization_player = setInterval(live, 10);
}, '5000');

function showVal(newVal) {
    document.getElementById('valBox').innerHTML = newVal;
}
function live() {
    if (!isPaused) {
        if (visualization_player.max > visualization_player.value) {
            check_timer();
            show_timer();
            currentVisualizationValue = visualization_player.value;
            if (data.time_series_data[visualization_player.value]) {
                replay_data = data.time_series_data[visualization_player.value];
                run_data(replay_data);
            }
        }
    }
}
function play_pause() {
    if (!isPaused) {
        isPaused = true;
        document.getElementById('main_controller').innerHTML = '<i class="fa-solid fa-play"></i>';
    } else {
        isPaused = false;
        document.getElementById('main_controller').innerHTML = '<i class="fa-solid fa-pause"></i>';
    }
}
function fast_forward() {
    visualization_player.value = parseInt(visualization_player.value) + 5000;
    show_timer();
    check_timer();
    edit_timer();
}
function rewind() {
    visualization_player.value = parseInt(visualization_player.value) - 5000;
    show_timer();
    check_timer();
    edit_timer();
}
function check_timer() {
    if (String(visualization_player.value).slice(-1) !== '0') {
        visualization_player.value = Number(visualization_player.value.replace(/.{0,2}$/, '') + '10');
    }
}
function show_timer() {
    visualization_player.value = parseInt(visualization_player.value) + 10;
    timer_current = visualization_player.value - visualization_player.min;
    max_timer = visualization_player.max - visualization_player.min;
    var result =
        format_timer(Math.floor(timer_current / (1000 * 60 * 60))) +
        ':' +
        format_timer(Math.floor(timer_current / (1000 * 60)) % 60) +
        ':' +
        format_timer(Math.floor(timer_current / 1000) % 60) +
        ' / ' +
        format_timer(Math.floor(max_timer / (1000 * 60 * 60))) +
        ':' +
        format_timer(Math.floor(max_timer / (1000 * 60)) % 60) +
        ':' +
        format_timer(Math.floor(max_timer / 1000) % 60);
    document.getElementById('timer').innerHTML = result;
}
function format_timer(currentMinutes) {
    if (currentMinutes.toString().length == 1) {
        currentMinutes = '0' + currentMinutes;
        return currentMinutes;
    }
    return currentMinutes;
}
function run_data(live_data) {
    Object.keys(live_data).forEach(function (key) {
        if (key.split(' ')[0] === 'telemetry') {
            allDronesArr = get_all_drone_info_array();
            for (let i = 0; i < allDroneInfoArray.length; i++) {
                if (allDroneInfoArray[i].droneAvailability === AVAILABLE) {
                    if (allDroneInfoArray[i]['dronePK'] == live_data[key]['drone_id']) {
                        updateDrone(live_data[key], allDroneInfoArray, i).then(function (updatedDroneArray) {
                            let currentDroneLocation = allDroneInfoArray[i].droneInfo.currentCoordinate;
                            let lon = currentDroneLocation[0];
                            let lat = currentDroneLocation[1];
                            if (pan_to_drone_only_once && lat !== 0 && lon !== 0) {
                                pan_to_drone_only_once = false;
                                map.flyTo({ center: currentDroneLocation, zoom: 20 });
                            }
                            update_all_drone_array(updatedDroneArray);
                        });
                    }
                }
            }
        } else if (key.split(' ')[0] === 'build_map_session') {
            console.log('build map session', live_data[key]);
        } else if (key.split(' ')[0] === 'build_map_image') {
            image_id = live_data[key].id.toString();
            if (!map.getSource(image_id)) {
                map.addSource(image_id, {
                    type: 'image',
                    url: dutils.urls.resolve('build_map_image_path', { image_path: live_data[key].path }),
                    coordinates: [
                        [live_data[key].top_right[0], live_data[key].top_right[1]],
                        [live_data[key].bottom_right[0], live_data[key].bottom_right[1]],
                        [live_data[key].bottom_left[0], live_data[key].bottom_left[1]],
                        [live_data[key].top_left[0], live_data[key].top_left[1]],
                    ],
                });
                if (!map.getLayer(image_id)) {
                    let allDrones = get_all_drone_info_array();
                    if (allDrones.length > 0) {
                        drone = allDrones[0].droneModel.id;
                    }
                    if (drone !== undefined) {
                        map.addLayer(
                            {
                                id: image_id,
                                type: 'raster',
                                source: image_id,
                                paint: { 'raster-opacity': 0.6 },
                            },
                            drone
                        );
                    } else {
                        map.addLayer({
                            id: image_id,
                            type: 'raster',
                            source: image_id,
                            paint: { 'raster-opacity': 0.6 },
                        });
                    }
                }
            } else {
                if (!map.getLayer(image_id)) {
                    let allDrones = get_all_drone_info_array();
                    if (allDrones.length > 0) {
                        drone = allDrones[0].droneModel.id;
                    }
                    if (drone !== undefined) {
                        map.addLayer(
                            {
                                id: image_id,
                                type: 'raster',
                                source: image_id,
                                paint: { 'raster-opacity': 0.6 },
                            },
                            drone
                        );
                    } else {
                        map.addLayer({
                            id: image_id,
                            type: 'raster',
                            source: image_id,
                            paint: { 'raster-opacity': 0.6 },
                        });
                    }
                }
            }
        } else if (key.split(' ')[0] === 'weather_station') {
            webSocketMessage['weather_station']['current_time'] = live_data[key].current_time;
            webSocketMessage['weather_station']['wind_direction'] = live_data[key].wind_direction;
            webSocketMessage['weather_station']['wind_speed'] = live_data[key].wind_speed;
            webSocketMessage['weather_station']['temperature'] = live_data[key].temperature;
            webSocketMessage['weather_station']['pressure'] = live_data[key].pressure;
            webSocketMessage['weather_station']['humidity'] = live_data[key].humidity;
            webSocketMessage['weather_station']['heading'] = live_data[key].heading;
        } else if (key.split(' ')[0] === 'detection_frame') {
            allDronesArr = get_all_drone_info_array();
            for (let i = 0; i < allDronesArr.length; i++) {
                if (live_data[key]['drone_id'] == allDronesArr[i].droneID) {
                    if (allDronesArr[i].droneDetectionStatus === DETECTION_DISCONNECTED) {
                        allDronesArr[i].droneDetectionStatus = DETECTION_CONNECTED;
                    }
                    if (webSocketMessage['drones'][allDronesArr[i].droneID] === undefined) {
                        webSocketMessage['drones'][allDronesArr[i].droneID] = {};
                    }
                    webSocketMessage['drones'][allDronesArr[i].droneID]['detection_frame'] = live_data[key];
                }
            }
        } else if (key.split(' ')[0] === 'detected_object') {
            allDronesArr = get_all_drone_info_array();
            for (let i = 0; i < allDronesArr.length; i++) {
                if (live_data[key]['drone_id'] == allDronesArr[i].droneID) {
                    if (allDronesArr[i].droneDetectionStatus === DETECTION_DISCONNECTED) {
                        allDronesArr[i].droneDetectionStatus = DETECTION_CONNECTED;
                    }
                    if (webSocketMessage['drones'][allDronesArr[i].droneID] === undefined) {
                        webSocketMessage['drones'][allDronesArr[i].droneID] = {};
                    }
                    if (
                        webSocketMessage['drones'][allDronesArr[i].droneID]['detection_object'] === undefined ||
                        new_time_data
                    ) {
                        new_time_data = false;
                        webSocketMessage['drones'][allDronesArr[i].droneID]['detection_object'] = [];
                    }

                    webSocketMessage['drones'][allDronesArr[i].droneID]['detection_object'].push(live_data[key]);
                }
            }
        } else if (key.split(' ')[0] === 'algorithm') {
            if (live_data[key]['algorithm_name'] === 'FIRE_PROPAGATION_ALGORITHM') {
                fireAlgorithmData['fields'] = live_data[key];
                RunFireSpreadOutput(fireAlgorithmData);
            } else {
                console.log('algorithm', live_data[key]);
            }
        } else if (key.split(' ')[0] === 'error') {
            create_popup_for_a_little(WARNING_ALERT, live_data[key]['message'], 4000);
        } else if (key.split(' ')[0] === 'user_input') {
            if (document.getElementById(live_data[key].element_name).checked !== live_data[key].active) {
                if (
                    list_user_action_menu.find((item) => {
                        return item === live_data[key].element_name;
                    })
                ) {
                    if (live_data[key].active === true) {
                        element_user_action_menu = live_data[key];
                        for (let i = 0; i < list_user_action_menu.length; i++) {
                            if (list_user_action_menu[i] !== element_user_action_menu.element_name) {
                                document.getElementById(list_user_action_menu[i]).classList.remove('active');
                                document.getElementById(list_user_action_menu[i] + '-side').style.display = 'none';
                            }
                        }
                        document.getElementById(live_data[key].element_name).classList.add('active');
                        document.getElementById(live_data[key].element_name + '-side').style.display = 'block';
                    } else if (live_data[key].active === false) {
                        element_user_action_menu = live_data[key];
                        for (let i = 0; i < list_user_action_menu.length; i++) {
                            if (list_user_action_menu[i] !== element_user_action_menu.element_name) {
                                document.getElementById(list_user_action_menu[i]).classList.remove('active');
                                document.getElementById(list_user_action_menu[i] + '-side').style.display = 'none';
                            }
                        }
                        document.getElementById(live_data[key].element_name).classList.remove('active');
                        document.getElementById(live_data[key].element_name + '-side').style.display = 'none';
                    }
                } else {
                    if (element_user_action_menu !== undefined) {
                        if (element_user_action_menu.active === true) {
                            for (let i = 0; i < list_user_action_menu.length; i++) {
                                if (list_user_action_menu[i] !== element_user_action_menu.element_name) {
                                    document.getElementById(list_user_action_menu[i]).classList.remove('active');
                                    document.getElementById(list_user_action_menu[i] + '-side').style.display = 'none';
                                }
                            }
                            document.getElementById(element_user_action_menu.element_name).classList.add('active');
                            document.getElementById(element_user_action_menu.element_name + '-side').style.display =
                                'block';
                        } else if (element_user_action_menu.active === false) {
                            document.getElementById(element_user_action_menu.element_name).classList.remove('active');
                            document.getElementById(element_user_action_menu.element_name + '-side').style.display =
                                'none';
                        }
                    }
                    if (live_data[key].value !== null) {
                        document.getElementById(live_data[key].element_name).value = live_data[key].value;
                        $('#' + live_data[key].element_name).trigger('oninput');
                    } else {
                        if (live_data[key].active === true) {
                            $('#' + live_data[key].element_name).bootstrapToggle('on');
                        } else if (live_data[key].active === false) {
                            $('#' + live_data[key].element_name).bootstrapToggle('off');
                        }
                    }
                }
            }
        } else if (key.split(' ')[0] === 'video_frame') {
            allDronesArr = get_all_drone_info_array();
            for (let i = 0; i < allDronesArr.length; i++) {
                if (live_data[key]['drone_id'] == allDronesArr[i]['dronePK']) {
                    if (webSocketMessage['drones'][allDronesArr[i].droneID] === undefined) {
                        webSocketMessage['drones'][allDronesArr[i].droneID] = {};
                    }
                    webSocketMessage['drones'][allDronesArr[i].droneID]['video_frame'] = live_data[key];
                }
            }

            // data.drone_available.forEach((drone) => {
            //     if (live_data[key].drone_id == drone['drone']) {
            //         if (webSocketMessage['drones'][live_data[key].drone_id] === undefined) {
            //             webSocketMessage['drones'][live_data[key].drone_id] = {};
            //         }
            //         webSocketMessage['drones'][live_data[key].drone_id]['live_frame'] = [];
            //         webSocketMessage['drones'][live_data[key].drone_id]['live_frame'].push(live_data[key]['frame']);
            //     }
            // console.log(webSocketMessage);
            // webSocketMessage['drones'].forEach((messageDrone) => {

            //     console.log(messageDrone);
            // });

            // if (drone['drone']===webSocketMessage[]) {

            // }
            // live_data[key];
            // webSocketMessage['drones'][drone['drone']]['live_frame'] = '';
            // console.log(drone);
            // console.log(webSocketMessage);
            // }
            // );
        } else {
            console.log('Unused data', key, live_data[key]);
        }
    });
}
function reverse_data(live_data) {
    new_time_data = true;
    Object.keys(live_data).forEach(function (key) {
        if (key.split(' ')[0] === 'build_map_image') {
            image_id = live_data[key].id.toString();
            if (map.getSource(image_id)) {
                if (map.getLayer(image_id)) map.removeLayer(image_id);
            }
        } else if (key.split(' ')[0] === 'user_input') {
            if (
                list_user_action_menu.find((item) => {
                    return item === live_data[key].element_name;
                }) !== undefined &&
                list_user_action_menu.find((item) => {
                    return item === live_data[key].element_name;
                })
            ) {
                if (live_data[key].active === true) {
                    element_user_action_menu = live_data[key];
                    for (let i = 0; i < list_user_action_menu.length; i++) {
                        if (list_user_action_menu[i] !== element_user_action_menu.element_name) {
                            document.getElementById(list_user_action_menu[i]).classList.remove('active');
                            document.getElementById(list_user_action_menu[i] + '-side').style.display = 'none';
                        }
                    }
                    document.getElementById(live_data[key].element_name).classList.add('active');
                    document.getElementById(live_data[key].element_name + '-side').style.display = 'block';
                } else if (live_data[key].active === false) {
                    document.getElementById(live_data[key].element_name).classList.add('active');
                    document.getElementById(live_data[key].element_name + '-side').style.display = 'block';
                }
            } else {
                if (live_data[key].value !== null) {
                    document.getElementById(live_data[key].element_name).value = live_data[key].value;
                    $('#' + live_data[key].element_name).trigger('oninput');
                } else {
                    if (live_data[key].active === false) {
                        $('#' + live_data[key].element_name).bootstrapToggle('on');
                    } else if (live_data[key].active === true) {
                        $('#' + live_data[key].element_name).bootstrapToggle('off');
                    }
                }
            }
        } else if (key.split(' ')[0] === 'algorithm') {
            let geojsonSourceID = 'geojson_fire_data_' + String(live_data[key].id);
            if (map.getSource(geojsonSourceID)) {
                if (map.getLayer(geojsonSourceID)) map.removeLayer(geojsonSourceID);
                map.removeLayer(geojsonSourceID + '_outline');
                map.removeSource(geojsonSourceID);
            }
        }
    });
}
let temp_drones = [];
function create_drones(drone_list) {
    drone_list.forEach(function (drone) {
        new_drone_names.push(drone.drone_name);
        drone_names_and_ids.push({ drone_name: drone.drone_name, pk: drone.drone });
    });
    allDroneInfoArray = add_new_drones_to_array(new_drone_names, drone_names_and_ids);
    remove_no_drones_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS);
    add_list_items_to_trajectories(new_drone_names);
    add_list_items_to_uav_missions(new_drone_names);
    add_list_items_to_video_feeds(new_drone_names);
    add_list_items_to_det_video_feeds(new_drone_names);
    add_list_items_to_map_tools(new_drone_names);
    add_list_items_to_enable_detections_list(new_drone_names);
    create_video_elements(new_drone_names);
    create_det_video_elements(new_drone_names);

    temp_drones = new_drone_names;

    function check_and_create_drone_models() {
        allDronesArr = get_all_drone_info_array();
        if (allDronesArr.length === undefined) {
            setTimeout(check_and_create_drone_models, 500); //console.log('works')
        } else {
            for (let i = 0; i < drone_names_and_ids.length; i++) {
                for (let j = 0; j < data.mission_data.length; j++) {
                    if (data.mission_data[j].dronePK === drone_names_and_ids[i].pk) {
                        let waypoints = [];
                        data.mission_data[j].mission_points.forEach(function (point, index) {
                            point = [point['point'][0], point['point'][1]];
                            waypoints.push(point);
                        });
                        if (data.mission_data[j].mission_type === 'SEARCH_AND_RESCUE_MISSION') {
                            addLineLayerInIndex(waypoints, drone_names_and_ids[i].pk);
                            let point_of_waypoints = [];
                            point_of_waypoints.push(waypoints[0]);
                            point_of_waypoints.push(waypoints[8]);
                            point_of_waypoints.push(waypoints[81]);
                            point_of_waypoints.push(waypoints[waypoints.length - 1]);
                            place_waypoints_on_map(point_of_waypoints, drone_names_and_ids[i].pk);
                        } else {
                            place_waypoints_on_map(waypoints, data.mission_data[j].dronePK);
                        }
                    }
                }
                $('#videoLoading').hide();
            }
            new_drone_names.forEach(function (name, index) {
                webSocketMessage['drones'][index] = [];
                webSocketMessage['drones'][index]['drone_name'] = name;
                webSocketMessage['drones'][index]['detected_objects'] = '';
                webSocketMessage['drones'][index]['detection'] = DETECTION_DISCONNECTED;
            });
        }
    }
    check_and_create_drone_models();
}

function dronesLines() {
    allDronesArr = get_all_drone_info_array();
    Object.keys(data.time_series_data).forEach(function (time) {
        Object.keys(data.time_series_data[time]).forEach(function (field) {
            if (field.split(' ')[0] === 'telemetry') {
                for (let i = 0; i < allDronesArr.length; i++) {
                    if (allDronesArr[i].dronePK === data.time_series_data[time][field]['drone_id']) {
                        var currentLineData;
                        allDronesArr[i].droneInfo.currentCoordinate = [
                            data.time_series_data[time][field]['lon'],
                            data.time_series_data[time][field]['lat'],
                            data.time_series_data[time][field]['alt'],
                        ];
                        currentLineData = {
                            source: allDronesArr[i].droneInfo.previousCoordinate,
                            dest: allDronesArr[i].droneInfo.currentCoordinate,
                            color: [23, 184, 190],
                        };

                        currentDrone = updateLineLayer(allDronesArr[i], currentLineData);

                        allDronesArr[i].droneInfo.previousCoordinate = allDronesArr[i].droneInfo.currentCoordinate;
                    }
                    update_all_drone_array([allDronesArr[i]]);
                }
            }
        });
    });
}
let droneObj = {};
function updateDrone(telemetry, allDroneInfoArray, iteration) {
    let updatedDroneArray = allDroneInfoArray;
    let currentDrone = allDroneInfoArray[iteration];

    currentDrone.previousdroneMissionState = currentDrone.droneMissionState;
    currentDrone.droneMissionState = telemetry.drone_state;
    currentDrone.droneInfo.currentBatteryLevel = telemetry.battery_percentage;
    var selectElement = document.getElementById('clickSelect');
    var goElement = document.getElementById('clickGo');
    var cancelElement = document.getElementById('clickCancel');
    var pauseElement = document.getElementById('clickPause');
    let clickGo = $('#clickGo');
    try {
        droneObj.time = telemetry.received_at.replace('T', ' ').split('.')[0];
        droneObj.drone_state = telemetry.drone_state;
        droneObj.battery_percentage = telemetry.battery_percentage;
        droneObj.gps_signal = telemetry.gps_signal;
        droneObj.satellites = telemetry.satellites;
        droneObj.latitude = telemetry.lat;
        droneObj.longitude = telemetry.lon;
        droneObj.altitude = telemetry.alt;
        droneObj.velocity = telemetry.velocity;
        droneObj.heading = telemetry.heading;
        droneObj.gimbal_angle = telemetry.gimbal_angle;
        displayDroneData(droneObj, currentDrone.droneID);
        droneObj = {};
    } catch (error) {}
    if (currentDrone.droneObject !== undefined && currentDrone.droneObject.selected) {
        updateMissionButtonStates(currentDrone.droneObject);
        updateBuildMapButtonStates(currentDrone, true);
    }
    updatedDroneArray = updateLocalDroneAttributes(currentDrone, iteration, allDroneInfoArray, telemetry); //Perform the actual update with the new locations here
    updatedDroneArray[iteration] = currentDrone;
    return new Promise((resolve, reject) => resolve(updatedDroneArray));
}

function updateLocalDroneAttributes(currentDrone, iteration, allDronesArray, telemetry) {
    if (currentDrone.droneInfo.currentCoordinate !== null) {
        currentDrone.droneInfo.previousCoordinate = currentDrone.droneInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    currentDrone.droneInfo.currentCoordinate = [telemetry.lon, telemetry.lat, telemetry.alt];
    currentDrone.droneInfo.heading = telemetry.heading;
    currentDrone.droneInfo.velocity = telemetry.velocity;
    currentDrone.droneInfo.altitude = telemetry.alt;

    if (currentDrone.droneObject !== undefined) {
        if (temp_drones !== []) {
            addTooltipOnDrones(temp_drones);
            temp_drones = [];
        }
        currentDrone.droneObject.setCoords([
            currentDrone.droneInfo.currentCoordinate[0],
            currentDrone.droneInfo.currentCoordinate[1],
            currentDrone.droneInfo.currentCoordinate[2],
        ]);
        currentDrone.droneObject.setRotation(-telemetry.heading);
    }

    allDronesArray[iteration] = currentDrone;

    return allDronesArray;
}

function main_map() {
    const { MapboxLayer, LineLayer, COORDINATE_SYSTEM } = deck;
    let currentStyle = sessionStorage.getItem('currentStyle');
    let currentStyleRadioBtn = sessionStorage.getItem('currentStyleRadioBtn');
    let lastDroneLocation = JSON.parse(sessionStorage.getItem('lastDroneLocation'));
    if (currentStyle === null) {
        //If it's the first time loading the platform, this variable is null
        currentStyle = 'https://api.maptiler.com/maps/<>/style.json?key=blpQOMdNw0JJIq07I9Ln'.replace(
            '<>',
            BASIC_STYLE_ID
        );
    }

    if (currentStyleRadioBtn === null) {
        currentStyleRadioBtn = 'basic-v2';
    }

    if (lastDroneLocation === null || typeof lastDroneLocation === 'undefined' || lastDroneLocation.length === 0) {
        lastDroneLocation = DEFAULT_MAP_CENTER;
    }

    $('#' + currentStyleRadioBtn).prop('checked', true);

    if (!USE_ONLINE_MAPS) {
        currentStyle = {
            version: 8,
            sources: {
                osm: {
                    type: 'raster',
                    tiles: ['http://0.0.0.0:8081/tile/{z}/{x}/{y}.png'],
                    tileSize: 256,
                },
            },
            layers: [
                {
                    id: 'osm',
                    type: 'raster',
                    source: 'osm',
                },
            ],
        };
    }
    maplibregl.accessToken =
        'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg';
    var map = new maplibregl.Map({
        container: 'map',
        style: currentStyle,
        zoom: 7,
        center: [33.0028145, 35.0725358],
    });
    if (USE_ONLINE_MAPS) {
        /*This element is a searchbox that allows user to search locations*/
        var geocoder = new MapboxGeocoder({
            accessToken:
                'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg',
            placeholder: "Try 'Cyprus' or '33,35' or a drone's ID",
            mapboxgl: maplibregl,
            marker: true,
            localGeocoder: coordinatesGeocoder,
        });

        document.getElementById('geocoder').appendChild(geocoder.onAdd(map));
    }
    var buildingPopup;

    map.addControl(new maplibregl.NavigationControl()); // Add zoom and rotation controls to the map.

    // Add the Screen shot feature for MapBox
    map.addControl(
        new MaplibreExportControl({
            PageSize: Size.A3,
            PageOrientation: PageOrientation.Portrait,
            Format: Format.PNG,
            DPI: DPI[96],
            Crosshair: true,
            PrintableArea: true,
            Local: 'en',
        }),
        'top-right'
    );

    //Adding Threebox library to mapbox to add built-in mouseover/mouseout and click behaviors
    window.tb = new Threebox(map, map.getCanvas().getContext('webgl'), {
        defaultLights: true,
        enableSelectingFeatures: true, //change this to false to disable fill-extrusion features selection
        enableSelectingObjects: true, //change this to false to disable 3D objects selection
        enableDraggingObjects: true, //change this to false to disable 3D objects drag & move once selected
        enableRotatingObjects: true, //change this to false to disable 3D objects rotation once selected
        enableTooltips: false, // change this to false to disable default tooltips on fill-extrusion and 3D models
        outlinePass: false,
        multiLayer: true,
    });
    // tb.renderer.outputEncoding = THREE.sRGBEncoding
    tb.renderer.outputEncoding = THREE.LinearEncoding;
    map.on('load', function () {
        add_available_detector_types_on_panel();
        // map.addLayer(threeDbuildingLayer);

        map.on('SelectedFeatureChange', onSelectedFeatureChange);

        var drone_iter_index = 0;
        var previousDroneNumber = 0;
        var previous_drone_ids = [];
        var currentDroneNumber = 0;
        let iter_iter = 0;
        var allDroneInfoArray = [];
        // add_list_items_to_detection_types()
    });
    return map;
}
let weatherInterval;
function weatherClickInfoBox(toggleButtonID) {
    let clickInfoBox = $('#weatherDataBox');
    clickInfoBox.toggle();
    let pressed = $(`#${toggleButtonID}`).is(':checked');

    if (pressed) {
        clickInfoBox.draggable();
        weatherInterval = setInterval(function () {
            presentWeatherData();
        }, WEATHER_UPDATE_INTERVAL);
        function presentWeatherData() {
            let weatherObj = {};
            weatherObj.time = webSocketMessage['weather_station']['current_time'];
            weatherObj.windDir = parseFloat(webSocketMessage['weather_station']['wind_direction']).toFixed(2);
            weatherObj.windSpeed = parseFloat(webSocketMessage['weather_station']['wind_speed']).toFixed(3);
            weatherObj.temp = parseFloat(webSocketMessage['weather_station']['temperature']).toFixed(2);
            weatherObj.pressure = parseFloat(webSocketMessage['weather_station']['pressure']).toFixed(2);
            weatherObj.humidity = parseFloat(webSocketMessage['weather_station']['humidity']).toFixed(2);
            weatherObj.heading = parseFloat(webSocketMessage['weather_station']['heading']).toFixed(3);

            displayWeatherData(weatherObj);
            weatherObj = {};
        }
    } else {
        clearInterval(weatherInterval);
    }
}

function displayWeatherData(weatherObj) {
    let timeDisplay = document.getElementById('time1');
    let windDirDisplay = document.getElementById('wDir');
    let wSpeedDisplay = document.getElementById('wSpeed');
    let tempDisplay = document.getElementById('temp');
    let pressureDisplay = document.getElementById('press');
    let humidityDisplay = document.getElementById('humidity');
    let headingDisplay = document.getElementById('heading');
    if (weatherObj.time !== undefined) {
        weatherObj.time = weatherObj.time.replace('T', ' ').split('.')[0];
        timeDisplay.textContent = weatherObj.time;
        windDirDisplay.textContent = weatherObj.windDir + ' \u00B0';
        wSpeedDisplay.textContent = weatherObj.windSpeed + ' m/s';
        tempDisplay.textContent = weatherObj.temp + ' \u2103';
        pressureDisplay.textContent = weatherObj.pressure + ' Pa';
        humidityDisplay.innerHTML = weatherObj.humidity + ' g.m<sup>-</sup><sup>3</sup>';
        headingDisplay.textContent = weatherObj.heading + ' \u00B0';
    }
}
function onSelectedFeatureChange(e) {
    let feature = e.detail;
    if (feature && feature.state && feature.state.select) {
        if (buildingPopup) {
            buildingPopup.remove();
        }

        let center = [];
        let coords = tb.getFeatureCenter(feature, null, 0);

        center.push([coords[0], coords[1]]);

        //Creating a mapbox popup to show on buildings
        buildingPopup = new maplibregl.Popup({ offset: 0 })
            .setLngLat(center[0].slice())
            .setHTML('<strong>' + ('Height: ' + feature.properties.height + 'm') + '</strong >')
            .addTo(map);

        let geoJson = {
            geometry: feature.geometry,
            type: 'Feature',
            properties: feature.properties,
        };
        console.log(JSON.stringify(geoJson, null, 2));
    }
}

function addLineLayerInIndex(pathObj, droneID) {
    let path = pathObj;
    path.pop();
    if (map.getLayer('path_' + droneID)) {
        map.removeLayer('path_' + droneID);
    }
    if (map.getSource('path_' + droneID)) {
        map.removeSource('path_' + droneID);
    }
    map.addSource('path_' + droneID, {
        type: 'geojson',
        data: {
            type: 'Feature',
            properties: {},
            geometry: {
                type: 'LineString',
                coordinates: path,
            },
        },
    });
    let allDrones = get_all_drone_info_array();
    if (allDrones.length > 0) {
        map.addLayer(
            {
                id: 'path_' + droneID,
                type: 'line',
                source: 'path_' + droneID,
                layout: {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                paint: {
                    'line-color': getBasicColor(get_drone_index(droneID)),
                    'line-width': 8,
                },
            },
            allDrones[0].droneModel.id
        );
    } else {
        map.addLayer({
            id: 'path_' + droneID,
            type: 'line',
            source: 'path_' + droneID,
            layout: {
                'line-join': 'round',
                'line-cap': 'round',
            },
            paint: {
                'line-color': getBasicColor(get_drone_index(droneID)),
                'line-width': 8,
            },
        });
    }
    map.flyTo({ center: [path[0][0], path[0][1]] });
}
function allModelsLoaded(allDrones) {
    if (allDrones[0] === 'undefined') {
        return false;
    }
    for (let i = 0; i < allDrones.length; i++) {
        let obj = allDrones[i].droneObject;
        if (isObjEmpty(obj)) {
            return false;
        }
    }
    return true;
}
function clear_timer(timer) {
    if (timer !== undefined) {
        clearInterval(timer);
    }
}
