
let currentMapStyle = '';
let previousMapStyle = currentMapStyle;
let first_time_flag = false;
let first_time = true;

var previousDroneNumber = 0;
var previousDeviceNumber = 0;
var previousBaloraNumber = 0;

var altitudeSafetyDistance = 10;

const wsInterval = 200; // in milliseconds
// const wsAddress = 'ws://' + window.location.host + '/' + dutils.urls.resolve('ws_platform'); // django ws
const wsAddress = 'ws://' + window.location.hostname + ':' + WS_PORT; // go ws
let g_websocketMessage;
let g_websocketCounter = 0;
let g_websocket = initWebsocket(wsAddress);


map.on('load', function () {

    // side-panel initialization actions
    add_no_drones_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DRONES);
    add_no_devices_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
    add_no_baloras_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);    

    setInterval(sendWebsocketMessage, wsInterval); // init websocket data timer

    map.on('SelectedFeatureChange', onSelectedFeatureChange);
    createDisasterEpicenterMarker(map);

    //map.addSource(  'terrainSource' , { type: 'raster-dem',url: 'https://api.maptiler.com/tiles/terrain-rgb-v2/tiles.json?key=blpQOMdNw0JJIq07I9Ln', } );
    //map.addSource(  'hillshadeSource' , { type: 'raster-dem', url: 'https://api.maptiler.com/tiles/terrain-rgb-v2/tiles.json?key=blpQOMdNw0JJIq07I9Ln', } );
    // let drones_url = dutils.urls.resolve('drones', { operation_name: CURRENT_OP });
    // map.addLayer(threeDbuildingLayer);
});








// establish websocket connection and set up event listeners
function initWebsocket(_address) {
    console.log(_address);
    let ws = new WebSocket(_address + '?token=' + encodeURIComponent(TOKEN));
    ws.addEventListener('open', function (event) {
        console.log('WebSocket connection established.');
    });
    ws.addEventListener('close', function (event) {
        console.log('Socket is closed. Reconnect will be attempted in 1 second.');
        setTimeout(function () {
            g_websocket = initWebsocket(wsAddress);
        }, 1000);
    });
    ws.addEventListener('error', function (event) {
        console.error('Socket encountered error: Closing connection');
    });
    ws.addEventListener('message', function (e) {
        g_websocketMessage = JSON.parse(e.data);
        handleIncomingWebsocketMessage(g_websocketMessage);
    });
    return ws;
}


// send a message to the websocket server
function sendWebsocketMessage() {
    if (g_websocket.readyState == 1) {
        var getAllData = g_websocketCounter % 3
        const jsonData = {
            operation_id: parseInt(OPERATION_ID),
            operation_name: CURRENT_OP,
            get_all_data: getAllData,
        };
        g_websocket.send(JSON.stringify(jsonData));
        g_websocketCounter = (g_websocketCounter + 1) % 6;  // increment g_websocketCounter
    }
}


// triggered when a websocket message is received
function handleIncomingWebsocketMessage(_wsMessage) {
    // console.log(JSON.stringify(_wsMessage));
    if ('devices' in _wsMessage) {  // if the ws message includes devices then do the full update procedure
        if (_wsMessage['error_msg'].length !== 0) {
            create_popup_for_a_little(WARNING_ALERT, _wsMessage['error_msg'], 10000);
        }
        droneUpdate(_wsMessage['drones']);
        updateDroneFOVPolygons(_wsMessage['drones']);
        deviceUpdate(_wsMessage['devices']);
        baloraUpdate(_wsMessage['baloras']);

        checkForMissionPointSelection();
    }
    updateStreamFrames(_wsMessage['drones']);   // update the stream frames
}


const fovLineLayers = {};

// update the drone's field of view polygons
function updateDroneFOVPolygons(drones) {
    drones.forEach(function (drone) {
        let droneName = drone['drone_name'];

        let localDrone = get_drone_object(droneName);

        if (typeof(localDrone.droneObject) === "undefined") {
            console.log("no local drone object");
            return;
        }

        // hide layers if drone not selected
        if (!localDrone.droneObject.selected) {
            if (map.getLayer("fov-polygon-layer-" + droneName)) {
                map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'none');
                map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'none');
                map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'none');
            }
            return;
        }

        // make layers visible
        if (map.getLayer("fov-polygon-layer-" + droneName)) {
            map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'visible');
            map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'visible');
            map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'visible');
        }        

        var fov_coordinates = JSON.parse(drone["telemetry"]["fov_coordinates"]);
        var fov_points = [];
        if (fov_coordinates == null) {
            return;
        }
        fov_coordinates.forEach(function (point, i) {
            fov_points.push([point[1], point[0]]);
        });

        var newData_fov = {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [fov_points.concat([fov_points[0]])] // Ensure the polygon is closed
            }
        };

        // Check if the source exists before setting data
        if (map.getSource("fov-polygon-" + droneName)) {
            map.getSource("fov-polygon-" + droneName).setData(newData_fov);
            // calculate the center of the polygon
            let sumLat = 0, sumLon = 0;
            for (let i = 0; i < fov_coordinates.length; i++) {
                sumLat += fov_coordinates[i][0];
                sumLon += fov_coordinates[i][1];
            }
            let centerLat = sumLat / fov_coordinates.length;
            let centerLon = sumLon / fov_coordinates.length;
            // update the view direction line
            fovLineLayers[droneName].setProps({
                data: [
                    {
                        source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
                        dest: [fov_coordinates[0][1], fov_coordinates[0][0], fov_coordinates[0][2]]
                    },
                    {
                        source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
                        dest: [fov_coordinates[1][1], fov_coordinates[1][0], fov_coordinates[1][2]]
                    },
                    {
                        source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
                        dest: [fov_coordinates[2][1], fov_coordinates[2][0], fov_coordinates[2][2]]
                    },
                    {
                        source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
                        dest: [fov_coordinates[3][1], fov_coordinates[3][0], fov_coordinates[3][2]]
                    },
                ]
            });
            
                     
        } else {

            var test = new MapboxLayer({
                id: droneName + '-to-fov-line',
                type: LineLayer,
                data: [],
                fp64: false,
                widthScale: 0.1,
                getWidth: 15, //Change getWidth and widthScale to fine-tune the line width
                opacity: 0.2,
                widthUnit: 'meters',
                // getStrokeWidth: 6,
                getSourcePosition: (d) => d.source,
                getTargetPosition: (d) => d.dest,
                getColor: [250, 50, 50],
            });
            map.addLayer(test);
            fovLineLayers[droneName] = test;


            // If the source doesn't exist, add it to the map
            map.addSource("fov-polygon-" + droneName, {
                "type": "geojson",
                "data": newData_fov
            });

            // Add a layer for the polygon
            map.addLayer({
                "id": "fov-polygon-layer-" + droneName,
                "type": "fill",
                "source": "fov-polygon-" + droneName,
                "layout": {},
                "paint": {
                    "fill-color": "#888888",
                    "fill-opacity": 0.2
                }
            });
            // add an outline layer
            map.addLayer({
                "id": "fov-polygon-outline-" + droneName,
                "type": "line",
                "source": "fov-polygon-" + droneName,
                "paint": {
                    "line-color": "#ff5555", // Set the outline color here
                    "line-width": 2,
                    "line-opacity": 0.6
                }
            });
        }
    });
} // end of updateDroneFOVPolygons()



// change the src attribute of the stream image if the stream is visible
function updateStreamFrames(drones) {
    drones.forEach(function (drone) {
        let droneName = drone['drone_name'];
        changeStreamImageAndStatus(droneName, drone['video_frame_url'], "live-stream");
        changeStreamImageAndStatus(droneName, drone['detected_frame_url'], "detection-stream");
    });
}


function changeStreamImageAndStatus(droneName, framePath, elementIdPrefix) {
    // console.log(framePath);
    try {
        let wrapper_div = document.getElementById(elementIdPrefix + '-wrapper-div-' + droneName);
        if ($(wrapper_div).is(":visible")) {
            let status_div = document.getElementById(elementIdPrefix + '-status-div-' + droneName);
            let img_element = document.getElementById(elementIdPrefix + '-img-' + droneName);

            if (framePath.startsWith("/")) {
                img_element.src = framePath;
                status_div.style.color = 'lawngreen';
                status_div.innerHTML = 'Connected';                
                // if (img_element.src != "http://" + window.location.host + framePath) {
                //     img_element.src = framePath;
                //     status_div.style.color = 'lawngreen';
                //     status_div.innerHTML = 'Connected';
                // }
                // else {
                //     console.log("Same image path for " + droneName);
                // }
            }
            else {
                img_element.src = VIDEO_COVER_PHOTO_DRONE;
                status_div.style.color = 'red';
                status_div.innerHTML = 'Disconnected';
            }
        }
    }
    catch (error) {
        console.log("Video elements for " + droneName + " not yet initialized");
    }
}







var previous_drone_ids = [];
var allDroneInfoArray = [];


var canAddTooltip = false;
var temp_new_drone_ids = [];
var tooltipAddedDrone = false;
let canProceed = false;
let pan_to_drone_only_once = true;

/**
 * Starts the process of updating the drone's attributes and/or layers (e.g the drone's position, the drone's line layer etc.)
 * */
function droneUpdate(_wsDrones) {

    let drones_core_info = _wsDrones.map((drone) => ({ drone_name: drone.drone_name, drone_pk: drone.id, drone_type: drone.type }));

    let drone_names = _wsDrones.map(function (drone) {
        return drone.drone_name;
    });

    var currentDroneNumber = drone_names.length;
    
    // no drones currently connected
    if (currentDroneNumber === 0) {
        if (previousDroneNumber > 0) {
            document.getElementById("drones-menu").style.display = "none";  // hide sidebar section for drones
            handleDisconnectedDrones(previous_drone_ids);                   // remove drones that were disconnected
            add_no_drones_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DRONES);
        }
        previousDroneNumber = 0;
        return;
    }
    
    document.getElementById("connected-drones").innerHTML = currentDroneNumber; // show number of drones on sidebar
    document.getElementById("drones-menu").style.display = "block";             // show sidebar section for drones

    //  drones connected
    if (currentDroneNumber > previousDroneNumber) {
        
        var new_drone_names = [];
        tooltipAddedDrone = false;
        canAddTooltip = false;
        canProceed = false;
        var new_index = 0;

        if (currentDroneNumber - previousDroneNumber === 0 || previousDroneNumber === 0) {
            //If the number of previous drones was zero (e.g at the beginning), then just overwrite
            new_drone_names = drone_names;
        } //Otherwise, get the ids of newly connected drones
        else {
            new_index = previousDroneNumber;
            new_drone_names = find_difference_on_two_arrays(drone_names, previous_drone_ids);
        }

        previousDroneNumber = currentDroneNumber;
        temp_new_drone_ids = new_drone_names;

        allDroneInfoArray = add_new_drones_to_array(new_drone_names, drones_core_info); //It's time to add the new drones to the drones array

        remove_no_drones_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DRONES);
        create_video_elements(new_drone_names);
        create_det_video_elements(new_drone_names);
        addNewDronesToSidePanel(new_drone_names);

        _wsDrones.forEach(function (drone) {
            for (let index = 0; index < new_drone_names.length; index++) {
                if (drone['drone_name'] == new_drone_names[index] && drone['weather_station_available'] == true) {
                    add_list_items_to_map_tools_weather([new_drone_names[index]]);
                }
                if (drone['drone_name'] == new_drone_names[index] && drone['lidar_available'] == true) {
                    add_list_items_to_uav_missions_lidar([new_drone_names[index]]);
                }
                if (drone['drone_name'] == new_drone_names[index] && drone['water_sampler_available'] == true) {
                    add_list_items_to_uav_missions_water_sampler([new_drone_names[index]]);
                }
                // add_list_items_to_uav_missions_water_sampler([new_drone_names[index]]);

            }
        });
        // add_list_items_to_enable_detections_list(new_drone_names);


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

        for (let i = 0; i < new_drone_names.length; i++) {
            let drone_index = get_drone_index(new_drone_names[i]);
            _wsDrones.forEach(function (drone) {
                if (drone['drone_name'] == new_drone_names[i]) {
                    if (drone['telemetry'] === null) {
                        return;
                    }
                    telemetry = drone['telemetry'];
                    if (telemetry.drone_state === IN_MISSION || telemetry.drone_state === PAUSED_MISSION) {
                        $.getJSON(allDroneInfoArray[drone_index].missionPointsURL).then(function (points) {
                            let waypoints = [];
                            points.forEach(function (point, index) {
                                point = [point['point']['longitude'], point['point']['latitude']];
                                waypoints[index] = point;
                            });
                            $.getJSON(allDroneInfoArray[drone_index].missionURL).then(function (mission) {
                                if (mission === 'SEARCH_AND_RESCUE_MISSION') {
                                    addLineLayerInIndex(waypoints, allDroneInfoArray[drone_index].droneID); // draw the mission path (?)
                                    let point_of_waypoints = [];
                                    point_of_waypoints.push(waypoints[0]);
                                    point_of_waypoints.push(waypoints[8]);
                                    point_of_waypoints.push(waypoints[81]);
                                    point_of_waypoints.push(waypoints[waypoints.length - 1]);
                                    place_waypoints_on_map(point_of_waypoints, drones_core_info[i].drone_pk); // draw a box around the mission path
                                } else {
                                    place_waypoints_on_map(waypoints, allDroneInfoArray[drone_index].droneID); // draw the mission path
                                }
                            });
                        });
                    }
                }
            });
        }

        first_time_flag = true;
        previousMapStyle = currentMapStyle;
    } else if (currentDroneNumber < previousDroneNumber) {  // drones have been disconnected/removed
        let removed_drones_ids = find_difference_on_two_arrays(drone_names, previous_drone_ids);
        handleDisconnectedDrones(removed_drones_ids);                   // remove drones that were disconnected
    }

    previous_drone_ids = drone_names;
    previousDroneNumber = currentDroneNumber;

    if (allDroneModelsLoaded(allDroneInfoArray)) {
        //Before doing the actual update of the drone's attributes, make sure that all 3d drone models are loaded, otherwise we get an error
        canProceed = true;
        canAddTooltip = true;
    }

    if (canProceed) {
        for (let i = 0; i < allDroneInfoArray.length; i++) {
            if (allDroneInfoArray[i].droneAvailability === AVAILABLE) {
                _wsDrones.forEach(function (drone) {
                    if (drone['drone_name'] == allDroneInfoArray[i].droneID) {
                        if (drone['telemetry'] === null) {
                            return;
                        }
                        drone['telemetry'].crps_requested = drone.requested_collaboration;
                        drone['telemetry'].crps_responding = drone.responding_to_collaboration;
                        // console.log(drone["telemetry"]);
                        updateDroneWS(drone['telemetry'], allDroneInfoArray, i).then(function () {
                            if (
                                pan_to_drone_only_once &&
                                allDroneInfoArray[i].droneInfo.currentCoordinate[1] !== 0 &&
                                allDroneInfoArray[i].droneInfo.currentCoordinate[0] !== 0
                            ) {
                                pan_to_drone_only_once = false;
                                map.flyTo({ center: allDroneInfoArray[i].droneInfo.currentCoordinate, zoom: 20 });
                                remove_line_layers_from_map_drone(allDroneInfoArray);
                            }
                            update_all_drone_array(allDroneInfoArray);
                        });
                        checkIfDroneBuildMapIsActivatedOrDeactivated(allDroneInfoArray[i], drone.build_map_activated)
                    }
                });
            }
        }
    }
    if (canAddTooltip && !tooltipAddedDrone) {
        //Add a tooltip to the 3d drone models only if they were all successfully loaded otherwise we get an error
        tooltipAddedDrone = true;
        addTooltipOnDrones(temp_new_drone_ids);
    }

    checkForDroneWarnings(allDroneInfoArray);
}




function handleDisconnectedDrones(_droneNames) {
    update_drones_local_availability(_droneNames, NOT_AVAILABLE);
    removeDronesFromSidePanel(_droneNames);
    remove_video_element(_droneNames);
    remove_det_video_elements(_droneNames);
    hide_drones_from_map(_droneNames);
}



function checkForDroneWarnings(drones) {
    var altitudes = [];
    for (let i = 0; i < drones.length; i++) {
        altitudes.push(drones[i].droneInfo.altitude);
    }
    var warningsDiv = document.getElementById("drone-warnings");

    if (hasDifferenceWithinRange(altitudes, altitudeSafetyDistance - 1)) {
        warningsDiv.innerHTML = "CAUTION: Two or more drones are flying at similar altitudes (less than " + altitudeSafetyDistance + "m).";
        warningsDiv.style.display = "inline-block";
    }
    else {
        warningsDiv.innerHTML = "";
        warningsDiv.style.display = "none";
    }
}




// show a message if the mission point selection is in progress
function checkForMissionPointSelection() {

    // var pointSelectionMessageDiv = document.getElementById("point-selection-message");
    var drones = get_selected_drones();

    if (selectionInProgress(drones)) {
        $("#selected-drones-list").html("")
        drones.forEach(function (drone) {
            $("#selected-drones-list").append("<div> - "+drone.droneID+"</div>");
        });
        $("#point-selection-message").show();
        $("#selected-drones-list").show();
        $("#sidebar-overlay").show();
    }
    else {
        // pointSelectionMessageDiv.style.display = "none";
        $("#point-selection-message").hide();
        $("#selected-drones-list").hide();
        $("#sidebar-overlay").hide();
    }
}









var device_iter_index = 0;
var tooltipAddedDevice = false;
var temp_new_device_ids = [];
pan_to_device_only_once = true;
/**
 * Starts the process of updating the device's attributes and/or layers
 * */
function deviceUpdate(_wsDevices) {

    let device_names_and_ids = _wsDevices.map((device) => ({ device_name: device.name, device_pk: device.id }));

    let device_names = device_names_and_ids.map(function (device) {
        return device.device_name;
    });
    var currentDeviceNumber = device_names.length;


    // no devices currently connected
    if (currentDeviceNumber === 0) {
        if (previousDeviceNumber > 0) {
            document.getElementById("devices-menu").style.display = "none";  // hide sidebar section for devices
            removeDevicesFromSidePanel(previous_device_ids);                 // remove devices that were disconnected
            add_no_devices_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
            hide_devices_from_map(previous_device_ids);
        }
        previousDeviceNumber = 0;
        return;
    }

    document.getElementById("connected-devices").innerHTML = currentDeviceNumber; // show number of devices on sidebar
    document.getElementById("devices-menu").style.display = "block";             // show sidebar section for devices


    // Add connected devices
    if (currentDeviceNumber > previousDeviceNumber) {
        var new_device_names = [];
        tooltipAddedDevice = false;
        canAddTooltip = false;
        canProceed = false;

        var new_index = 0;

        if (currentDeviceNumber - previousDeviceNumber === 0 || previousDeviceNumber === 0) {
            //If the number of previous device was zero (e.g at the beginning), then just overwrite
            new_device_names = device_names;
        } //Otherwise, get the ids of newly connected device
        else {
            new_index = previousDeviceNumber;

            new_device_names = find_difference_on_two_arrays(device_names, previous_device_ids);
        }
        remove_no_devices_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
        addNewDevicesToSidePanel(new_device_names);
        // add_list_items_to_trajectories_device(new_device_names);
        // add_list_items_to_map_tools_devices(new_device_names);
        allDeviceInfoArray = add_new_device_to_array(new_device_names, device_names_and_ids);
        previousDeviceNumber = currentDeviceNumber;
        temp_new_device_ids = new_device_names;
    }
    else if (currentDeviceNumber < previousDeviceNumber) {
        // a number of device has been disconnected/removed
        let removed_devices_ids = find_difference_on_two_arrays(device_names, previous_device_ids);
        // remove_list_items_from_selected_device(removed_devices_ids);
        removeDevicesFromSidePanel(removed_devices_ids);
        // remove_list_items_from_trajectories(removed_devices_ids);
        // remove_list_items_to_map_tools(removed_devices_ids, 'device-info-');
        hide_devices_from_map(removed_devices_ids);
    }

    previous_device_ids = device_names;
    previousDeviceNumber = currentDeviceNumber;

    if (allDeviceModelsLoaded(allDeviceInfoArray)) {
        //Before doing the actual update of the device's attributes, make sure that all 3d device models are loaded, otherwise we get an error
        canProceed = true;
        canAddTooltip = true;
    }

    if (canProceed) {
        for (let i = 0; i < allDeviceInfoArray.length; i++) {
            if (allDeviceInfoArray[i].deviceAvailability === AVAILABLE) {
                _wsDevices.forEach(function (device) {
                    if (device['name'] == allDeviceInfoArray[i].deviceID) {
                        if (device['telemetry'] === null) {
                            return;
                        }
                        telemetry = device['telemetry'];
                        updateDeviceWS(telemetry, allDeviceInfoArray, i, device['operator']).then(function () {
                            if (
                                pan_to_device_only_once &&
                                allDeviceInfoArray[i].deviceInfo.currentCoordinate[1] !== 0 &&
                                allDeviceInfoArray[i].deviceInfo.currentCoordinate[0] !== 0
                            ) {
                                pan_to_device_only_once = false;
                                map.flyTo({ center: allDeviceInfoArray[i].deviceInfo.currentCoordinate, zoom: 20 });
                                remove_line_layers_from_map_device(allDeviceInfoArray);
                            }
                            update_all_device_array(allDeviceInfoArray);
                        });
                        if (document.getElementById('deviceInfoBox' + allDeviceInfoArray[i].deviceID)) {
                            displayDeviceData(allDeviceInfoArray[i].deviceID);
                        }
                    }
                });
            }
        }
    }

    if (canAddTooltip && !tooltipAddedDevice) {
        //Add a tooltip to the 3d device models only if they were all successfully loaded otherwise we get an error
        tooltipAddedDevice = true;
        addTooltipOnDevices(temp_new_device_ids);
    }

    device_iter_index++;
}















var balora_iter_index = 0;
var tooltipAddedBalora = false;
var temp_new_balora_ids = [];
pan_to_balora_only_once = true;
/**
 * Starts the process of updating the balora's attributes and/or layers
 * */
function baloraUpdate(_wsBaloras) {

    let balora_names_and_ids = _wsBaloras.map((balora) => ({ balora_name: balora.name, balora_pk: balora.id }));

    let balora_names = balora_names_and_ids.map(function (balora) {
        return balora.balora_name;
    });
    var currentBaloraNumber = balora_names.length;

    // no baloras currently connected
    if (currentBaloraNumber === 0) {
        if (previousBaloraNumber > 0) {
            document.getElementById("trackers-menu").style.display = "none";  // hide sidebar section for baloras
            removeBalorasFromSidePanel(previous_balora_ids);                 // remove baloras that were disconnected
            add_no_baloras_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);
            hide_baloras_from_map(previous_balora_ids);
        }
        previousBaloraNumber = 0;
        return;
    }

    document.getElementById("connected-trackers").innerHTML = currentBaloraNumber; // show number of baloras on sidebar
    document.getElementById("trackers-menu").style.display = "block";             // show sidebar section for baloras


    // Add connected baloras
    if (currentBaloraNumber > previousBaloraNumber) {
        var new_balora_names = [];
        tooltipAddedBalora = false;
        canAddTooltip = false;
        canProceed = false;

        var new_index = 0;

        if (currentBaloraNumber - previousBaloraNumber === 0 || previousBaloraNumber === 0) {
            //If the number of previous balora was zero (e.g at the beginning), then just overwrite
            new_balora_names = balora_names;
        } //Otherwise, get the ids of newly connected balora
        else {
            new_index = previousBaloraNumber;

            new_balora_names = find_difference_on_two_arrays(balora_names, previous_balora_ids);
        }
        remove_no_baloras_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);
        allBaloraInfoArray = add_new_balora_to_array(new_balora_names, balora_names_and_ids);
        addNewBalorasToSidePanel(new_balora_names);
        console.log(allBaloraInfoArray);
        // add_list_items_to_trajectories_balora(new_balora_names);
        // add_list_items_to_selected_balora(new_balora_names);
        // add_list_items_to_map_tools_baloras(new_balora_names);
        previousBaloraNumber = currentBaloraNumber;
        temp_new_balora_ids = new_balora_names;
    } else if (currentBaloraNumber < previousBaloraNumber) {
        //Means that a number of balora has been disconnected/removed
        let removed_baloras_ids = find_difference_on_two_arrays(balora_names, previous_balora_ids);
        removeBalorasFromSidePanel(previous_balora_ids);
        // remove_list_items_from_selected_balora(removed_baloras_ids);
        // remove_list_items_from_trajectories(removed_baloras_ids);
        // remove_list_items_to_map_tools(removed_baloras_ids, 'balora-info-');
        hide_baloras_from_map(removed_baloras_ids);
    }

    previous_balora_ids = balora_names;
    previousBaloraNumber = currentBaloraNumber;

    if (allBaloraModelsLoaded(allBaloraInfoArray)) {
        //Before doing the actual update of the balora's attributes, make sure that all 3d balora models are loaded, otherwise we get an error
        canProceed = true;
        canAddTooltip = true;
    }
    if (canProceed) {
        for (let i = 0; i < allBaloraInfoArray.length; i++) {
            if (allBaloraInfoArray[i].baloraAvailability === AVAILABLE) {
                _wsBaloras.forEach(function (balora) {
                    if (balora['name'] == allBaloraInfoArray[i].baloraID) {
                        if (balora['telemetry'] === null) {
                            return;
                        }
                        if (
                            (balora['telemetry'].pm25 != undefined || balora['telemetry'].pm25 != null) &&
                            document.getElementById('balora-pm25-toggle').checked
                        ) {
                            updateHeatmapLayer(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].pm25);
                        }
                        updateBaloraWS(balora['telemetry'], allBaloraInfoArray, i, balora['balora_id']).then(function () {
                            if (
                                pan_to_balora_only_once &&
                                allBaloraInfoArray[i].baloraInfo.currentCoordinate[1] !== 0 &&
                                allBaloraInfoArray[i].baloraInfo.currentCoordinate[0] !== 0
                            ) {
                                pan_to_balora_only_once = false;
                                map.flyTo({ center: allBaloraInfoArray[i].baloraInfo.currentCoordinate, zoom: 20 });
                                remove_line_layers_from_map_balora(allBaloraInfoArray);
                            }
                            update_all_balora_array(allBaloraInfoArray);
                        });
                        if (document.getElementById('baloraInfoBox' + allBaloraInfoArray[i].baloraID)) {
                            displayBaloraData(allBaloraInfoArray[i].baloraID);
                        }
                    }
                });
            }
        }
    }

    if (canAddTooltip && !tooltipAddedBalora) {
        //Add a tooltip to the 3d balora models only if they were all successfully loaded otherwise we get an error
        tooltipAddedBalora = true;
        addTooltipOnBaloras(temp_new_balora_ids);
    }

    balora_iter_index++;
}



















/*
 * Handler for the feature selection where user is able to color-select a building and
 * display its height on a popup
 * */
function onSelectedFeatureChange(e) {
    console.log("onSelectedFeatureChange");
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

var el = document.createElement('div');
el.className = 'marker1';










/*
 * Takes as parameter the array with all drone objects and iterates over each object (each drone).
 * Every iteration, each drone object's attributes are updated with the latest data we get from the drone's telemetry
 * e.g the drone's current location, the drone's current battery etc.
 * */
let droneObj = {};
function updateDroneWS(telemetry, allDroneInfoArray, iteration) {
    let updatedDroneArray = allDroneInfoArray;
    let currentDrone = allDroneInfoArray[iteration];

    // console.log(currentDrone);

    // update sidebar data
    document.getElementById('sidepanel-drone-altitude-' + currentDrone.droneID).innerHTML = Math.round(telemetry.alt) + "m";
    document.getElementById('sidepanel-drone-battery-' + currentDrone.droneID).innerHTML = Math.round(telemetry.battery_percentage) + "%";

    setDroneAttributeValue(currentDrone.droneID, "droneState", telemetry.drone_state)
    setDroneAttributeValue(currentDrone.droneID, "vtolState", telemetry.vtol_state);

    // adjust MAVLINK buttons based on the drone's status
    if (allDroneInfoArray[iteration].droneType == "MAVLINK") {
        if (telemetry.drone_state == "Landed") {
            document.getElementById('mavlink-takeoff-land-' + currentDrone.droneID).innerHTML = "<i class='fa fa-plane-departure'></i> Takeoff";
            document.getElementById('mavlink-return-' + currentDrone.droneID).disabled = true;
            document.getElementById('mavlink-transition-' + currentDrone.droneID).disabled = true;
        }
        else {
            document.getElementById('mavlink-takeoff-land-' + currentDrone.droneID).innerHTML = "<i class='fa fa-plane-arrival'></i> Land";
            document.getElementById('mavlink-return-' + currentDrone.droneID).disabled = false;
            document.getElementById('mavlink-transition-' + currentDrone.droneID).disabled = false;
        }        
    }


    // let iconColor = (telemetry.drone_state == "In_Mission") ? "#9dffb3" : "#bbb";
    // let missionIcon = '<i class="fa fa-life-ring" style="color: ' + iconColor + ';"></i>';
    // document.getElementById('sidepanel-drone-status-' + currentDrone.droneID).innerHTML = missionIcon;

    let statusIcon = "fa fa-mountain";
    if(telemetry.drone_state == "In_Mission") {
        statusIcon = "fa fa-life-ring";
    }
    else if(telemetry.drone_state == "Paused_Mission") {
        statusIcon = "fa fa-circle-pause";
    }
    else if(telemetry.drone_state == "Flying") {
        statusIcon = "fa fa-paper-plane";
    }

    let statusIconColor = "white";
    let crpsStatus = "Disengaged";
    if (telemetry.crps_requested == true) {
        statusIcon = "fa fa-sitemap";
        statusIconColor = "orange";
        crpsStatus = "Requested";
    }
    else if (telemetry.crps_responding == true) {
        statusIcon = "fa fa-sitemap";
        statusIconColor = "#4efcb4";
        crpsStatus = "Responding";
    }

    statusIconHtml = '<i class="' + statusIcon + '" style="color: ' + statusIconColor + '; font-size: 14px;" title="' + telemetry.drone_state + '"></i>';

    let droneStatusHeader = document.getElementById('sidepanel-drone-status-' + currentDrone.droneID)
    droneStatusHeader.innerHTML = statusIconHtml;
    $(droneStatusHeader).attr("title", telemetry.drone_state);

    

    currentDrone.previousdroneMissionState = currentDrone.droneMissionState;
    currentDrone.droneMissionState = telemetry.drone_state;
    currentDrone.droneInfo.currentBatteryLevel = telemetry.battery_percentage;
    currentDrone.droneInfo.satellites = telemetry.satellites;
    // var selectElement = document.getElementById('clickSelect');
    // var goElement = document.getElementById('clickGo');
    // var cancelElement = document.getElementById('clickCancel');
    // var pauseElement = document.getElementById('clickPause');
    // let clickGo = $('#clickGo');
    try {
        droneObj.time = telemetry.time;
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
        droneObj.under_water = telemetry.water_sampler_in_water;
        droneObj.crpsStatus = crpsStatus;
        droneObj.vtol_state = telemetry.vtol_state;
        droneObj.droneType = allDroneInfoArray[iteration].droneType;
        displayDroneData(droneObj, currentDrone.droneID);


        droneObj = {};
    } catch (error) {}
    updatedDroneArray = updateLocalDroneAttributes(currentDrone, iteration, allDroneInfoArray, telemetry); //Perform the actual update with the new locations here
    if (currentDrone.droneObject.selected) {
        updateMissionButtonStates(currentDrone.droneObject);
    }
    if (currentDrone.droneMissionIntent === MOVING) {
        // do nothing
    } else if (currentDrone.droneMissionIntent === WANT_TO_START_MISSION) {
        //If user wants to start mission, wait for approval from drone
        if (currentDrone.droneMissionState === IN_MISSION) {
            //Approval granted!
            clear_interval(WANT_TO_START_MISSION); //Stop the countdown timer for this action
            // $(selectElement).removeClass('active');
            enableDroneSelection();
            enableBuildingSelection();
            // reset_marker_counter(currentDrone.droneID);
            setDroneAttributeValue(currentDrone.droneID, "droneMarkerCounter", 0);

            // clickGo.text('PAUSE');
            // clickGo.removeClass('btn-outline-success')
            // clickGo.addClass('btn-outline-primary')
            // clickGo.addClass('active')
            // $(selectElement).removeClass('active');
            // currentDrone.droneMissionIntent = MOVING
            // let drone_index = get_drone_index(currentDrone.droneID);
            // updateDroneMissionIntentLocally(drone_index, MOVING);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", MOVING);
            hidePopup('#loadingBox');
            hidePopup('#mission_failure_box_retry');
            // showPopupForALittle("#successBox", '', 2000)
            let msg = `Mission successfully started for drone ${currentDrone.droneID} !`;
            // showPopupForALittle("#successBox", '', 2000)
            create_popup_for_a_little(SUCCESS_ALERT, msg, 2000);
            // emptyMissionPath(currentDrone.droneID)
            // get_selected_drone_id()
            // updateMissionButtonStates(currentDrone.droneObject)
        }
    } else if (currentDrone.droneMissionIntent === WANT_TO_PAUSE_MISSION) {
        if (currentDrone.droneMissionState === PAUSED_MISSION) {
            hidePopup('#pause_failure_box_retry');

            enableDroneSelection();
            enableBuildingSelection();
            hidePopup('#pausingMissionBox');
            showPopupForALittle('#missionPausedBox', '', 2000);

            // $(selectElement).removeClass('active');
            // clickGo.text('RESUME');
            // clickGo.addClass('active')

            // reset_marker_counter(currentDrone.droneID);
            setDroneAttributeValue(currentDrone.droneID, "droneMarkerCounter", 0);

            // let drone_index = get_drone_index(currentDrone.droneID);
            // updateDroneMissionIntentLocally(drone_index, NO_MOVING);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", NO_MOVING);
        }
    } else if (currentDrone.droneMissionIntent === WANT_TO_RESUME_MISSION) {
        if (currentDrone.droneMissionState === IN_MISSION) {
            hidePopup('#resume_failure_box_retry');
            hidePopup('#resumingMissionBox');
            showPopupForALittle('#missionResumedBox', '', 2000);
            enableDroneSelection();
            enableBuildingSelection();

            // $(selectElement).removeClass('active');
            // $(selectElement).removeClass('active');
            // $(goElement).addClass('active');
            // clickGo.text('PAUSE');

            // currentDrone.droneMissionIntent = MOVING
            // let drone_index = get_drone_index(currentDrone.droneID);
            // updateDroneMissionIntentLocally(drone_index, MOVING);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", MOVING);
        }
    } else if (currentDrone.droneMissionIntent === WANT_TO_CANCEL_MISSION) {
        if (currentDrone.droneMissionState === FLYING) {
            enableDroneSelection();
            enableBuildingSelection();
            hidePopup('#cancel_failure_box_retry');
            hidePopup('#cancellingMissionBox');
            showPopupForALittle('#missionCancelledBox', '', 2000);
            //
            // $(selectElement).removeClass('active');
            // clickGo.removeClass('btn-outline-primary')
            // clickGo.addClass('btn-outline-success')
            // clickGo.removeClass('active')
            // clickGo.text('GO')
            // $("#clickSelect").removeClass('active')
            // $("#clickCancel").removeClass('active')

            remove_drone_markers(currentDrone.droneID);
            // emptyMissionPath(currentDrone.droneID);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionPath", []);
            // reset_marker_counter(currentDrone.droneID);
            setDroneAttributeValue(currentDrone.droneID, "droneMarkerCounter", 0);

            enableDroneSelection(); //Let user be able to select/unselect drone again
            // currentDrone.droneMissionIntent = MOVING
            // let drone_index = get_drone_index(currentDrone.droneID);
            // updateDroneMissionIntentLocally(drone_index, MOVING);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", MOVING);
        }
    } else if (currentDrone.droneMissionIntent === WANT_TO_SELECT_MISSION) {
        disableBuildingSelection(); //Don't let user select buildings because if he does so, the drone will get un-selected
        disableDroneSelection();
        $("#clickSelect").addClass('active');
    } else if (currentDrone.droneMissionIntent === NO_MOVING) {
        enableDroneSelection();
        enableBuildingSelection();
    }

    // let markerCounter = getDroneCurrentMarkersOnMap(currentDrone.droneID)
    // console.log("MARKER COUNTER OF DRONE: " + markerCounter)
    // if (markerCounter === 0) //Means that mission was initiated from another client. Now we have to load the mission markers on this client as well.
    // {
    //     $.getJSON(currentDrone.droneURL).then(function (drone_obj)
    //     {
    //         let mission_state = drone_obj.properties.droneMissionState
    //         if (mission_state === IN_MISSION || mission_state === PAUSED_MISSION)
    //         {
    //             let waypoints = drone_obj['properties']['mission']['persistent_missionPath']
    //
    //             place_waypoints_on_map(waypoints, currentDrone.droneID)
    //         }
    //     });
    // }

    if (currentDrone.previousdroneMissionState === IN_MISSION || currentDrone.previousdroneMissionState === PAUSED_MISSION) {
        if (currentDrone.droneMissionState === FLYING) {
            //Mission completed successfully
            showPopupForALittle('#missionDoneBox', '', 2000);
            $('#clickGo').removeClass('active');
            $('#clickGo').text('GO');
            $('#clickGo').removeClass('btn-outline-primary');
            $('#clickGo').addClass('btn-outline-success');
            $('#clickSelect').removeClass('active');
            $('#clickCancel').removeClass('active');
            remove_drone_markers(currentDrone.droneID);
            // reset_marker_counter(currentDrone.droneID);
            setDroneAttributeValue(currentDrone.droneID, "droneMarkerCounter", 0);
            clearMissionPathLine(currentDrone.droneID);
        } else if (currentDrone.droneMissionState === PAUSED_MISSION && currentDrone.previousdroneMissionState === IN_MISSION) {
            //In case the mission was paused from another client
            // let drone_index = get_drone_index(currentDrone.droneID);
            // updateDroneMissionIntentLocally(drone_index, WANT_TO_PAUSE_MISSION);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", WANT_TO_PAUSE_MISSION);
        }
    }
    // If there is active mission but no waypoint markers exist for this drone, it means that mission was initiated from another client (pc). Now we have to load the mission markers on this client's map as well.
    else if (
        currentDrone.previousdroneMissionState === FLYING &&
        currentDrone.droneMissionState === IN_MISSION &&
        currentDrone.droneCurrentMarkers.length === 0
    ) {
        // let drone_index = get_drone_index(currentDrone.droneID);
        // updateDroneMissionIntentLocally(drone_index, WANT_TO_START_MISSION);
        setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", WANT_TO_START_MISSION);
        $.getJSON(currentDrone.droneURL).then(function (drone_obj) {
            let waypoints = drone_obj['properties']['mission']['persistentMissionPath'];
            place_waypoints_on_map(waypoints, currentDrone.droneID);
            // setDroneMissionPath(currentDrone.droneID, waypoints);
            setDroneAttributeValue(currentDrone.droneID, "droneMissionPath", waypoints);
        });
    } else if (currentDrone.previousdroneMissionState === PAUSED_MISSION && currentDrone.droneMissionState === IN_MISSION) {
        // let drone_index = get_drone_index(currentDrone.droneID);
        // updateDroneMissionIntentLocally(drone_index, WANT_TO_RESUME_MISSION);
        setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", WANT_TO_RESUME_MISSION);
    } else if (
        (currentDrone.previousdroneMissionState === IN_MISSION || currentDrone.previousdroneMissionState === PAUSED_MISSION) &&
        currentDrone.droneMissionState === FLYING
    ) {
        // let drone_index = get_drone_index(currentDrone.droneID);
        // updateDroneMissionIntentLocally(drone_index, WANT_TO_PAUSE_MISSION);
        setDroneAttributeValue(currentDrone.droneID, "droneMissionIntent", WANT_TO_PAUSE_MISSION);
    }

    updatedDroneArray[iteration] = currentDrone;
    return new Promise((resolve, reject) => resolve(updatedDroneArray));
}









let deviceObj = {};
function updateDeviceWS(telemetry, allDeviceInfoArray, iteration, operator) {
    let updatedDeviceArray = allDeviceInfoArray;
    let currentDevice = allDeviceInfoArray[iteration];
    document.getElementById('mobile-device-battery-' + currentDevice.deviceID).innerHTML = Math.round(telemetry.battery_percentage) + "%";
    currentDevice.deviceInfo.currentBatteryLevel = telemetry.battery_percentage;
    currentDevice.deviceInfo.time = telemetry.time;
    currentDevice.deviceInfo.operator = operator;
    updatedDeviceArray = updateLocalDeviceAttributes(currentDevice, iteration, allDeviceInfoArray, telemetry); //Perform the
    updatedDeviceArray[iteration] = currentDevice;
    return new Promise((resolve, reject) => resolve(updatedDeviceArray));
}

let baloraObj = {};
function updateBaloraWS(telemetry, allBaloraInfoArray, iteration, receiver) {
    let updatedBaloraArray = allBaloraInfoArray;
    let currentBalora = allBaloraInfoArray[iteration];
    document.getElementById('lora-tracker-battery-' + currentBalora.baloraID).innerHTML = Math.round(telemetry.battery_percentage) + "%";
    currentBalora.baloraInfo.currentBatteryLevel = telemetry.battery_percentage;
    currentBalora.baloraInfo.time = telemetry.time;
    currentBalora.baloraInfo.receiver = receiver;
    currentBalora.baloraInfo.pm1 = telemetry.pm1;
    currentBalora.baloraInfo.pm25 = telemetry.pm25;
    currentBalora.baloraInfo.rssi = telemetry.received_signal_strength_indication;
    currentBalora.baloraInfo.snr = telemetry.SignalToNoiseRatio;
    updatedBaloraArray = updateLocalBaloraAttributes(currentBalora, iteration, allBaloraInfoArray, telemetry); //Perform the
    updatedBaloraArray[iteration] = currentBalora;
    return new Promise((resolve, reject) => resolve(updatedBaloraArray));
}







/*
 * Performs the actual update of the current drone's attributes (location, layers etc.)
 * This function is called periodically (Every X ms)
 * Returns the updated array of the drones with the new drone's coordinates, new line data etc.
 * */
function updateLocalDroneAttributes(currentDrone, iteration, allDronesArray, telemetry) {
    if (currentDrone.droneInfo.currentCoordinate !== null) {
        currentDrone.droneInfo.previousCoordinate = currentDrone.droneInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    currentDrone.droneInfo.currentCoordinate = [telemetry.lon, telemetry.lat, telemetry.alt];
    // console.log("TELEMETRY HEADING", telemetry.heading)
    currentDrone.droneInfo.heading = telemetry.heading;
    currentDrone.droneInfo.velocity = telemetry.velocity;
    currentDrone.droneInfo.altitude = telemetry.alt;
    // updateDroneTooltip(currentDrone)
    var currentLineData;

    currentLineData = {
        source: currentDrone.droneInfo.previousCoordinate,
        dest: currentDrone.droneInfo.currentCoordinate,
        color: [23, 184, 190],
    };
    currentDrone = updateDroneLineLayer(currentDrone, currentLineData);
    if (currentDrone.droneObject !== undefined) {
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














function updateLocalDeviceAttributes(currentDevice, iteration, allDevicesArray, telemetry) {
    if (currentDevice.deviceInfo.currentCoordinate !== null) {
        currentDevice.deviceInfo.previousCoordinate = currentDevice.deviceInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    currentDevice.deviceInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, 0];
    // currentDevice.deviceInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, telemetry.altitude];
    currentDevice.deviceInfo.heading = telemetry.heading;
    currentDevice.deviceInfo.altitude = 0;
    // currentDevice.deviceInfo.altitude = telemetry.altitude;
    var currentLineData;

    currentLineData = {
        source: currentDevice.deviceInfo.previousCoordinate,
        dest: currentDevice.deviceInfo.currentCoordinate,
        color: [23, 184, 190],
    };
    // line late update
    currentDevice = updateDeviceLineLayer(currentDevice, currentLineData);
    currentDevice.deviceObject.setCoords([
        currentDevice.deviceInfo.currentCoordinate[0],
        currentDevice.deviceInfo.currentCoordinate[1],
        currentDevice.deviceInfo.currentCoordinate[2],
    ]);
    currentDevice.deviceObject.setRotation(-telemetry.heading);

    allDevicesArray[iteration] = currentDevice;

    return allDevicesArray;
}












function updateLocalBaloraAttributes(currentBalora, iteration, allBalorasArray, telemetry) {
    if (currentBalora.baloraInfo.currentCoordinate !== null) {
        currentBalora.baloraInfo.previousCoordinate = currentBalora.baloraInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    currentBalora.baloraInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, 0];
    // currentBalora.baloraInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, telemetry.altitude];
    currentBalora.baloraInfo.heading = telemetry.heading;
    currentBalora.baloraInfo.altitude = 0;
    // currentBalora.baloraInfo.altitude = telemetry.altitude;
    var currentLineData;

    currentLineData = {
        source: currentBalora.baloraInfo.previousCoordinate,
        dest: currentBalora.baloraInfo.currentCoordinate,
        color: [23, 184, 190],
    };
    // line late update
    currentBalora = updateBaloraLineLayer(currentBalora, currentLineData);
    currentBalora.baloraObject.setCoords([
        currentBalora.baloraInfo.currentCoordinate[0],
        currentBalora.baloraInfo.currentCoordinate[1],
        currentBalora.baloraInfo.currentCoordinate[2],
    ]);
    currentBalora.baloraObject.setRotation(-telemetry.heading);

    allBalorasArray[iteration] = currentBalora;

    return allBalorasArray;
}





/*Returns true if all drone 3d models were correctly loaded, and false otherwise*/
function allDroneModelsLoaded(allDrones) {
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








/*Returns true if all drone 3d models were correctly loaded, and false otherwise*/
function allDeviceModelsLoaded(allDevice) {
    if (allDevice[0] === 'undefined') {
        return false;
    }
    for (let i = 0; i < allDevice.length; i++) {
        let obj = allDevice[i].deviceObject;
        if (isObjEmpty(obj)) {
            return false;
        }
    }
    return true;
}






/*Returns true if all drone 3d models were correctly loaded, and false otherwise*/
function allBaloraModelsLoaded(allBalora) {
    if (allBalora[0] === 'undefined') {
        return false;
    }
    for (let i = 0; i < allBalora.length; i++) {
        let obj = allBalora[i].baloraObject;
        if (isObjEmpty(obj)) {
            return false;
        }
    }
    return true;
}





/*
 * Clears the specified timer
 * */
function clear_timer(timer) {
    if (timer !== undefined) {
        clearInterval(timer);
    }
}







/*
 * Returns true if all toggles about the detection drones are off
 * */
function are_all_detection_toggles_off() {
    let detVideoToggles = $('[id^="detection-toggle"]');
    for (let i = 0; i < detVideoToggles.length; i++) {
        let pressed = $(detVideoToggles[i]).is(':checked');
        if (pressed) {
            return false;
        }
    }
    return true;
}

