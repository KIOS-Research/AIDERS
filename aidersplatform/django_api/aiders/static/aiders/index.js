const { MapboxLayer, LineLayer, COORDINATE_SYSTEM } = deck;
let currentStyle = sessionStorage.getItem('currentStyle');
let currentStyleRadioBtn = sessionStorage.getItem('currentStyleRadioBtn');
let lastDroneLocation = JSON.parse(sessionStorage.getItem('lastDroneLocation'));
if (currentStyle === null) {
    //If it's the first time loading the platform, this variable is null
    currentStyle = 'https://api.maptiler.com/maps/<>/style.json?key=blpQOMdNw0JJIq07I9Ln'.replace('<>', BASIC_STYLE_ID);
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
// mapboxgl.accessToken = 'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg';

// map.addControl(
//     new MapboxGeocoder({
//     accessToken: mapboxgl.accessToken,
//     mapboxgl: mapboxgl
//     })
// );

/*This element is a searchbox that allows user to search locations*/
var geocoder = new MapboxGeocoder({
    accessToken: 'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg',
    placeholder: "Try 'Cyprus' or '33,35' or a drone's ID",
    mapboxgl: maplibregl,
    marker: true,
    localGeocoder: coordinatesGeocoder,
});

document.getElementById('geocoder').appendChild(geocoder.onAdd(map));

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

let droneTimer;

let currentMapStyle = '';
let previousMapStyle = currentMapStyle;
let first_time_flag = false;
let first_time = true;

function connect_to_ws(wsUrl) {
    console.log('ws://' + window.location.host + '/' + dutils.urls.resolve(wsUrl));
    var ws = new WebSocket('ws://' + window.location.host + '/' + dutils.urls.resolve(wsUrl));
    ws.onopen = function () {
        // subscribe to some channels
        ws.send(CURRENT_OP);
    };

    ws.onclose = function (e) {
        console.log('Socket is closed. Reconnect will be attempted in 1 second.');
        setTimeout(function () {
            connect_to_ws(wsUrl);
        }, 1000);
    };

    ws.onerror = function (err) {
        console.error('Socket encountered error: ', err.message, 'Closing socket');
        ws.close();
    };
    return ws;
}
var socket = connect_to_ws('ws_index');

let webSocketMessage;
addListeners();
map.on('load', function () {
    add_available_detector_types_on_panel();

    let drones_url = dutils.urls.resolve('drones', { operation_name: CURRENT_OP });

    // map.addLayer(threeDbuildingLayer);

    map.on('SelectedFeatureChange', onSelectedFeatureChange);

    var drone_iter_index = 0;
    var previousDroneNumber = 0;
    var previous_drone_ids = [];
    var currentDroneNumber = 0;
    let iter_iter = 0;
    var allDroneInfoArray = [];

    let temp_interval = setInterval(checkIfDetectionTypesPublished, 2000);

    add_no_drones_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS);

    /*
     * There are several constant variables that need to be published from API.
     * Among them, there is a list of object types that drone can detect with computer vision. This list is published from the API
     * This function takes care of this list, once it is published from api
     * */
    function checkIfDetectionTypesPublished() {
        if (WEB_SERVER_URL !== undefined) {
            http_get_available_types().then(function (all_model_types) {
                all_model_types = all_model_types.split('\n');

                for (let i = 0; i < all_model_types.length; i++) {
                    all_model_types[i] = all_model_types[i].trim();
                }

                all_model_types.splice(3, all_model_types.length); // To display only the first 3 model types. Remove this to display all types

                create_prototype_models(all_model_types);

                add_list_items_to_detection_types();
                let protModelsLayer = get_prot_models_layer(all_model_types); //get the layer that will host all the detected objects found by drone
                map.addLayer(protModelsLayer);
            });
            clearInterval(temp_interval);
        }
    }
    let runOnce = 0;
    function updateDataWs() {
        socket.send(CURRENT_OP);
        if (runOnce === 0) {
            runOnce = 1;
            socket.onmessage = function (e) {
                wsData = JSON.parse(e.data);
                // console.log(wsData)
                webSocketMessage = wsData;
                drones = wsData['drones'];
                if (wsData['error_msg'].length !== 0) {
                    console.log(wsData['error_msg']);
                    error_msg = wsData['error_msg'];
                    create_popup_for_a_little(WARNING_ALERT, error_msg, 10000);
                }
                let drone_names_and_ids = [];
                drones.forEach(function (drone) {
                    if (drone.is_connected_with_platform) {
                        drone_names_and_ids.push({
                            drone_name: drone.drone_name,
                            drone_pk: drone.id,
                        });
                    } else {
                        removeByAttr(drone_names_and_ids, 'drone_name', drone.drone_name);
                    }
                });
                droneUpdate(drone_names_and_ids, wsData);
            };
        }
    }

    droneTimer = setInterval(updateDataWs, 90);
    // droneTimer = setInterval(checkForNewDrones, UPDATE_INTERVAL)

    var canAddTooltip = false;
    var temp_new_drone_ids = [];
    var tooltipAdded = false;
    let canProceed = false;
    let pan_to_drone_only_once = true;

    /*
     *Starts the process of updating the drone's attributes and/or layers (e.g the drone's position, the drone's line layer etc.)
     * */
    function droneUpdate(drone_names_and_ids, wsData) {
        let drone_names = drone_names_and_ids.map(function (drone) {
            return drone.drone_name;
        });
        let drone_pks = drone_names_and_ids.map(function (drone) {
            return drone.pk;
        });
        var currentDroneNumber = drone_names.length;

        if (drone_names.length === 0) {
            if (previousDroneNumber > 0) {
                //If we had drones before and now we don't have any, remove elements associated with them
                update_drones_local_availability(previous_drone_ids, NOT_AVAILABLE);
                remove_list_items_from_trajectories(previous_drone_ids);
                remove_list_items_from_uav_missions(previous_drone_ids);
                remove_list_items_from_video_feeds(previous_drone_ids);
                remove_list_items_from_det_video_feeds(previous_drone_ids);
                remove_list_items_from_enable_detections(previous_drone_ids);
                remove_list_items_from_select_drones(previous_drone_ids);
                remove_list_items_from_map_tools(previous_drone_ids);
                remove_video_element(previous_drone_ids);
                hide_drones_from_map(previous_drone_ids);
                remove_det_video_elements(previous_drone_ids);
                add_no_drones_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS);
            }

            previousDroneNumber = 0;
            return;
        }

        if (currentDroneNumber > previousDroneNumber) {
            //Means that new drones connected. Thus, we have to create  new drone objects and elements
            var new_drone_names = [];
            tooltipAdded = false;
            canAddTooltip = false;
            canProceed = false;
            var howManyNewDrones = currentDroneNumber - previousDroneNumber;

            var new_index = 0;

            if (howManyNewDrones === 0 || previousDroneNumber === 0) {
                //If the number of previous drones was zero (e.g at the beginning), then just overwrite
                new_drone_names = drone_names;
            } //Otherwise, get the ids of newly connected drones
            else {
                new_index = previousDroneNumber;

                new_drone_names = find_difference_on_two_arrays(drone_names, previous_drone_ids);
            }

            previousDroneNumber = currentDroneNumber;
            temp_new_drone_ids = new_drone_names;

            allDroneInfoArray = add_new_drones_to_array(new_drone_names, drone_names_and_ids); //It's time to add the new drones to the drones array

            remove_no_drones_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS);
            add_list_items_to_trajectories(new_drone_names);

            add_list_items_to_uav_missions(new_drone_names);
            add_list_items_to_video_feeds(new_drone_names);
            add_list_items_to_det_video_feeds(new_drone_names);
            add_list_items_to_map_tools(new_drone_names);
            webSocketMessage['drones'].forEach(function (drone) {
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
                }
            });
            add_list_items_to_enable_detections_list(new_drone_names);
            create_video_elements(new_drone_names);
            create_det_video_elements(new_drone_names);

            // update_local_build_map_status_only_once_for_all_drones(new_drone_ids)
            // update_MS_local_build_map_status_only_once_for_all_drones(new_drone_ids)

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
                wsData['drones'].forEach(function (drone) {
                    if (drone['drone_name'] == new_drone_names[i]) {
                        if (drone['telemetry'] === null) {
                            return;
                        }
                        telemetry = drone['telemetry'];
                        let mission_state = telemetry.drone_state;
                        if (mission_state === IN_MISSION || mission_state === PAUSED_MISSION) {
                            $.getJSON(allDroneInfoArray[drone_index].missionPointsURL).then(function (points) {
                                let waypoints = [];
                                points.forEach(function (point, index) {
                                    point = [point['point']['longitude'], point['point']['latitude']];
                                    waypoints[index] = point;
                                });
                                $.getJSON(allDroneInfoArray[drone_index].missionURL).then(function (mission) {
                                    console.log(waypoints);
                                    if (mission === 'SEARCH_AND_RESCUE_MISSION') {
                                        addLineLayerInIndex(waypoints, allDroneInfoArray[drone_index].droneID);
                                        let point_of_waypoints = [];
                                        point_of_waypoints.push(waypoints[0]);
                                        point_of_waypoints.push(waypoints[8]);
                                        point_of_waypoints.push(waypoints[81]);
                                        point_of_waypoints.push(waypoints[waypoints.length - 1]);
                                        place_waypoints_on_map(point_of_waypoints, drone_names_and_ids[i].pk);
                                    } else {
                                        place_waypoints_on_map(waypoints, allDroneInfoArray[drone_index].droneID);
                                    }
                                });
                            });
                        }
                    }
                    drone_data = drone;
                    let drone_Build_map_state = drone_data.build_map_activated;
                    if (drone_Build_map_state === true) {
                        $.ajax({
                            type: 'POST',
                            url: dutils.urls.resolve('get_last_build_map', {
                                operation_name: CURRENT_OP,
                            }),
                            data: { drone_id: allDroneInfoArray[drone_index].droneID },
                            headers: {
                                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                            },
                            success: function (response) {
                                response = JSON.parse(response);
                                postBuildMapLoad(
                                    response['id'],
                                    dutils.urls.resolve('load_build_map', {
                                        operation_name: CURRENT_OP,
                                    }),
                                    document.getElementById('csrf').querySelector('input').value
                                );
                                global_active_build_map_periods.push(response);
                            },
                        });
                    }
                });
                startListeningForBuildMap(new_drone_names[i]);
            }

            first_time_flag = true;
            previousMapStyle = currentMapStyle;
        } else if (currentDroneNumber < previousDroneNumber) {
            //Means that a number of drones has been disconnected/removed
            let removed_drones_ids = find_difference_on_two_arrays(drone_names, previous_drone_ids);
            update_drones_local_availability(removed_drones_ids, NOT_AVAILABLE);
            remove_list_items_from_trajectories(removed_drones_ids);
            remove_list_items_from_uav_missions(removed_drones_ids);
            remove_list_items_from_video_feeds(removed_drones_ids);
            remove_list_items_from_det_video_feeds(removed_drones_ids);
            remove_list_items_from_enable_detections(removed_drones_ids);
            remove_list_items_from_select_drones(removed_drones_ids);
            remove_list_items_from_detection_types(removed_drones_ids);
            remove_video_element(removed_drones_ids);
            hide_drones_from_map(removed_drones_ids);
            remove_det_video_elements(removed_drones_ids);
        }

        previous_drone_ids = drone_names;
        previousDroneNumber = currentDroneNumber;

        if (allModelsLoaded(allDroneInfoArray)) {
            //Before doing the actual update of the drone's attributes, make sure that all 3d drone models are loaded, otherwise we get an error
            canProceed = true;
            canAddTooltip = true;
        }

        if (canProceed) {
            for (let i = 0; i < allDroneInfoArray.length; i++) {
                if (allDroneInfoArray[i].droneAvailability === AVAILABLE) {
                    let updatedDroneArray = allDroneInfoArray;
                    let currentDrone = allDroneInfoArray[i];
                    wsData['drones'].forEach(function (drone) {
                        if (drone['drone_name'] == currentDrone.droneID) {
                            if (drone['telemetry'] === null) {
                                return;
                            }
                            telemetry = drone['telemetry'];
                            updateDroneWS(telemetry, allDroneInfoArray, i).then(function () {
                                let currentDroneLocation = allDroneInfoArray[i].droneInfo.currentCoordinate;
                                let lon = currentDroneLocation[0];
                                let lat = currentDroneLocation[1];
                                if (pan_to_drone_only_once && lat !== 0 && lon !== 0) {
                                    pan_to_drone_only_once = false;
                                    map.flyTo({ center: currentDroneLocation, zoom: 20 });
                                    remove_line_layers_from_map(allDroneInfoArray);
                                }
                                update_all_drone_array(updatedDroneArray);
                            });
                        }
                    });
                }
            }
        }

        if (canAddTooltip && !tooltipAdded) {
            //Add a tooltip to the 3d drone models only if they were all successfully loaded otherwise we get an error
            tooltipAdded = true;
            addTooltipOnDrones(temp_new_drone_ids);
        }
        drone_iter_index++;
    }
});

// Weather Startup
$.ajax({
    type: 'POST',
    url: dutils.urls.resolve('weather_live', { operation_name: CURRENT_OP }),
    data: { state: true },
    headers: {
        'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
    },
});

window.addEventListener('beforeunload', function (e) {
    $.ajax({
        type: 'POST',
        url: dutils.urls.resolve('weather_live', { operation_name: CURRENT_OP }),
        data: { state: false },
        headers: {
            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
        },
    });
});

/*
 * Handler for the feature selection where user is able to color-select a building and
 * display its height on a popup
 * */
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

    currentDrone.previousdroneMissionState = currentDrone.droneMissionState;
    currentDrone.droneMissionState = telemetry.drone_state;
    currentDrone.droneInfo.currentBatteryLevel = telemetry.battery_percentage;
    var selectElement = document.getElementById('clickSelect');
    var goElement = document.getElementById('clickGo');
    var cancelElement = document.getElementById('clickCancel');
    var pauseElement = document.getElementById('clickPause');
    let clickGo = $('#clickGo');
    try {
        droneObj.time = telemetry.received_at;
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
        displayDroneData(droneObj, currentDrone.droneID);
        droneObj = {};
    } catch (error) {}
    updatedDroneArray = updateLocalDroneAttributes(currentDrone, iteration, allDroneInfoArray, telemetry); //Perform the actual update with the new locations here
    if (currentDrone.droneObject.selected) {
        updateMissionButtonStates(currentDrone.droneObject);
        updateBuildMapButtonStates(currentDrone, true);
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
            reset_marker_counter(currentDrone.droneID);
            // clickGo.text('PAUSE');
            // clickGo.removeClass('btn-outline-success')
            // clickGo.addClass('btn-outline-primary')
            // clickGo.addClass('active')
            // $(selectElement).removeClass('active');
            // currentDrone.droneMissionIntent = MOVING
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, MOVING);
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

            reset_marker_counter(currentDrone.droneID);
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, NO_MOVING);
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
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, MOVING);
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
            emptyMissionPath(currentDrone.droneID);
            reset_marker_counter(currentDrone.droneID);
            enableDroneSelection(); //Let user be able to select/unselect drone again
            // currentDrone.droneMissionIntent = MOVING
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, MOVING);
        }
    } else if (currentDrone.droneMissionIntent === WANT_TO_SELECT_MISSION) {
        disableBuildingSelection(); //Don't let user select buildings because if he does so, the drone will get un-selected
        disableDroneSelection();
        $(selectElement).addClass('active');
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

    if (
        currentDrone.previousdroneMissionState === IN_MISSION ||
        currentDrone.previousdroneMissionState === PAUSED_MISSION
    ) {
        if (currentDrone.droneMissionState === FLYING) {
            //Mission completed successfully
            showPopupForALittle('#missionDoneBox', '', 2000);
            clickGo.removeClass('active');
            clickGo.text('GO');
            clickGo.removeClass('btn-outline-primary');
            clickGo.addClass('btn-outline-success');
            $('#clickSelect').removeClass('active');
            $('#clickCancel').removeClass('active');
            remove_drone_markers(currentDrone.droneID);
            reset_marker_counter(currentDrone.droneID);
            clearMissionPathLine(currentDrone.droneID);
        } else if (
            currentDrone.droneMissionState === PAUSED_MISSION &&
            currentDrone.previousdroneMissionState === IN_MISSION
        ) {
            //In case the mission was paused from another client
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, WANT_TO_PAUSE_MISSION);
        }
    }
    // If there is active mission but no waypoint markers exist for this drone, it means that mission was initiated from another client (pc). Now we have to load the mission markers on this client's map as well.
    else if (
        currentDrone.previousdroneMissionState === FLYING &&
        currentDrone.droneMissionState === IN_MISSION &&
        currentDrone.droneCurrentMarkers.length === 0
    ) {
        let drone_index = get_drone_index(currentDrone.droneID);
        updateDroneMissionIntentLocally(drone_index, WANT_TO_START_MISSION);
        $.getJSON(currentDrone.droneURL).then(function (drone_obj) {
            let waypoints = drone_obj['properties']['mission']['persistentMissionPath'];
            place_waypoints_on_map(waypoints, currentDrone.droneID);
            setDroneMissionPath(currentDrone.droneID, waypoints);
        });
    } else if (
        currentDrone.previousdroneMissionState === PAUSED_MISSION &&
        currentDrone.droneMissionState === IN_MISSION
    ) {
        let drone_index = get_drone_index(currentDrone.droneID);
        updateDroneMissionIntentLocally(drone_index, WANT_TO_RESUME_MISSION);
    } else if (
        currentDrone.previousdroneMissionState === IN_MISSION ||
        currentDrone.previousdroneMissionState === PAUSED_MISSION
    ) {
        if (currentDrone.droneMissionState === FLYING) {
            let drone_index = get_drone_index(currentDrone.droneID);
            updateDroneMissionIntentLocally(drone_index, WANT_TO_PAUSE_MISSION);
        }
    }

    updatedDroneArray[iteration] = currentDrone;
    return new Promise((resolve, reject) => resolve(updatedDroneArray));
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

    currentDrone = updateLineLayer(currentDrone, currentLineData);
    currentDrone.droneObject.setCoords([
        currentDrone.droneInfo.currentCoordinate[0],
        currentDrone.droneInfo.currentCoordinate[1],
        currentDrone.droneInfo.currentCoordinate[2],
    ]);
    currentDrone.droneObject.setRotation(-telemetry.heading);

    allDronesArray[iteration] = currentDrone;

    return allDronesArray;
}

/*Returns true if all drone 3d models were correctly loaded, and false otherwise*/
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

function addListeners() {
    list_user_action_menu = [
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
    for (let i = 0; i < list_user_action_menu.length; i++) {
        $('#' + list_user_action_menu[i]).click(function () {
            if ($('#' + list_user_action_menu[i])[0].attributes.class.nodeValue.includes('active')) {
                postElementId(list_user_action_menu[i], true);
            } else {
                postElementId(list_user_action_menu[i], false);
            }
        });
    }
}
