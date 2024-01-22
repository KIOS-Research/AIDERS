const { MapboxLayer, LineLayer, COORDINATE_SYSTEM } = deck;
visualization_player = document.getElementById('progress-bar');
let webSocketMessage = {};
let isFirstTimeLoad = true;

function main_map() {
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
    maplibregl.accessToken = 'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg';
    var map = new maplibregl.Map({
        container: 'map',
        style: currentStyle,
        zoom: 7,
        center: [33.0028145, 35.0725358],
    });
    if (USE_ONLINE_MAPS) {
        /*This element is a searchbox that allows user to search locations*/
        var geocoder = new MapboxGeocoder({
            accessToken: maplibregl.accessToken,
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
        initialize_replay_bar();
        add_no_drones_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DRONES);
        add_no_devices_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
        add_no_baloras_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);

        if (time_data.drones !== undefined) {
            createDroneObjectOnMap(time_data.drones);
            createDronesTrajectoryLineOfReplay(time_data.time_series_data);
            checkIfDronesHaveWeather(time_data.time_series_data)
        }
        if (time_data.devices !== undefined) {
            createDeviceObjectOnMap(time_data.devices);
            devicesLines(time_data.time_series_data);
        }
        if (time_data.devices !== undefined) {
            createBaloraObjectOnMap(time_data.baloras);
            balorasLines(time_data.time_series_data);
        }
    });
    return map;
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
    const path = [...pathObj];
    path.pop();

    const layerId = 'path_' + droneID;

    if (map.getLayer(layerId)) {
        map.removeLayer(layerId);
    }

    if (map.getSource(layerId)) {
        map.removeSource(layerId);
    }

    map.addSource(layerId, {
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

    const allDrones = get_all_drone_info_array();

    const layerOptions = {
        id: layerId,
        type: 'line',
        source: layerId,
        layout: {
            'line-join': 'round',
            'line-cap': 'round',
        },
        paint: {
            'line-color': getBasicColor(get_drone_index(droneID)),
            'line-width': 8,
        },
    };

    if (allDrones.length > 0) {
        map.addLayer(layerOptions, allDrones[0].droneModel.id);
    } else {
        map.addLayer(layerOptions);
    }

    map.flyTo({ center: [path[0][0], path[0][1]] });
}

temp_drones = [];
temp_devices = [];
temp_baloras = [];

function createDeviceObjectOnMap(device_list) {
    let new_device_names = [];
    let device_names_and_ids = [];
    device_list.forEach(function (device) {
        new_device_names.push(device.name);
        device_names_and_ids.push({ device_name: device.name, device_pk: device.id });
    });
    if (device_list.length > 0) {
        remove_no_devices_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
    }

    allDeviceInfoArray = add_new_device_to_array(new_device_names, device_names_and_ids);
    add_list_items_to_selected_device(new_device_names);
    add_list_items_to_map_tools_devices(new_device_names);
    add_list_items_to_trajectories_device(new_device_names);
    temp_devices = new_device_names;
}
function updateDevice(telemetry, DeviceObject) {
    DeviceObject.deviceInfo.currentBatteryLevel = telemetry.battery_percentage;
    DeviceObject.deviceInfo.time = telemetry.time;
    DeviceObject.deviceInfo.operator = telemetry.operator;

    if (DeviceObject.deviceInfo.currentCoordinate !== null) {
        DeviceObject.deviceInfo.previousCoordinate = DeviceObject.deviceInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    DeviceObject.deviceInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, 0];
    DeviceObject.deviceInfo.heading = telemetry.heading;
    DeviceObject.deviceInfo.altitude = 0;
    if (DeviceObject.deviceObject !== undefined) {
        if (temp_devices !== []) {
            addTooltipOnDevices(temp_devices);
            temp_devices = [];
        }
        DeviceObject.deviceObject.setCoords([
            DeviceObject.deviceInfo.currentCoordinate[0],
            DeviceObject.deviceInfo.currentCoordinate[1],
            DeviceObject.deviceInfo.currentCoordinate[2],
        ]);
        DeviceObject.deviceObject.setRotation(-telemetry.heading);
        if (document.getElementById('deviceInfoBox' + DeviceObject.deviceID)) {
            displayDeviceData(DeviceObject.deviceID);
        }
        if (isFirstTimeLoad && telemetry.lon !== 0 && telemetry.lat !== 0) {
            isFirstTimeLoad = false;
            map.flyTo({ center: DeviceObject.deviceInfo.currentCoordinate, zoom: 20 });
            remove_line_layers_from_map_device(get_all_device_info_array());
        }
    }
}
function devicesLines(time_series_data) {
    time_series_data
        .filter((singleDictionaryValue) => singleDictionaryValue.type == 'deviceTelemetry')
        .forEach((singleDeviceTelemetryValue) => {
            if (singleDeviceTelemetryValue.latitude != 0 && singleDeviceTelemetryValue.longitude != 0) {
                deviceObject = get_all_device_info_array().find((deviceObject) => deviceObject.deviceID === singleDeviceTelemetryValue.device);
                deviceObject.deviceInfo.currentCoordinate = [
                    singleDeviceTelemetryValue.longitude,
                    singleDeviceTelemetryValue.latitude,
                    // singleDeviceTelemetryValue.altitude,
                ];
                updateDeviceLineLayer(deviceObject, {
                    source: deviceObject.deviceInfo.previousCoordinate,
                    dest: deviceObject.deviceInfo.currentCoordinate,
                    color: [23, 184, 190],
                });
                deviceObject.deviceInfo.previousCoordinate = deviceObject.deviceInfo.currentCoordinate;
            }
        });
}

function createBaloraObjectOnMap(balora_list) {
    new_balora_names = [];
    balora_names_and_ids = [];
    balora_list.forEach(function (balora) {
        new_balora_names.push(balora.name);
        balora_names_and_ids.push({ balora_name: balora.name, balora_pk: balora.id });
    });
    if (balora_list.length > 0) {
        remove_no_baloras_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);
    }
    allBaloraInfoArray = add_new_balora_to_array(new_balora_names, balora_names_and_ids);
    add_list_items_to_selected_balora(new_balora_names);
    add_list_items_to_map_tools_baloras(new_balora_names);
    add_list_items_to_trajectories_balora(new_balora_names);
    temp_baloras = new_balora_names;
}
function updateBalora(telemetry, BaloraObject) {
    BaloraObject.baloraInfo.currentBatteryLevel = telemetry.battery_percentage;
    BaloraObject.baloraInfo.time = telemetry.time;
    BaloraObject.baloraInfo.receiver = telemetry.receiver;
    BaloraObject.baloraInfo.pm1 = telemetry.pm1;
    BaloraObject.baloraInfo.pm25 = telemetry.pm25;
    BaloraObject.baloraInfo.rssi = telemetry.received_signal_strength_indication;
    BaloraObject.baloraInfo.snr = telemetry.SignalToNoiseRatio;

    if (BaloraObject.baloraInfo.currentCoordinate !== null) {
        BaloraObject.baloraInfo.previousCoordinate = BaloraObject.baloraInfo.currentCoordinate; //Previous and current location needed for the line layer
    }

    BaloraObject.baloraInfo.currentCoordinate = [telemetry.longitude, telemetry.latitude, 0];
    BaloraObject.baloraInfo.altitude = 0;
    if (BaloraObject.baloraObject !== undefined) {
        if (temp_baloras !== []) {
            addTooltipOnBaloras(temp_baloras);
            temp_baloras = [];
        }
        BaloraObject.baloraObject.setCoords([
            BaloraObject.baloraInfo.currentCoordinate[0],
            BaloraObject.baloraInfo.currentCoordinate[1],
            BaloraObject.baloraInfo.currentCoordinate[2],
        ]);
        BaloraObject.baloraObject.setRotation(-telemetry.heading);
        if (document.getElementById('baloraInfoBox' + BaloraObject.baloraID)) {
            displayBaloraData(BaloraObject.baloraID);
        }
        if (isFirstTimeLoad && telemetry.lon !== 0 && telemetry.lat !== 0) {
            isFirstTimeLoad = false;
            map.flyTo({ center: BaloraObject.baloraInfo.currentCoordinate, zoom: 20 });
            remove_line_layers_from_map_balora(get_all_balora_info_array());
        }
    }
}
function balorasLines(time_series_data) {
    time_series_data
        .filter((singleDictionaryValue) => singleDictionaryValue.type == 'balora_telemetry')
        .forEach((singleBaloraTelemetryValue) => {
            if (singleBaloraTelemetryValue.latitude != 0 && singleBaloraTelemetryValue.longitude != 0) {
                baloraObject = get_all_balora_info_array().find((baloraObject) => baloraObject.baloraID === singleBaloraTelemetryValue.balora);
                baloraObject.baloraInfo.currentCoordinate = [singleBaloraTelemetryValue.longitude, singleBaloraTelemetryValue.latitude, 0];
                updateBaloraLineLayer(baloraObject, {
                    source: baloraObject.baloraInfo.previousCoordinate,
                    dest: baloraObject.baloraInfo.currentCoordinate,
                    color: [23, 184, 190],
                });
                baloraObject.baloraInfo.previousCoordinate = baloraObject.baloraInfo.currentCoordinate;
            }
        });
}
function format_timer(value) {
    return (value < 10 ? '0' : '') + value;
}
function formatTimeFromTimestampToHoursMinutesSeconds(timestamp) {
    return new Date(timestamp * 1000).toLocaleTimeString([], {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: false,
    });
}

function showCurrentTime() {
    timer_current = visualization_player.value - visualization_player.min;
    document.getElementById('timer').textContent = [
        String(Math.floor(timer_current / 3600)).padStart(2, '0'),
        String(Math.floor((timer_current % 3600) / 60)).padStart(2, '0'),
        String(Math.floor(timer_current % 60)).padStart(2, '0'),
    ].join(':');
    visualization_player.value = Number(visualization_player.value).toFixed(1).toString();
    if (visualization_player.previousValue === undefined || visualization_player.value > visualization_player.previousValue) {
        const previousValue = parseFloat(visualization_player.previousValue);
        const value = parseFloat(visualization_player.value);
        const precision = 10; // Number of decimal places
        for (let i = previousValue; i < value; i = Math.round((i + 0.1) * precision) / precision) {
            if (Math.round((i + 0.1) * precision) / precision == value.toFixed(1)) {
                visualizeDataOnSpecificTime(i.toFixed(1), 'forward', true);
            } else {
                visualizeDataOnSpecificTime(i.toFixed(1), 'forward');
            }
        }
    } else {
        const previousValue = parseFloat(visualization_player.previousValue);
        const value = parseFloat(visualization_player.value);
        const precision = 10; // Number of decimal places
        for (let i = value; i < previousValue; i = Math.round((i + 0.1) * precision) / precision) {
            if (Math.round((i - 0.1) * precision) / precision == value.toFixed(1)) {
                visualizeDataOnSpecificTime(i.toFixed(1), 'backward', true);
            } else {
                visualizeDataOnSpecificTime(i.toFixed(1), 'backward');
            }
        }
    }
    visualization_player.previousValue = visualization_player.value;
}
function updateMissionForward(timeDataRecord) {
    if (timeDataRecord.action == 'START_MISSION') {
        let waypoints = [];
        timeDataRecord.mission.mission_points.forEach(function (point, index) {
            point = [point[0], point[1]];
            waypoints.push(point);
        });
        if (timeDataRecord.mission.mission_type === 'SEARCH_AND_RESCUE_MISSION') {
            addLineLayerInIndex(waypoints, timeDataRecord.drone);
            let point_of_waypoints = [];
            point_of_waypoints.push(waypoints[0]);
            point_of_waypoints.push(waypoints[8]);
            point_of_waypoints.push(waypoints[81]);
            point_of_waypoints.push(waypoints[waypoints.length - 1]);
            place_waypoints_on_map(point_of_waypoints, timeDataRecord.drone);
        } else {
            place_waypoints_on_map(waypoints, timeDataRecord.drone);
        }
    } else if (timeDataRecord.action == 'FINISH_MISSION' || timeDataRecord.action == 'CANCEL_MISSION') {
        remove_drone_markers(timeDataRecord.drone);
        clearMissionPathLine(timeDataRecord.drone);
        get_all_drone_info_array().find((drone) => drone.droneID === timeDataRecord.drone).droneMarkerCounter = 0;
    }
}
function updateMissionBackward(timeDataRecord) {
    if (timeDataRecord.action == 'START_MISSION') {
        remove_drone_markers(timeDataRecord.drone);
        clearMissionPathLine(timeDataRecord.drone);
        get_all_drone_info_array().find((drone) => drone.droneID === timeDataRecord.drone).droneMarkerCounter = 0;
    } else if (timeDataRecord.action == 'FINISH_MISSION' || timeDataRecord.action == 'CANCEL_MISSION') {
        let waypoints = [];
        timeDataRecord.mission.mission_points.forEach(function (point, index) {
            point = [point[0], point[1]];
            waypoints.push(point);
        });
        if (timeDataRecord.mission.mission_type === 'SEARCH_AND_RESCUE_MISSION') {
            addLineLayerInIndex(waypoints, timeDataRecord.drone);
            let point_of_waypoints = [];
            point_of_waypoints.push(waypoints[0]);
            point_of_waypoints.push(waypoints[8]);
            point_of_waypoints.push(waypoints[81]);
            point_of_waypoints.push(waypoints[waypoints.length - 1]);
            place_waypoints_on_map(point_of_waypoints, timeDataRecord.drone);
        } else {
            place_waypoints_on_map(waypoints, timeDataRecord.drone);
        }
    }
}
function addNewRecordUserActions(time, user, button, action) {
    const table = document.getElementById('user-action-table');
    const newRow = table.insertRow();

    const timeCell = newRow.insertCell();
    const userCell = newRow.insertCell();
    const buttonCell = newRow.insertCell();
    const actionCell = newRow.insertCell();

    timeCell.innerHTML = time;
    userCell.textContent = user;
    buttonCell.textContent = button;
    actionCell.textContent = action;
}

function removeRecordUserActions() {
    var table = document.getElementById('user-action-table');
    var rows = table.rows;

    // Check if there are rows in the table
    if (rows.length > 1) {
        var lastRowIndex = rows.length - 1;
        var lastRow = rows[lastRowIndex];

        // Check if the last row is not the header row
        if (!lastRow.classList.contains('header-row')) {
            table.deleteRow(lastRowIndex);
        }
    }
}

function visualizeDataOnSpecificTime(time, type, lastValueToShowTelemetry = false) {
    time_data.time_series_data
        .filter((timeDataRecord) => timeDataRecord.time.toFixed(1) == Number(time).toFixed(1))
        .forEach((timeDataRecord) => {
            switch (timeDataRecord.type) {
                case 'droneTelemetry':
                    if (lastValueToShowTelemetry) {
                        tempTimeDataRecord = Object.assign({}, timeDataRecord);
                        tempTimeDataRecord.time = formatTimeFromTimestampToHoursMinutesSeconds(tempTimeDataRecord.time);
                        updateDroneObjectOnMap(
                            tempTimeDataRecord,
                            get_all_drone_info_array().find((drone) => drone.droneID === timeDataRecord.drone)
                        );
                    }
                    break;
                case 'droneVideoFrame':
                    webSocketMessage['drones'].find((droneObject) => droneObject.drone_name === timeDataRecord.drone)['video_frame_url'] =
                        timeDataRecord.frame;
                    break;
                case 'droneDetectionVideoFrame':
                    webSocketMessage['drones'].find((droneObject) => droneObject.drone_name === timeDataRecord.drone)['detected_frame_url'] =
                        timeDataRecord.frame;
                    webSocketMessage['drones'].find((droneObject) => droneObject.drone_name === timeDataRecord.drone)['detection'][
                        'detection_status'
                    ] = DETECTION_CONNECTED;
                    break;
                case 'droneVideoDetectionObject':
                    webSocketMessage['drones']
                        .find((droneObject) => droneObject.drone_name === timeDataRecord.drone)
                        ['detected_frame'].push(timeDataRecord);
                    break;
                case 'mission':
                    if (type == 'forward') {
                        updateMissionForward(timeDataRecord);
                    } else {
                        updateMissionBackward(timeDataRecord);
                    }
                    break;
                case 'droneBuildMapImage':
                    if (type == 'forward') {
                        if (!map.getSource(timeDataRecord.id)) {
                            map.addSource(timeDataRecord.id, {
                                type: 'image',
                                url: dutils.urls.resolve('build_map_image_path', { image_path: timeDataRecord.path }),
                                coordinates: [
                                    [timeDataRecord.top_right[0], timeDataRecord.top_right[1]],
                                    [timeDataRecord.bottom_right[0], timeDataRecord.bottom_right[1]],
                                    [timeDataRecord.bottom_left[0], timeDataRecord.bottom_left[1]],
                                    [timeDataRecord.top_left[0], timeDataRecord.top_left[1]],
                                ],
                            });
                            if (!map.getLayer(timeDataRecord.id)) {
                                let allDrones = get_all_drone_info_array();
                                if (allDrones.length > 0) {
                                    drone = allDrones[0].droneModel.id;
                                }
                                if (drone !== undefined) {
                                    map.addLayer(
                                        {
                                            id: timeDataRecord.id,
                                            type: 'raster',
                                            source: timeDataRecord.id,
                                            paint: { 'raster-opacity': 0.6 },
                                        },
                                        drone
                                    );
                                } else {
                                    map.addLayer({
                                        id: timeDataRecord.id,
                                        type: 'raster',
                                        source: timeDataRecord.id,
                                        paint: { 'raster-opacity': 0.6 },
                                    });
                                }
                            }
                        } else if (!map.getLayer(timeDataRecord.id)) {
                            let allDrones = get_all_drone_info_array();
                            if (allDrones.length > 0) {
                                drone = allDrones[0].droneModel.id;
                            }
                            if (drone !== undefined) {
                                map.addLayer(
                                    {
                                        id: timeDataRecord.id,
                                        type: 'raster',
                                        source: timeDataRecord.id,
                                        paint: { 'raster-opacity': 0.6 },
                                    },
                                    drone
                                );
                            } else {
                                map.addLayer({
                                    id: timeDataRecord.id,
                                    type: 'raster',
                                    source: timeDataRecord.id,
                                    paint: { 'raster-opacity': 0.6 },
                                });
                            }
                        }
                    } else {
                        if (map.getSource(timeDataRecord.id) && map.getLayer(timeDataRecord.id)) {
                            map.removeLayer(timeDataRecord.id);
                        }
                    }

                    break;
                case 'deviceTelemetry':
                    tempTimeDataRecord = Object.assign({}, timeDataRecord);
                    tempTimeDataRecord.time = formatTimeFromTimestampToHoursMinutesSeconds(tempTimeDataRecord.time);
                    updateDevice(
                        tempTimeDataRecord,
                        get_all_device_info_array().find((device) => device.deviceID === timeDataRecord.device)
                    );
                    break;
                case 'deviceImage':
                    if (type == 'forward') {
                        updateImageMarkers([timeDataRecord]);
                    } else {
                        removeDeviceImageMarker(timeDataRecord);
                    }
                    break;
                case 'baloraTelemetry':
                    tempTimeDataRecord = Object.assign({}, timeDataRecord);
                    tempTimeDataRecord.time = formatTimeFromTimestampToHoursMinutesSeconds(tempTimeDataRecord.time);
                    updateBalora(
                        tempTimeDataRecord,
                        get_all_balora_info_array().find((balora) => balora.baloraID === timeDataRecord.balora)
                    );
                    break;
                case 'algorithm':
                    if (type == 'forward') {
                        if (timeDataRecord.algorithm_name == 'FIRE_PROPAGATION_ALGORITHM') {
                            RunFireSpreadOutput((fireObject = { ['fields']: timeDataRecord }));
                        }
                    } else {
                        if (timeDataRecord.algorithm_name == 'FIRE_PROPAGATION_ALGORITHM') {
                            RemoveFireSpreadOutput(timeDataRecord.id);
                        }
                    }
                    break;
                case 'weatherDrone': {
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'wDir').textContent = timeDataRecord.wind_direction + ' \u00B0';
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'wSpeed').textContent = timeDataRecord.wind_speed + ' m/s';
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'temp').textContent = timeDataRecord.temperature + ' \u2103';
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'press').textContent = timeDataRecord.pressure + ' Pa';
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'humidity').innerHTML = timeDataRecord.humidity + ' g.m<sup>-</sup><sup>3</sup>';
                    document.getElementById('weatherDataBox'+timeDataRecord.drone + 'heading').textContent = timeDataRecord.heading + ' \u00B0';
                    break;
                }
                case 'weatherStation':
                    tempTimeDataRecord = Object.assign({}, timeDataRecord);
                    tempTimeDataRecord.time = formatTimeFromTimestampToHoursMinutesSeconds(tempTimeDataRecord.time);
                    webSocketMessage['weather_station'] = tempTimeDataRecord;
                    break;
                case 'errorMessage':
                    create_popup_for_a_little(WARNING_ALERT, timeDataRecord['message'], 4000);
                    break;
                case 'userInput':
                    if (type == 'forward') {
                        timer_current = timeDataRecord.time - visualization_player.min;
                        timer_current = [
                            String(Math.floor(timer_current / 3600)).padStart(2, '0'),
                            String(Math.floor((timer_current % 3600) / 60)).padStart(2, '0'),
                            String(Math.floor(timer_current % 60)).padStart(2, '0'),
                        ].join(':');
                        buttonAction = timeDataRecord.value;
                        if (timeDataRecord.value == 'true') {
                            buttonAction = 'On';
                        } else if (timeDataRecord.value == 'false') {
                            buttonAction = 'Off';
                        }
                        addNewRecordUserActions(timer_current, timeDataRecord.user, timeDataRecord.element_name, buttonAction);
                    } else {
                        removeRecordUserActions();
                    }
                    break;
                default:
                    console.log(timeDataRecord);
                    break;
            }
        });
}

function initialize_max_time() {
    max_timer = visualization_player.max - visualization_player.min;
    document.getElementById('max-timer').innerHTML = [Math.floor(max_timer / 3600), Math.floor((max_timer % 3600) / 60), max_timer % 60]
        .map(format_timer)
        .join(':');
}
function startPlayback() {
    document.getElementById('main-controller').innerHTML = '<i class="fa-solid fa-pause"></i>';
    intervalId = setInterval(function () {
        visualization_player.value = Number(visualization_player.value) + 0.1;
        if (visualization_player.value >= visualization_player.max) {
            if ('true' == document.getElementById('replayBarRepeat').value) {
                visualization_player.value = visualization_player.min;
            } else {
                pausePlayback();
            }
        }
        visualization_player.dispatchEvent(new Event('input'));
    }, 100 / document.getElementById('replayBarSpeed').value);
}
function pausePlayback() {
    document.getElementById('main-controller').innerHTML = '<i class="fa-solid fa-play"></i>';
    clearInterval(intervalId);
    intervalId = null;
}

function play_pause(playPauseButtonElement) {
    if (playPauseButtonElement.value == 'play') {
        startPlayback();
        playPauseButtonElement.value = 'pause';
    } else {
        pausePlayback();
        playPauseButtonElement.value = 'play';
    }
}
function fast_forward() {
    visualization_player.value = Number(visualization_player.value) + 5;
    visualization_player.dispatchEvent(new Event('input'));
}
function fast_forward_x2() {
    visualization_player.value = Number(visualization_player.value) + 10;
    visualization_player.dispatchEvent(new Event('input'));
}
function rewind() {
    visualization_player.value = Number(visualization_player.value) - 5;
    visualization_player.dispatchEvent(new Event('input'));
}
function rewind_x2() {
    visualization_player.value = Number(visualization_player.value) - 10;
    visualization_player.dispatchEvent(new Event('input'));
}
function changeReplaySpeed() {
    play_pause(document.getElementById('main-controller'));
    play_pause(document.getElementById('main-controller'));
}
function initialize_replay_bar() {
    visualization_player.min = start_time;
    visualization_player.max = end_time;
    visualization_player.addEventListener('input', function () {
        showCurrentTime();
    });
    initialize_max_time();
    startPlayback();
}

map = main_map();
