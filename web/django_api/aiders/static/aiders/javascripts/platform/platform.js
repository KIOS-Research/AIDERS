var previousDroneNumber = 0;
var previousDroneNames = [];
var allDroneInfoArray = [];
// var web_rtc_port= 8889; // Port for WebRTC streaming

var previousDeviceNumber = 0;
var allDeviceInfoArray = [];


var previousBaloraNumber = 0;
var allBaloraInfoArray = [];


var altitudeSafetyDistance = 10;

// Global variable to track if clearAllHeatmaps has been called
var isHeatmapCleared = false;

const wsInterval = 400; // in milliseconds
// const wsInterval = 1000; // for checking exernal computer vision

// const wsAddress = 'ws://' + window.location.host + '/' + dutils.urls.resolve('ws_platform'); // django ws
const wsAddress = 'ws://' + window.location.hostname + ':' + NGINX_PORT + "/ws/"; // go ws
// const wsAddress = 'wss://' + window.location.hostname + "/ws/"; // go ws

let g_websocketMessage;
let g_websocket = initWebsocket(wsAddress);



map.on('load', function () {

    // side-panel initialization actions
    add_no_devices_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES);
    add_no_baloras_text_on_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);
    // Hide side panel elements based on user permissions
    hideElementBaseOnUserPermission("#monitor-points-list")
    hideElementBaseOnUserPermission("#uav-build-map-list")

    setInterval(sendWebsocketMessage, wsInterval); // init websocket data timer

    map.on('SelectedFeatureChange', onSelectedFeatureChange);
    createBooMarker(map);
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
        console.log('Telemetry WebSocket connection established.');
    });
    ws.addEventListener('close', function (event) {
        console.log('Telemetry Socket is closed. Reconnect will be attempted in 1 second.');
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
        const jsonData = {
            operation_id: parseInt(OPERATION_ID),
        };
        g_websocket.send(JSON.stringify(jsonData));
    }
}


// triggered when a websocket message is received
function handleIncomingWebsocketMessage(_wsMessage) {
    //console.log(JSON.stringify(_wsMessage));
    if ('devices' in _wsMessage) {  // if the ws message includes devices then do the full update procedure
        if (_wsMessage['error_msg'].length !== 0) {
            create_popup_for_a_little(WARNING_ALERT, _wsMessage['error_msg'], 10000);
        }
        updateDronesFromWebsocketMessage(_wsMessage['drones']);

        deviceUpdate(_wsMessage['devices']);
        baloraUpdate(_wsMessage['baloras']);
        handleStaticCamerasUpdate(_wsMessage['static_cameras']); // static_cameras.js

    }
    // updateStreamFrames(_wsMessage['drones']);   // update the stream frames
}


// const fovLineLayers = {};

// // update the drone's field of view polygons
// function updateDroneFOVPolygons(drones) {
//     drones.forEach(function (drone) {
//         let droneName = drone['drone_name'];

//         let localDrone = get_drone_object(droneName);

//         if (typeof (localDrone.droneObject) === "undefined") {
//             console.log("no local drone object");
//             return;
//         }

//         // hide layers if drone not selected
//         if (!localDrone.droneObject.selected) {
//             if (map.getLayer("fov-polygon-layer-" + droneName)) {
//                 map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'none');
//                 map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'none');
//                 map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'none');
//             }
//             return;
//         }

//         // make layers visible
//         if (map.getLayer("fov-polygon-layer-" + droneName)) {
//             map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'visible');
//             map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'visible');
//             map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'visible');
//         }

//         var fov_coordinates = JSON.parse(drone["telemetry"]["fov_coordinates"]);
//         var fov_points = [];
//         if (fov_coordinates == null) {
//             return;
//         }
//         fov_coordinates.forEach(function (point, i) {
//             fov_points.push([point[1], point[0]]);
//         });

//         var newData_fov = {
//             "type": "Feature",
//             "geometry": {
//                 "type": "Polygon",
//                 "coordinates": [fov_points.concat([fov_points[0]])] // Ensure the polygon is closed
//             }
//         };

//         // Check if the source exists before setting data
//         if (map.getSource("fov-polygon-" + droneName)) {
//             map.getSource("fov-polygon-" + droneName).setData(newData_fov);
//             // calculate the center of the polygon
//             let sumLat = 0, sumLon = 0;
//             for (let i = 0; i < fov_coordinates.length; i++) {
//                 sumLat += fov_coordinates[i][0];
//                 sumLon += fov_coordinates[i][1];
//             }
//             let centerLat = sumLat / fov_coordinates.length;
//             let centerLon = sumLon / fov_coordinates.length;
//             // update the view direction line
//             fovLineLayers[droneName].setProps({
//                 data: [
//                     {
//                         source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
//                         dest: [fov_coordinates[0][1], fov_coordinates[0][0], fov_coordinates[0][2]]
//                     },
//                     {
//                         source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
//                         dest: [fov_coordinates[1][1], fov_coordinates[1][0], fov_coordinates[1][2]]
//                     },
//                     {
//                         source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
//                         dest: [fov_coordinates[2][1], fov_coordinates[2][0], fov_coordinates[2][2]]
//                     },
//                     {
//                         source: [drone["telemetry"]["lon"], drone["telemetry"]["lat"], drone["telemetry"]["alt"]],
//                         dest: [fov_coordinates[3][1], fov_coordinates[3][0], fov_coordinates[3][2]]
//                     },
//                 ]
//             });


//         } else {

//             var test = new MapboxLayer({
//                 id: droneName + '-to-fov-line',
//                 type: LineLayer,
//                 data: [],
//                 fp64: false,
//                 widthScale: 0.1,
//                 getWidth: 15, //Change getWidth and widthScale to fine-tune the line width
//                 opacity: 0.2,
//                 widthUnit: 'meters',
//                 // getStrokeWidth: 6,
//                 getSourcePosition: (d) => d.source,
//                 getTargetPosition: (d) => d.dest,
//                 getColor: [250, 50, 50],
//             });
//             map.addLayer(test);
//             fovLineLayers[droneName] = test;


//             // If the source doesn't exist, add it to the map
//             map.addSource("fov-polygon-" + droneName, {
//                 "type": "geojson",
//                 "data": newData_fov
//             });

//             // Add a layer for the polygon
//             map.addLayer({
//                 "id": "fov-polygon-layer-" + droneName,
//                 "type": "fill",
//                 "source": "fov-polygon-" + droneName,
//                 "layout": {},
//                 "paint": {
//                     "fill-color": "#888888",
//                     "fill-opacity": 0.2
//                 }
//             });
//             // add an outline layer
//             map.addLayer({
//                 "id": "fov-polygon-outline-" + droneName,
//                 "type": "line",
//                 "source": "fov-polygon-" + droneName,
//                 "paint": {
//                     "line-color": "#ff5555", // Set the outline color here
//                     "line-width": 2,
//                     "line-opacity": 0.6
//                 }
//             });
//         }
//     });
// } // end of updateDroneFOVPolygons()




var temp_new_drone_ids = [];
var tooltipAddedDrone = false;
let pan_to_drone_only_once = true;

/**
 * Starts the process of updating the drone's attributes and/or layers (e.g the drone's position, the drone's line layer etc.)
 * */
function updateDronesFromWebsocketMessage(_wsDrones) {
    var currentDroneNumber = _wsDrones.length;
    let droneCountDifference = currentDroneNumber - previousDroneNumber;

    // No drones currently connected
    if (droneCountDifference == 0 && previousDroneNumber === 0) {
        return;
    }
    // Drones number increased
    if (droneCountDifference > 0) { // drones connected
        // Initial Drones Connected
        if (previousDroneNumber === 0) {
            document.getElementById("drones-menu").style.display = "block";  // show sidebar section for drones
            console.log("Initial Drone Connection");
        }
        // Add connected drones
        // Drones Connected
        handleConnectedDrones(_wsDrones)
    }
    // Drones number decreased
    else if (droneCountDifference < 0) { // drones disconnected
        // Drones Disconnected
        handleDisconnectedDrones(_wsDrones)
        console.log("Drones Disconnected");
        // Final Drones Disconnected
        if (Math.abs(droneCountDifference) == previousDroneNumber) {
            document.getElementById("drones-menu").style.display = "none";  // hide sidebar section for drones
            console.log("All Drones Disconnected");
            previousDroneNumber = currentDroneNumber;
            return;
        }
    }
    // Update Drones
    document.getElementById("connected-drones").innerHTML = currentDroneNumber; // show number of drones on sidebar
    handleUpdatedDrones(_wsDrones)
    previousDroneNumber = currentDroneNumber;
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

    var allModelsLoaded = allDeviceModelsLoaded(allDeviceInfoArray);

    if (allModelsLoaded) {
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

    if (allModelsLoaded && !tooltipAddedDevice) {
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

let lastProcessedTelemetryKey = null; // Variable to track the last processed telemetry

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

        var new_index = 0;

        if (currentBaloraNumber - previousBaloraNumber === 0 || previousBaloraNumber === 0) {
            //If the number of previous balora was zero (e.g at the beginning), then just overwrite
            new_balora_names = balora_names;
        } //Otherwise, get the ids of newly connected balora
        else {
            new_index = previousBaloraNumber;

            new_balora_names = find_difference_on_two_arrays(balora_names, previous_balora_ids);
        }

        console.log("NEW BALORA NAMES:", new_balora_names);

        remove_no_baloras_text_from_panel_sections(ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS);
        allBaloraInfoArray = add_new_balora_to_array(new_balora_names, balora_names_and_ids);
        addNewBalorasToSidePanel(new_balora_names);
        console.log("BALORAS:", allBaloraInfoArray);
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

    var allModelsLoaded = allBaloraModelsLoaded(allBaloraInfoArray);

    if (allModelsLoaded) {
        for (let i = 0; i < allBaloraInfoArray.length; i++) {
            if (allBaloraInfoArray[i].baloraAvailability === AVAILABLE) {
                _wsBaloras.forEach(function (balora) {
                    if (balora['name'] == allBaloraInfoArray[i].baloraID) {
                        if (balora['telemetry'] === null) {
                            return;
                        }
                        const currentTelemetryKey = `${balora['telemetry'].latitude},${balora['telemetry'].longitude},${balora['telemetry'].time}`;
                        
                        // const randomCoordinates = generateLatLongAroundOrigin(1); // Generate within 1 degree of 0,0
                        // balora['telemetry'].latitude = randomCoordinates.latitude;
                        // balora['telemetry'].longitude = randomCoordinates.longitude;
             
                        if (lastProcessedTelemetryKey !== currentTelemetryKey) {
                            if (
                                balora['telemetry'].pm25 !== undefined && balora['telemetry'].pm25 !== null &&
                                document.getElementById('balora-pm25-radio').checked
                            ) {
                                updateHeatmapLayerPM25(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].pm25);
                                //updateHeatmapLayerPM25(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].pm25, balora['name']);
                            }
    
                            if (
                                balora['telemetry'].pm1 !== undefined && balora['telemetry'].pm1 !== null &&
                                document.getElementById('balora-pm1-radio').checked
                            ) {
    
                                updateHeatmapLayerPM1(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].pm1);
                                //updateHeatmapLayerPM1(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].pm1, balora['name']);
                            }
    
                            if (
                                balora['telemetry'].nox !== undefined && balora['telemetry'].nox !== null &&
                                document.getElementById('balora-nox-radio').checked
                            ) {
    
                                updateHeatmapLayerNOx(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].nox);
                                //updateHeatmapLayerNOx(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].nox, balora['name']);
                            }
    
                            if (
                                balora['telemetry'].voc !== undefined && balora['telemetry'].voc !== null &&
                                document.getElementById('balora-voc-radio').checked
                            ) {
    
                                updateHeatmapLayerVOC(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].voc);
                                //updateHeatmapLayerVOC(balora['telemetry'].longitude, balora['telemetry'].latitude, balora['telemetry'].voc, balora['name']);
                            }
                        }
                        

                        lastProcessedTelemetryKey = currentTelemetryKey;

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

    if (allModelsLoaded && !tooltipAddedBalora) {
        //Add a tooltip to the 3d balora models only if they were all successfully loaded otherwise we get an error
        tooltipAddedBalora = true;
        addTooltipOnBaloras(temp_new_balora_ids);
    }

    balora_iter_index++;
}

function clearAllHeatmaps() {
    // Get all layers and sources dynamically
    const allLayers = map.getStyle().layers; // Fetch all layers
    const allSources = Object.keys(map.style.sourceCaches); // Fetch all sources

    // Remove all heatmap-related layers
    allLayers.forEach(layer => {
        if (layer.id.includes('heat') || layer.id.includes('circles')) {
            map.removeLayer(layer.id);
        }
    });

    // Remove all heatmap-related sources
    allSources.forEach(sourceId => {
        if (sourceId.includes('balora')) {
            map.removeSource(sourceId);
        }
    });

    isHeatmapCleared = true;
    console.log('All heatmaps cleared dynamically.');
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
    currentBalora.baloraInfo.humidity = telemetry.humidity;
    currentBalora.baloraInfo.temp = telemetry.temp;
    currentBalora.baloraInfo.voc = telemetry.voc;
    currentBalora.baloraInfo.nox = telemetry.nox;
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

function generateLatLongAroundOrigin(maxDistanceDegrees = 1) {
    const distance = Math.random() * maxDistanceDegrees; // Random distance within the range
    return {
        latitude: distance,  // Latitude progresses along the line
        longitude: distance  // Longitude progresses along the same line (45-degree diagonal)
    };
}

function generateRandomPM25(min = 0, max = 500) {
    return parseFloat((Math.random() * (max - min) + min).toFixed(2));
}

