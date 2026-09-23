function baloraClickInfoBox(toggleId, baloraId) {
    postElementId('GPS ' + baloraId, document.getElementById(toggleId).checked);
    if (document.getElementById(toggleId).checked) {
        let div = document.createElement('div');
        div.id = 'baloraInfoBox' + baloraId;
        div.classList.add('baloraDataBox');
        div.classList.add('overlay-popup');

        div.style.position = 'absolute';
        let offset = getNumberOfOverlayPanels() * 20;
        div.style.top = offset + 'px';
        div.style.left = offset + 'px';

        document.getElementsByClassName('overlay-section')[0].appendChild(div);
        div.innerHTML =
            '<div><b> Balora: ' +
            baloraId +
            '</b></div>' +
            "<div><b>Receiver: </b><span id='" +
            baloraId +
            "_receiver'></span></div>" +
            "<div><b>Time: </b><span id='" +
            baloraId +
            "_time'></span></div>" +
            "<div><b>Latitude: </b><span id='" +
            baloraId +
            "_latitude'></span></div>" +
            "<div><b>Longitude: </b><span id='" +
            baloraId +
            "_longitude'></span></div>" +
            "<div><b>PM1: </b><span id='" +
            baloraId +
            "_pm1'></span></div>" +
            "<div><b>PM2.5: </b><span id='" +
            baloraId +
            "_pm25'></span></div>" +
            "<div><b>NOx: </b><span id='" +
            baloraId +
            "_nox'></span></div>" +
            "<div><b>VOC: </b><span id='" +
            baloraId +
            "_voc'></span></div>" +
            "<div><b>Temperature: </b><span id='" +
            baloraId +
            "_temp'></span></div>" +
            "<div><b>Humidity: </b><span id='" +
            baloraId +
            "_humidity'></span></div>" +
            "<div><b>RSSI: </b><img id='" +
            baloraId +
            "_rssi' style='height: 30px;'></img></div>" +
            "<div><b>SNR: </b><img id='" +
            baloraId +
            "_snr' style='height: 40px;'></img></div>";
        jQuery(div).draggable();
    } else {
        document.getElementById('baloraInfoBox' + baloraId).remove();
    }
}
function displayBaloraData(baloraId) {

    let timestamp = get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.time;
    let date = new Date(timestamp);
    let formattedTime = date.getHours().toString().padStart(2, '0') + ":" +
        date.getMinutes().toString().padStart(2, '0') + ":" +
        date.getSeconds().toString().padStart(2, '0');
    document.getElementById(baloraId + '_time').textContent = formattedTime

    document.getElementById(baloraId + '_receiver').innerHTML = get_all_balora_info_array().find(
        (balora) => balora.baloraID === baloraId
    ).baloraInfo.receiver;
    document.getElementById(baloraId + '_latitude').innerHTML =
        parseFloat(get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.currentCoordinate[1]).toFixed(6) + '°';
    document.getElementById(baloraId + '_longitude').innerHTML =
        parseFloat(get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.currentCoordinate[0]).toFixed(6) + '°';

    document.getElementById(baloraId + '_pm1').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.pm1
    ).toFixed(2);
    document.getElementById(baloraId + '_pm25').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.pm25
    ).toFixed(2);
    document.getElementById(baloraId + '_nox').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.nox
    ).toFixed(2);
    document.getElementById(baloraId + '_voc').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.voc
    ).toFixed(2);;
    document.getElementById(baloraId + '_temp').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.temp
    ).toFixed(2);
    document.getElementById(baloraId + '_humidity').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.humidity
    ).toFixed(2);;
    rssi = displayRSSI(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.rssi,
        document.getElementById(baloraId + '_rssi').src
    );
    if (rssi != null) {
        document.getElementById(baloraId + '_rssi').src = rssi;
    }
    snr = displaySNR(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.snr,
        document.getElementById(baloraId + '_snr').src
    );
    // console.log(snr);
    if (snr != null) {
        document.getElementById(baloraId + '_snr').src = snr;
    }

    function displayRSSI(rssi, value) {
        if (
            rssi > -67 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal4.png',
            })
        ) {
            // return '****';
            return dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal4.png',
            }); // Full signal strength
        } else if (
            rssi > -70 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal3.png',
            })
        ) {
            // return '*** ';
            return dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal3.png',
            }); // Three bars
        } else if (
            rssi > -80 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal2.png',
            })
        ) {
            // return '**  ';
            return dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal2.png',
            }); // Two bars
        } else if (
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal1.png',
            })
        ) {
            // return '*   ';
            return dutils.urls.resolve('static_images', {
                file_name: 'wifi_signal1.png',
            }); // One bar
        } else {
            return null;
        }
    }
    function displaySNR(snr, value) {
        if (
            snr > 20 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'signal_noise4.png',
            })
        ) {
            // return '****';
            return dutils.urls.resolve('static_images', {
                file_name: 'signal_noise4.png',
            }); // Full signal strength
        } else if (
            snr > 15 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'signal_noise3.png',
            })
        ) {
            // return '*** ';
            return dutils.urls.resolve('static_images', {
                file_name: 'signal_noise3.png',
            }); // Three bars
        } else if (
            snr > 10 &&
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'signal_noise2.png',
            })
        ) {
            // return '**  ';
            return dutils.urls.resolve('static_images', {
                file_name: 'signal_noise2.png',
            }); // Two bars
        } else if (
            value !==
            dutils.urls.resolve('static_images', {
                file_name: 'signal_noise1.png',
            })
        ) {
            // return '*   ';
            return dutils.urls.resolve('static_images', {
                file_name: 'signal_noise1.png',
            }); // One bar
        } else {
            return null;
        }
    }
}

// Variables to track the previous position and time for each sensor
let sensorTracking = {
    pm25: { lat: null, lon: null, timestamp: null },
    pm1: { lat: null, lon: null, timestamp: null },
    nox: { lat: null, lon: null, timestamp: null },
    voc: { lat: null, lon: null, timestamp: null },
};

// Tracks the current active sensor
let currentActiveSensor = null;

// Track the active popup
let activePopup = null; 




// Updates the heatmap layer for PM2.5 sensor 
function updateHeatmapLayerPM25(longitude, latitude, intensity, replace = true) {
    const currentTimeSeconds = Math.floor(Date.now() / 1000);
    const activeSensor = 'pm25';

    // Switch to the new sensor if it's not the current one
    if (currentActiveSensor !== activeSensor) {
        console.log(`[PM2.5] New sensor selected: ${activeSensor}, clearing heatmap.`);
        clearAllHeatmaps(); // Clear the heatmap when switching sensors
        currentActiveSensor = activeSensor; // Update the active sensor
        isHeatmapCleared = true; // Mark heatmap as cleared
    }

    // If a previous position exists, calculate distance and time difference
    if (sensorTracking.pm25.lat !== null && sensorTracking.pm25.lon !== null) {
        const distance = calculateDistance(sensorTracking.pm25.lat, sensorTracking.pm25.lon, latitude, longitude);
        const timeDifferenceSeconds = currentTimeSeconds - sensorTracking.pm25.timestamp; // Time difference in seconds

        console.log("Time Difference (seconds):", timeDifferenceSeconds);
        console.log("Distance (m):", distance);

        // If the sensor is hovering (distance < 1 meter and time > 10 seconds)
        if (distance < 1 && timeDifferenceSeconds > 10 && !isHeatmapCleared) {
            console.log("[PM2.5] Sensor hovering for more than 10 seconds.");

            // Initialize history if not already initialized
            if (!sensorTracking.pm25.history) {
                sensorTracking.pm25.history = [];
            }

            // Add intensity to history and calculate the running average
            sensorTracking.pm25.history.push(intensity);
            if (sensorTracking.pm25.history.length > 10) {
                sensorTracking.pm25.history.shift(); // Remove oldest value if history exceeds 10
            }
            const runningAverage = sensorTracking.pm25.history.reduce((sum, val) => sum + val, 0) / sensorTracking.pm25.history.length;
            intensity = runningAverage;

            // Update the existing GeoJSON feature with the new intensity
            const dataId = 'balora_pm25_data';
            if (map.getSource(dataId)) {
                const matchingData = map.getSource(dataId)._data.features.find((data) => {
                    return (
                        data.geometry.type === 'Point' &&
                        Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                        Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
                    );
                });

                if (matchingData) {
                    matchingData.properties.intensity = intensity; // Update intensity
                    map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
                }
            }

            console.log("Last 10 values:", sensorTracking.pm25.history.slice(-10));
            return; // Exit to prevent adding a new feature
        }

        // Only update the timestamp if the sensor has moved significantly
        if (distance >= 1) {
            sensorTracking.pm25 = {
                lat: latitude,
                lon: longitude,
                timestamp: currentTimeSeconds, // Update timestamp only when the sensor moves
                history: sensorTracking.pm25.history || [], // Preserve existing history
            };
        }
    } else {
        // Initialize tracking data if no previous data exists
        sensorTracking.pm25 = {
            lat: latitude,
            lon: longitude,
            timestamp: currentTimeSeconds,
            history: [],
        };
    }

    isHeatmapCleared = false;

    // Add or update the heatmap layer
    const dataId = 'balora_pm25_data';
    const layerId = 'balora_pm25_heat_layer';
    const circleLayerId = 'balora_pm25_circles_layer';

    if (map.getSource(dataId)) {
        const matchingData = map.getSource(dataId)._data.features.find((data) => {
            return (
                data.geometry.type === 'Point' &&
                Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
            );
        });

        if (matchingData) {
            if (replace) {
                matchingData.properties.intensity = intensity; // Update the intensity
                map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
            }
        } else {
            map.getSource(dataId).setData({
                type: 'FeatureCollection',
                features: map.getSource(dataId)._data.features.concat([
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ]),
            });
        }
    } else {
        // Create a new source and layer if it doesn't exist
        map.addSource(dataId, {
            type: 'geojson',
            data: {
                type: 'FeatureCollection',
                features: [
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ],
            },
        });

        map.addLayer({
            id: layerId,
            type: 'heatmap',
            source: dataId,
            paint: {
                'heatmap-weight': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [1, 0],
                        [250, 1],
                    ],
                },
                'heatmap-color': [
                    'interpolate',
                    ['linear'],
                    ['heatmap-density'],
                    0,
                    'rgba(236,222,239,0)',
                    0.2,
                    'rgb(255, 255, 0)',
                    0.4,
                    'rgb(255, 165, 0)',
                    0.6,
                    'rgb(255, 0, 0)',
                    0.8,
                    'rgb(128, 0, 128)',
                    1,
                    'rgb(0, 0, 0)',
                ],
                'heatmap-radius': {
                    stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
                },
                'heatmap-opacity': {
                    default: 1,
                    stops: [
                        [17, 1],
                        [18, 0],
                    ],
                },
            },
        });

        map.addLayer({
            id: circleLayerId,
            type: 'circle',
            source: dataId,
            paint: {
                'circle-radius': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [0, 15],
                        [250, 30],
                    ],
                },
                'circle-color': {
                    property: 'intensity',
                    type: 'interval',
                    stops: [
                        [0, 'rgb(0, 255, 0)'],
                        [12, 'rgb(255, 255, 0)'],
                        [35, 'rgb(255, 165, 0)'],
                        [55, 'rgb(255, 0, 0)'],
                        [150, 'rgb(128, 0, 128)'],
                        [250, 'rgb(0, 0, 0)'],
                    ],
                },
                'circle-opacity': 0.8,
            },
        });

        map.on('click', circleLayerId, (event) => {
            const coords = event.features[0].geometry.coordinates;
            const props = event.features[0].properties;
            activePopup = new maplibregl.Popup()
                .setLngLat(coords)
                .setHTML(`<strong>PM2.5:</strong>  ${props.intensity.toFixed(2)}`)
                .addTo(map);
        });
    }
}



// Updates the heatmap layer for PM1 sensor with distance and time logic
function updateHeatmapLayerPM1(longitude, latitude, intensity, replace = true) {
    const currentTimeSeconds = Math.floor(Date.now() / 1000);
    const activeSensor = 'pm1';

    // Switch to the new sensor if it's not the current one
    if (currentActiveSensor !== activeSensor) {
        console.log(`[PM1] New sensor selected: ${activeSensor}, clearing heatmap.`);
        clearAllHeatmaps(); // Clear the heatmap when switching sensors
        currentActiveSensor = activeSensor; // Update the active sensor
        isHeatmapCleared = true; // Mark heatmap as cleared
    }

    // Ensure history is initialized
    if (!sensorTracking.pm1.history) {
        sensorTracking.pm1.history = [];
    }

    // If a previous position exists, calculate distance and time difference
    if (sensorTracking.pm1.lat !== null && sensorTracking.pm1.lon !== null) {
        const distance = calculateDistance(sensorTracking.pm1.lat, sensorTracking.pm1.lon, latitude, longitude);
        const timeDifferenceSeconds = currentTimeSeconds - sensorTracking.pm1.timestamp;

        console.log("Time Difference (seconds):", timeDifferenceSeconds);
        console.log("Distance (m):", distance);

        // If the sensor is hovering (distance < 1 meter and time > 10 seconds)
        if (distance < 1 && timeDifferenceSeconds > 10 && !isHeatmapCleared) {
            console.log("[PM1] Sensor hovering for more than 10 seconds.");

            // Add intensity to history and calculate the running average
            sensorTracking.pm1.history.push(intensity);
            if (sensorTracking.pm1.history.length > 10) {
                sensorTracking.pm1.history.shift(); // Remove oldest value if history exceeds 10
            }
            const runningAverage = sensorTracking.pm1.history.reduce((sum, val) => sum + val, 0) / sensorTracking.pm1.history.length;
            intensity = runningAverage;

            // Update the existing GeoJSON feature with the new intensity
            const dataId = 'balora_pm1_data';
            if (map.getSource(dataId)) {
                const matchingData = map.getSource(dataId)._data.features.find((data) => {
                    return (
                        data.geometry.type === 'Point' &&
                        Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                        Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
                    );
                });

                if (matchingData) {
                    matchingData.properties.intensity = intensity; // Update intensity
                    map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
                }
            }

            console.log("Last 10 values:", sensorTracking.pm1.history.slice(-10));
            return; // Exit to prevent adding a new feature
        }

        // Only update the timestamp if the sensor has moved significantly
        if (distance >= 1) {
            sensorTracking.pm1 = {
                lat: latitude,
                lon: longitude,
                timestamp: currentTimeSeconds, // Update timestamp only when the sensor moves
                history: sensorTracking.pm1.history || [], // Preserve existing history
            };
        }
    } else {
        // Initialize tracking data if no previous data exists
        sensorTracking.pm1 = {
            lat: latitude,
            lon: longitude,
            timestamp: currentTimeSeconds,
            history: [],
        };
    }

    isHeatmapCleared = false;

    // Add or update the heatmap layer
    const dataId = 'balora_pm1_data';
    const layerId = 'balora_pm1_heat_layer';
    const circleLayerId = 'balora_pm1_circles_layer';

    if (map.getSource(dataId)) {
        const matchingData = map.getSource(dataId)._data.features.find((data) => {
            return (
                data.geometry.type === 'Point' &&
                Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
            );
        });

        if (matchingData) {
            if (replace) {
                matchingData.properties.intensity = intensity; // Update the intensity
                map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
            }
        } else {
            map.getSource(dataId).setData({
                type: 'FeatureCollection',
                features: map.getSource(dataId)._data.features.concat([
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ]),
            });
        }
    } else {
        // Create a new source and layer if it doesn't exist
        map.addSource(dataId, {
            type: 'geojson',
            data: {
                type: 'FeatureCollection',
                features: [
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ],
            },
        });

        map.addLayer({
            id: layerId,
            type: 'heatmap',
            source: dataId,
            paint: {
                'heatmap-weight': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [1, 0],
                        [250, 1],
                    ],
                },
                'heatmap-color': [
                    'interpolate',
                    ['linear'],
                    ['heatmap-density'],
                    0,
                    'rgba(236,222,239,0)',
                    0.2,
                    'rgb(255, 255, 0)',
                    0.4,
                    'rgb(255, 165, 0)',
                    0.6,
                    'rgb(255, 0, 0)',
                    0.8,
                    'rgb(128, 0, 128)',
                    1,
                    'rgb(0, 0, 0)',
                ],
                'heatmap-radius': {
                    stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
                },
                'heatmap-opacity': {
                    default: 1,
                    stops: [
                        [17, 1],
                        [18, 0],
                    ],
                },
            },
        });

        map.addLayer({
            id: circleLayerId,
            type: 'circle',
            source: dataId,
            paint: {
                'circle-radius': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [0, 15],
                        [250, 30],
                    ],
                },
                'circle-color': {
                    property: 'intensity',
                    type: 'interval',
                    stops: [
                        [0, 'rgb(0, 255, 0)'],
                        [12, 'rgb(255, 255, 0)'],
                        [35, 'rgb(255, 165, 0)'],
                        [55, 'rgb(255, 0, 0)'],
                        [150, 'rgb(128, 0, 128)'],
                        [250, 'rgb(0, 0, 0)'],
                    ],
                },
                'circle-opacity': 0.8,
            },
        });

        map.on('click', circleLayerId, (event) => {
            const coords = event.features[0].geometry.coordinates;
            const props = event.features[0].properties;
            activePopup = new maplibregl.Popup()
                .setLngLat(coords)
                .setHTML(`<strong>PM1:</strong> ${props.intensity.toFixed(2)}`)
                .addTo(map);
        });
    }
}

// Updates the heatmap layer for NOx sensor with distance and time logic
function updateHeatmapLayerNOx(longitude, latitude, intensity, replace = true) {
    const currentTimeSeconds = Math.floor(Date.now() / 1000);
    const activeSensor = 'nox';

    if (currentActiveSensor !== activeSensor) {
        console.log(`[NOx] New sensor selected: ${activeSensor}, clearing heatmap.`);
        clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
        currentActiveSensor = activeSensor; // Update the active sensor
        isHeatmapCleared = true; // Mark the heatmap as cleared
    }

    // Check if this is the first point to add
    if (sensorTracking.nox.lat !== null && sensorTracking.nox.lon !== null) {
        // Perform distance and time checks for subsequent points
        const distance = calculateDistance(sensorTracking.nox.lat, sensorTracking.nox.lon, latitude, longitude);
        const timeDifference = currentTimeSeconds - sensorTracking.nox.timestamp;

        console.log(`[NOx] Distance: ${distance}, Time Difference: ${timeDifference} ms`);

        // If drone hovers for 10s in the same location and distance < 1m, update intensity but do not add new points
        if (distance < 1 && timeDifference > 10 && !isHeatmapCleared) {
            console.log("[NOx] Sensor hovering for more than 10 seconds.");

            // Initialize history if not already initialized
            if (!sensorTracking.nox.history) {
                sensorTracking.nox.history = [];
            }

            // Add intensity to history and calculate the running average
            sensorTracking.nox.history.push(intensity);
            if (sensorTracking.nox.history.length > 10) {
                sensorTracking.nox.history.shift(); // Remove the oldest value
            }

            const runningAverage = sensorTracking.nox.history.reduce((sum, val) => sum + val, 0) / sensorTracking.nox.history.length;
            intensity = runningAverage;

            // Update the existing GeoJSON feature with the new intensity
            const dataId = 'balora_nox_data';
            if (map.getSource(dataId)) {
                const matchingData = map.getSource(dataId)._data.features.find((data) => {
                    return (
                        data.geometry.type === 'Point' &&
                        Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                        Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
                    );
                });

                if (matchingData) {
                    matchingData.properties.intensity = intensity; // Update intensity
                    map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
                }
            }

            console.log("[NOx] Updated running average intensity:", intensity);
            return; // Exit to prevent adding a new feature
        }

        // Only update the timestamp if the sensor has moved significantly
        if (distance >= 1) {
            sensorTracking.nox = {
                lat: latitude,
                lon: longitude,
                timestamp: currentTimeSeconds, // Update timestamp only when the sensor moves
                history: sensorTracking.nox.history || [], // Preserve existing history
            };
        }
    } else {
        // Initialize tracking data if no previous data exists
        sensorTracking.nox = {
            lat: latitude,
            lon: longitude,
            timestamp: currentTimeSeconds,
            history: [],
        };
    }

    isHeatmapCleared = false;

    // Heatmap update logic
    const dataId = 'balora_nox_data';
    const layerId = 'balora_nox_heat_layer';
    const circleLayerId = 'balora_nox_circles_layer';

    if (map.getSource(dataId)) {
        const matchingData = map.getSource(dataId)._data.features.find((data) => {
            return (
                data.geometry.type === 'Point' &&
                Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
            );
        });

        if (matchingData) {
            if (replace) {
                matchingData.properties.intensity = intensity;
                map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
            }
        } else {
            map.getSource(dataId).setData({
                type: 'FeatureCollection',
                features: map.getSource(dataId)._data.features.concat([
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ]),
            });
        }
    } else {
        map.addSource(dataId, {
            type: 'geojson',
            data: {
                type: 'FeatureCollection',
                features: [
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ],
            },
        });

        map.addLayer({
            id: layerId,
            type: 'heatmap',
            source: dataId,
            paint: {
                'heatmap-weight': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [1, 0],
                        [250, 1],
                    ],
                },
                'heatmap-color': [
                    'interpolate',
                    ['linear'],
                    ['heatmap-density'],
                    0,
                    'rgba(236,222,239,0)',
                    0.2,
                    'rgb(0, 255, 0)', // Green
                    0.4,
                    'rgb(255, 255, 0)', // Yellow
                    0.6,
                    'rgb(255, 165, 0)', // Orange
                    1,
                    'rgb(255, 0, 0)', // Red
                ],
                'heatmap-radius': {
                    stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
                },
                'heatmap-opacity': {
                    default: 1,
                    stops: [
                        [17, 1],
                        [18, 0],
                    ],
                },
            },
        });

        map.addLayer({
            id: circleLayerId,
            type: 'circle',
            source: dataId,
            paint: {
                'circle-radius': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [0, 15], // Minimum intensity value, circle radius in pixels
                        [250, 30], // Maximum intensity value, circle radius in pixels
                    ],
                },
                'circle-color': {
                    property: 'intensity',
                    type: 'interval',
                    stops: [
                        [0, 'rgb(0, 255, 0)'], // Green
                        [20, 'rgb(255, 255, 0)'], // Yellow
                        [150, 'rgb(255, 165, 0)'], // Orange
                        [300, 'rgb(255, 0, 0)'], // Red
                    ],
                },
                'circle-opacity': 0.8,
            },
        });

        map.on('click', circleLayerId, (event) => {
            const coords = event.features[0].geometry.coordinates;
            const props = event.features[0].properties;
            activePopup = new maplibregl.Popup()
                .setLngLat(coords)
                .setHTML(`<strong>NOx:</strong> ${props.intensity.toFixed(2)}`)
                .addTo(map);
        });
    }
}



// Updates the heatmap layer for VOC sensor with distance and time logic
function updateHeatmapLayerVOC(longitude, latitude, intensity, replace = true) {
    const currentTimeSeconds = Math.floor(Date.now() / 1000);
    const activeSensor = 'voc';

    if (currentActiveSensor !== activeSensor) {
        console.log(`[VOC] New sensor selected: ${activeSensor}, clearing heatmap.`);
        clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
        currentActiveSensor = activeSensor; // Update the active sensor
        isHeatmapCleared = true; // Mark the heatmap as cleared
    }

    // Check if this is the first point to add
    if (sensorTracking.voc.lat !== null && sensorTracking.voc.lon !== null) {
        // Perform distance and time checks for subsequent points
        const distance = calculateDistance(sensorTracking.voc.lat, sensorTracking.voc.lon, latitude, longitude);
        const timeDifference = currentTimeSeconds - sensorTracking.voc.timestamp;

        console.log(`[VOC] Distance: ${distance}, Time Difference: ${timeDifference} ms`);

        // If drone hovers for 10s in the same location and distance < 1m, update intensity but do not add new points
        if (distance < 1 && timeDifference > 10 && !isHeatmapCleared) {
            console.log("[VOC] Sensor hovering for more than 10 seconds.");

            // Initialize history if not already initialized
            if (!sensorTracking.voc.history) {
                sensorTracking.voc.history = [];
            }

            // Add intensity to history and calculate the running average
            sensorTracking.voc.history.push(intensity);
            if (sensorTracking.voc.history.length > 10) {
                sensorTracking.voc.history.shift(); // Remove the oldest value
            }

            const runningAverage = sensorTracking.voc.history.reduce((sum, val) => sum + val, 0) / sensorTracking.voc.history.length;
            intensity = runningAverage;

            // Update the existing GeoJSON feature with the new intensity
            const dataId = 'balora_voc_data';
            if (map.getSource(dataId)) {
                const matchingData = map.getSource(dataId)._data.features.find((data) => {
                    return (
                        data.geometry.type === 'Point' &&
                        Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                        Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
                    );
                });

                if (matchingData) {
                    matchingData.properties.intensity = intensity; // Update intensity
                    map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
                }
            }

            console.log("[VOC] Updated running average intensity:", intensity);
            return; // Exit to prevent adding a new feature
        }

        // Only update the timestamp if the sensor has moved significantly
        if (distance >= 1) {
            sensorTracking.voc = {
                lat: latitude,
                lon: longitude,
                timestamp: currentTimeSeconds, // Update timestamp only when the sensor moves
                history: sensorTracking.voc.history || [], // Preserve existing history
            };
        }
    } else {
        // Initialize tracking data if no previous data exists
        sensorTracking.voc = {
            lat: latitude,
            lon: longitude,
            timestamp: currentTimeSeconds,
            history: [],
        };
    }

    isHeatmapCleared = false;

    // Heatmap update logic
    const dataId = 'balora_voc_data';
    const layerId = 'balora_voc_heat_layer';
    const circleLayerId = 'balora_voc_circles_layer';

    if (map.getSource(dataId)) {
        const matchingData = map.getSource(dataId)._data.features.find((data) => {
            return (
                data.geometry.type === 'Point' &&
                Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
            );
        });

        if (matchingData) {
            if (replace) {
                matchingData.properties.intensity = intensity;
                map.getSource(dataId).setData(map.getSource(dataId)._data); // Apply updates to the map
            }
        } else {
            map.getSource(dataId).setData({
                type: 'FeatureCollection',
                features: map.getSource(dataId)._data.features.concat([
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ]),
            });
        }
    } else {
        map.addSource(dataId, {
            type: 'geojson',
            data: {
                type: 'FeatureCollection',
                features: [
                    {
                        type: 'Feature',
                        geometry: { type: 'Point', coordinates: [longitude, latitude] },
                        properties: { intensity: intensity },
                    },
                ],
            },
        });

        map.addLayer({
            id: layerId,
            type: 'heatmap',
            source: dataId,
            paint: {
                'heatmap-weight': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [1, 0],
                        [250, 1],
                    ],
                },
                'heatmap-color': [
                    'interpolate',
                    ['linear'],
                    ['heatmap-density'],
                    0,
                    'rgba(236,222,239,0)',
                    0.2,
                    'rgb(0, 255, 0)', // Green
                    0.4,
                    'rgb(255, 255, 0)', // Yellow
                    0.6,
                    'rgb(255, 165, 0)', // Orange
                    1,
                    'rgb(255, 0, 0)', // Red
                ],
                'heatmap-radius': {
                    stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
                },
                'heatmap-opacity': {
                    default: 1,
                    stops: [
                        [17, 1],
                        [18, 0],
                    ],
                },
            },
        });

        map.addLayer({
            id: circleLayerId,
            type: 'circle',
            source: dataId,
            paint: {
                'circle-radius': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [0, 15], // Minimum intensity value, circle radius in pixels
                        [250, 30], // Maximum intensity value, circle radius in pixels
                    ],
                },
                'circle-color': {
                    property: 'intensity',
                    type: 'interval',
                    stops: [
                        [0, 'rgb(0, 255, 0)'], // Green
                        [150, 'rgb(255, 255, 0)'], // Yellow
                        [250, 'rgb(255, 165, 0)'], // Orange
                        [400, 'rgb(255, 0, 0)'], // Red
                    ],
                },
                'circle-opacity': 0.8,
            },
        });

        map.on('click', circleLayerId, (event) => {
            const coords = event.features[0].geometry.coordinates;
            const props = event.features[0].properties;
            activePopup = new maplibregl.Popup()
                .setLngLat(coords)
                .setHTML(`<strong>VOC:</strong> ${props.intensity.toFixed(2)}`)
                .addTo(map);
        });
    }
}








// Function to calculate distance using the Haversine formula
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Earth's radius in meters
    const toRadians = (degrees) => (degrees * Math.PI) / 180;

    const dLat = toRadians(lat2 - lat1);
    const dLon = toRadians(lon2 - lon1);
    const a =
        Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) *
        Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}




function getBaloraPM25SensorData(element) {
    postElementId('PM25 Sensor Data', element.checked);
    console.log(dutils.urls.resolve('balora_pm25', {
        operation_name: CURRENT_OP,
    }));
    if (element.checked) {
        getAllBaloraPM25SensorData().then((data) => {
            console.log("DATA:", data);
            data.forEach((element) => {
                updateHeatmapLayerPM25(
                    element.longitude,
                    element.latitude,
                    element.pm25,
                    false,
                );
            });
        });
    } else {
        if (map.getLayer('balora_pm25_heat_layer')) {
            map.removeLayer('balora_pm25_heat_layer');
        }
        if (map.getLayer('balora_pm25_circles_layer')) {
            map.removeLayer('balora_pm25_circles_layer');
        }
        if (map.getSource('balora_pm25_data')) {
            map.removeSource('balora_pm25_data');
        }
    }
}

function getBaloraPM1SensorData(element) {
    postElementId('PM1 Sensor Data', element.checked);
    console.log(dutils.urls.resolve('balora_pm1', {
        operation_name: CURRENT_OP,
    }));
    if (element.checked) {
        getAllBaloraPM1SensorData().then((data) => {
            data.forEach((element) => {
                updateHeatmapLayerPM1(
                    element.longitude,
                    element.latitude,
                    element.pm1,
                    false,
                );
            });
        });
    } else {
        if (map.getLayer('balora_pm1_heat_layer')) {
            map.removeLayer('balora_pm1_heat_layer');
        }
        if (map.getLayer('balora_pm1_circles_layer')) {
            map.removeLayer('balora_pm1_circles_layer');
        }
        if (map.getSource('balora_pm1_data')) {
            map.removeSource('balora_pm1_data');
        }
    }
}

function getBaloraNOxSensorData(element) {
    postElementId('NOx Sensor Data', element.checked);
    console.log(dutils.urls.resolve('balora_nox_avg', {
        operation_name: CURRENT_OP,
    }));
    if (element.checked) {
        getAllBaloraNOxSensorData().then((data) => {
            data.forEach((element) => {
                updateHeatmapLayerNOx(
                    element.longitude,
                    element.latitude,
                    element.nox,
                    false,
                );
            });
        });
        
    } else {
        if (map.getLayer('balora_nox_heat_layer')) {
            map.removeLayer('balora_nox_heat_layer');
        }
        if (map.getLayer('balora_nox_circles_layer')) {
            map.removeLayer('balora_nox_circles_layer');
        }
        if (map.getSource('balora_nox_data')) {
            map.removeSource('balora_nox_data');
        }
    }
}

function getBaloraVOCSensorData(element) {
    postElementId('VOC Sensor Data', element.checked);
    console.log(dutils.urls.resolve('balora_voc_avg', {
        operation_name: CURRENT_OP,
    }));
    if (element.checked) {
        getAllBaloraVOCSensorData().then((data) => {
            console.log("DD",data);
            data.forEach((element) => {
                updateHeatmapLayerVOC(
                    element.longitude,
                    element.latitude,
                    element.voc,
                    false,
                );
            });
        });
    } else {
        if (map.getLayer('balora_nox_heat_layer')) {
            map.removeLayer('balora_nox_heat_layer');
        }
        if (map.getLayer('balora_nox_circles_layer')) {
            map.removeLayer('balora_nox_circles_layer');
        }
        if (map.getSource('balora_nox_data')) {
            map.removeSource('balora_nox_data');
        }
    }
}


////////////////////////////////////////////////////////////////


// Global tracking object for running averages per baloraName
// const pm25Tracking = {};
// const pm1Tracking = {};
// const noxTracking = {};
// const vocTracking = {};

// function updateHeatmapLayerPM25(longitude, latitude, intensity, baloraName, replace = true) {
//     const currentTime = Date.now();
//     const activeSensor = 'pm25';

//     if (currentActiveSensor !== activeSensor) {
//         console.log(`[PM2.5] New sensor selected: ${activeSensor}, clearing heatmap.`);
//         clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
//         currentActiveSensor = activeSensor; // Update the active sensor
//         isHeatmapCleared = true; // Mark the heatmap as cleared
//     }

//     // Initialize tracking for this baloraName if not already present
//     if (!pm25Tracking[baloraName]) {
//         pm25Tracking[baloraName] = {
//             lat: null,
//             lon: null,
//             timestamp: null,
//             history: [],
//         };
//     }

//     const currentBalora = pm25Tracking[baloraName];

//     // Check if this is the first point to add
//     if (currentBalora.lat !== null || currentBalora.lon !== null) {
//         // Perform distance and time checks for subsequent points
//         const distance = calculateDistance(currentBalora.lat, currentBalora.lon, latitude, longitude);
//         const timeDifference = currentTime - currentBalora.timestamp;

//         if ((distance < 1 || timeDifference > 10000) && !isHeatmapCleared) {
//             console.log(`[${baloraName}] Skipping map update: Distance < 1 meter OR time > 10 seconds.`);
//             return;
//         }

//         // If conditions are met, calculate running average
//         currentBalora.history.push(intensity);
//         if (currentBalora.history.length > 10) {
//             currentBalora.history.shift(); // Remove the oldest value
//         }

//         // Calculate running average
//         const runningAverage = currentBalora.history.reduce((sum, val) => sum + val, 0) / currentBalora.history.length;
//         console.log(`[${baloraName}] Running average PM2.5: ${runningAverage}`);
//         intensity = runningAverage;
//     }

//     // Update the tracking data for this baloraName
//     currentBalora.lat = latitude;
//     currentBalora.lon = longitude;
//     currentBalora.timestamp = currentTime;

//     isHeatmapCleared = false;

//     // Heatmap update logic
//     const dataId = 'balora_pm25_data';
//     const layerId = 'balora_pm25_heat_layer';
//     const circleLayerId = 'balora_pm25_circles_layer';

//     if (map.getSource(dataId)) {
//         const matchingData = map.getSource(dataId)._data.features.find((data) => {
//             return (
//                 data.geometry.type === 'Point' &&
//                 Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
//                 Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
//             );
//         });

//         if (matchingData) {
//             if (replace) {
//                 matchingData.properties.intensity = intensity;
//             }
//         } else {
//             map.getSource(dataId).setData({
//                 type: 'FeatureCollection',
//                 features: map.getSource(dataId)._data.features.concat([
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ]),
//             });
//         }
//     } else {
//         map.addSource(dataId, {
//             type: 'geojson',
//             data: {
//                 type: 'FeatureCollection',
//                 features: [
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ],
//             },
//         });

//         map.addLayer({
//             id: layerId,
//             type: 'heatmap',
//             source: dataId,
//             paint: {
//                 'heatmap-weight': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [1, 0],
//                         [250, 1],
//                     ],
//                 },
//                 'heatmap-color': [
//                     'interpolate',
//                     ['linear'],
//                     ['heatmap-density'],
//                     0,
//                     'rgba(236,222,239,0)',
//                     0.2,
//                     'rgb(255, 255, 0)',
//                     0.4,
//                     'rgb(255, 165, 0)',
//                     0.6,
//                     'rgb(255, 0, 0)',
//                     0.8,
//                     'rgb(128, 0, 128)',
//                     1,
//                     'rgb(0, 0, 0)',
//                 ],
//                 'heatmap-radius': {
//                     stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
//                 },
//                 'heatmap-opacity': {
//                     default: 1,
//                     stops: [
//                         [17, 1],
//                         [18, 0],
//                     ],
//                 },
//             },
//         });

//         map.addLayer({
//             id: circleLayerId,
//             type: 'circle',
//             source: dataId,
//             paint: {
//                 'circle-radius': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [0, 15], // Minimum intensity value, circle radius in pixels
//                         [250, 30], // Maximum intensity value, circle radius in pixels
//                     ],
//                 },
//                 'circle-color': {
//                     property: 'intensity',
//                     type: 'interval',
//                     stops: [
//                         [0, 'rgb(0, 255, 0)'], // Color for low intensity values
//                         [12, 'rgb(255, 255, 0)'], // Color for moderate intensity values
//                         [35, 'rgb(255, 165, 0)'], // Color for high intensity values
//                         [55, 'rgb(255, 0, 0)'], // Color for very high intensity values
//                         [150, 'rgb(128, 0, 128)'],
//                         [250, 'rgb(0, 0, 0)'],
//                     ],
//                 },
//                 'circle-opacity': 0.8,
//             },
//         });

//         map.on('click', circleLayerId, (event) => {
//             const coords = event.features[0].geometry.coordinates;
//             const props = event.features[0].properties;
//             activePopup = new maplibregl.Popup()
//                 .setLngLat(coords)
//                 .setHTML(`<strong>PM2.5:</strong>  ${props.intensity.toFixed(2)}`)
//                 .addTo(map);
//         });
//     }
// }

// // Global tracking object for running averages per baloraName for PM1


// function updateHeatmapLayerPM1(longitude, latitude, intensity, baloraName, replace = true) {
//     const currentTime = Date.now();
//     const activeSensor = 'pm1';

//     if (currentActiveSensor !== activeSensor) {
//         console.log(`[PM1] New sensor selected: ${activeSensor}, clearing heatmap.`);
//         clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
//         currentActiveSensor = activeSensor; // Update the active sensor
//         isHeatmapCleared = true; // Mark the heatmap as cleared
//     }

//     // Initialize tracking for this baloraName if not already present
//     if (!pm1Tracking[baloraName]) {
//         pm1Tracking[baloraName] = {
//             lat: null,
//             lon: null,
//             timestamp: null,
//             history: [],
//         };
//     }

//     const currentBalora = pm1Tracking[baloraName];

//     // Check if this is the first point to add
//     if (currentBalora.lat !== null || currentBalora.lon !== null) {
//         // Perform distance and time checks for subsequent points
//         const distance = calculateDistance(currentBalora.lat, currentBalora.lon, latitude, longitude);
//         const timeDifference = currentTime - currentBalora.timestamp;

//         // Skip updates if the drone hovers in the same location or hasn't moved significantly
//         if ((distance < 1 || timeDifference > 10000) && !isHeatmapCleared) {
//             console.log(`[PM1] Skipping map update: Distance < 1 meter OR time > 10 seconds for ${baloraName}.`);
//             return;
//         }

//         // If conditions are met, calculate running average
//         currentBalora.history.push(intensity);
//         if (currentBalora.history.length > 10) {
//             currentBalora.history.shift(); // Remove the oldest value
//         }

//         // Calculate running average
//         const runningAverage = currentBalora.history.reduce((sum, val) => sum + val, 0) / currentBalora.history.length;
//         console.log(`[PM1] Running average intensity for ${baloraName}: ${runningAverage}`);
//         intensity = runningAverage;
//     }

//     // Update the tracking data for this baloraName
//     currentBalora.lat = latitude;
//     currentBalora.lon = longitude;
//     currentBalora.timestamp = currentTime;

//     isHeatmapCleared = false;

//     // Heatmap update logic
//     const dataId = 'balora_pm1_data';
//     const layerId = 'balora_pm1_heat_layer';
//     const circleLayerId = 'balora_pm1_circles_layer';

//     if (map.getSource(dataId)) {
//         const matchingData = map.getSource(dataId)._data.features.find((data) => {
//             return (
//                 data.geometry.type === 'Point' &&
//                 Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
//                 Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
//             );
//         });

//         if (matchingData) {
//             if (replace) {
//                 matchingData.properties.intensity = intensity;
//             }
//         } else {
//             map.getSource(dataId).setData({
//                 type: 'FeatureCollection',
//                 features: map.getSource(dataId)._data.features.concat([
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ]),
//             });
//         }
//     } else {
//         map.addSource(dataId, {
//             type: 'geojson',
//             data: {
//                 type: 'FeatureCollection',
//                 features: [
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ],
//             },
//         });

//         map.addLayer({
//             id: layerId,
//             type: 'heatmap',
//             source: dataId,
//             paint: {
//                 'heatmap-weight': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [1, 0],
//                         [250, 1],
//                     ],
//                 },
//                 'heatmap-color': [
//                     'interpolate',
//                     ['linear'],
//                     ['heatmap-density'],
//                     0,
//                     'rgba(236,222,239,0)',
//                     0.2,
//                     'rgb(255, 255, 0)',
//                     0.4,
//                     'rgb(255, 165, 0)',
//                     0.6,
//                     'rgb(255, 0, 0)',
//                     0.8,
//                     'rgb(128, 0, 128)',
//                     1,
//                     'rgb(0, 0, 0)',
//                 ],
//                 'heatmap-radius': {
//                     stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
//                 },
//                 'heatmap-opacity': {
//                     default: 1,
//                     stops: [
//                         [17, 1],
//                         [18, 0],
//                     ],
//                 },
//             },
//         });

//         map.addLayer({
//             id: circleLayerId,
//             type: 'circle',
//             source: dataId,
//             paint: {
//                 'circle-radius': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [0, 15], // Minimum intensity value, circle radius in pixels
//                         [250, 30], // Maximum intensity value, circle radius in pixels
//                     ],
//                 },
//                 'circle-color': {
//                     property: 'intensity',
//                     type: 'interval',
//                     stops: [
//                         [0, 'rgb(0, 255, 0)'], // Color for low intensity values
//                         [12, 'rgb(255, 255, 0)'], // Color for moderate intensity values
//                         [35, 'rgb(255, 165, 0)'], // Color for high intensity values
//                         [55, 'rgb(255, 0, 0)'], // Color for very high intensity values
//                         [150, 'rgb(128, 0, 128)'],
//                         [250, 'rgb(0, 0, 0)'],
//                     ],
//                 },
//                 'circle-opacity': 0.8,
//             },
//         });

//         map.on('click', circleLayerId, (event) => {
//             const coords = event.features[0].geometry.coordinates;
//             const props = event.features[0].properties;
//             activePopup = new maplibregl.Popup()
//                 .setLngLat(coords)
//                 .setHTML(`<strong>PM1:</strong> ${props.intensity.toFixed(2)}`)
//                 .addTo(map);
//         });
//     }
// }

// // Global tracking object for running averages per baloraName for NOx


// function updateHeatmapLayerNOx(longitude, latitude, intensity, baloraName, replace = true) {
//     const currentTime = Date.now();
//     const activeSensor = 'nox';

//     if (currentActiveSensor !== activeSensor) {
//         console.log(`[NOx] New sensor selected: ${activeSensor}, clearing heatmap.`);
//         clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
//         currentActiveSensor = activeSensor; // Update the active sensor
//         isHeatmapCleared = true; // Mark the heatmap as cleared
//     }

//     // Initialize tracking for this baloraName if not already present
//     if (!noxTracking[baloraName]) {
//         noxTracking[baloraName] = {
//             lat: null,
//             lon: null,
//             timestamp: null,
//             history: [],
//         };
//     }

//     const currentBalora = noxTracking[baloraName];

//     // Check if this is the first point to add
//     if (currentBalora.lat !== null || currentBalora.lon !== null) {
//         // Perform distance and time checks for subsequent points
//         const distance = calculateDistance(currentBalora.lat, currentBalora.lon, latitude, longitude);
//         const timeDifference = currentTime - currentBalora.timestamp;

//         // Skip updates if the drone hovers in the same location or hasn't moved significantly
//         if ((distance < 1 || timeDifference > 10000) && !isHeatmapCleared) {
//             console.log(`[NOx] Skipping map update: Distance < 1 meter OR time > 10 seconds for ${baloraName}.`);
//             return;
//         }

//         // If conditions are met, calculate running average
//         currentBalora.history.push(intensity);
//         if (currentBalora.history.length > 10) {
//             currentBalora.history.shift(); // Remove the oldest value
//         }

//         // Calculate running average
//         const runningAverage = currentBalora.history.reduce((sum, val) => sum + val, 0) / currentBalora.history.length;
//         console.log(`[NOx] Running average intensity for ${baloraName}: ${runningAverage}`);
//         intensity = runningAverage;
//     }

//     // Update the tracking data for this baloraName
//     currentBalora.lat = latitude;
//     currentBalora.lon = longitude;
//     currentBalora.timestamp = currentTime;

//     isHeatmapCleared = false;

//     // Heatmap update logic
//     const dataId = 'balora_nox_data';
//     const layerId = 'balora_nox_heat_layer';
//     const circleLayerId = 'balora_nox_circles_layer';

//     if (map.getSource(dataId)) {
//         const matchingData = map.getSource(dataId)._data.features.find((data) => {
//             return (
//                 data.geometry.type === 'Point' &&
//                 Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
//                 Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
//             );
//         });

//         if (matchingData) {
//             if (replace) {
//                 matchingData.properties.intensity = intensity;
//             }
//         } else {
//             map.getSource(dataId).setData({
//                 type: 'FeatureCollection',
//                 features: map.getSource(dataId)._data.features.concat([
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ]),
//             });
//         }
//     } else {
//         map.addSource(dataId, {
//             type: 'geojson',
//             data: {
//                 type: 'FeatureCollection',
//                 features: [
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ],
//             },
//         });

//         map.addLayer({
//             id: layerId,
//             type: 'heatmap',
//             source: dataId,
//             paint: {
//                 'heatmap-weight': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [1, 0],
//                         [250, 1],
//                     ],
//                 },
//                 'heatmap-color': [
//                     'interpolate',
//                     ['linear'],
//                     ['heatmap-density'],
//                     0,
//                     'rgba(236,222,239,0)',
//                     0.2,
//                     'rgb(0, 255, 0)', // Green
//                     0.4,
//                     'rgb(255, 255, 0)', // Yellow
//                     0.6,
//                     'rgb(255, 165, 0)', // Orange
//                     1,
//                     'rgb(255, 0, 0)', // Red
//                 ],
//                 'heatmap-radius': {
//                     stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
//                 },
//                 'heatmap-opacity': {
//                     default: 1,
//                     stops: [
//                         [17, 1],
//                         [18, 0],
//                     ],
//                 },
//             },
//         });

//         map.addLayer({
//             id: circleLayerId,
//             type: 'circle',
//             source: dataId,
//             paint: {
//                 'circle-radius': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [0, 15], // Minimum intensity value, circle radius in pixels
//                         [250, 30], // Maximum intensity value, circle radius in pixels
//                     ],
//                 },
//                 'circle-color': {
//                     property: 'intensity',
//                     type: 'interval',
//                     stops: [
//                         [0, 'rgb(0, 255, 0)'], // Green
//                         [20, 'rgb(255, 255, 0)'], // Yellow
//                         [150, 'rgb(255, 165, 0)'], // Orange
//                         [300, 'rgb(255, 0, 0)'], // Red
//                     ],
//                 },
//                 'circle-opacity': 0.8,
//             },
//         });

//         map.on('click', circleLayerId, (event) => {
//             const coords = event.features[0].geometry.coordinates;
//             const props = event.features[0].properties;
//             activePopup = new maplibregl.Popup()
//                 .setLngLat(coords)
//                 .setHTML(`<strong>NOx:</strong> ${props.intensity.toFixed(2)}`)
//                 .addTo(map);
//         });
//     }
// }

// // Global tracking object for running averages per baloraName for VOC


// function updateHeatmapLayerVOC(longitude, latitude, intensity, baloraName, replace = true) {
//     const currentTime = Date.now();
//     const activeSensor = 'voc';

//     if (currentActiveSensor !== activeSensor) {
//         console.log(`[VOC] New sensor selected: ${activeSensor}, clearing heatmap.`);
//         clearAllHeatmaps(); // Clear the heatmap when a new sensor is selected
//         currentActiveSensor = activeSensor; // Update the active sensor
//         isHeatmapCleared = true; // Mark the heatmap as cleared
//     }

//     // Initialize tracking for this baloraName if not already present
//     if (!vocTracking[baloraName]) {
//         vocTracking[baloraName] = {
//             lat: null,
//             lon: null,
//             timestamp: null,
//             history: [],
//         };
//     }

//     const currentBalora = vocTracking[baloraName];

//     // Check if this is the first point to add
//     if (currentBalora.lat !== null || currentBalora.lon !== null) {
//         // Perform distance and time checks for subsequent points
//         const distance = calculateDistance(currentBalora.lat, currentBalora.lon, latitude, longitude);
//         const timeDifference = currentTime - currentBalora.timestamp;

//         // Skip updates if the drone hovers in the same location or hasn't moved significantly
//         if ((distance < 1 || timeDifference > 10000) && !isHeatmapCleared) {
//             console.log(`[VOC] Skipping map update: Distance < 1 meter OR time > 10 seconds for ${baloraName}.`);
//             return;
//         }

//         // If conditions are met, calculate running average
//         currentBalora.history.push(intensity);
//         if (currentBalora.history.length > 10) {
//             currentBalora.history.shift(); // Remove the oldest value
//         }

//         // Calculate running average
//         const runningAverage = currentBalora.history.reduce((sum, val) => sum + val, 0) / currentBalora.history.length;
//         console.log(`[VOC] Running average intensity for ${baloraName}: ${runningAverage}`);
//         intensity = runningAverage;
//     }

//     // Update the tracking data for this baloraName
//     currentBalora.lat = latitude;
//     currentBalora.lon = longitude;
//     currentBalora.timestamp = currentTime;

//     isHeatmapCleared = false;

//     // Heatmap update logic
//     const dataId = 'balora_voc_data';
//     const layerId = 'balora_voc_heat_layer';
//     const circleLayerId = 'balora_voc_circles_layer';

//     if (map.getSource(dataId)) {
//         const matchingData = map.getSource(dataId)._data.features.find((data) => {
//             return (
//                 data.geometry.type === 'Point' &&
//                 Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
//                 Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
//             );
//         });

//         if (matchingData) {
//             if (replace) {
//                 matchingData.properties.intensity = intensity;
//             }
//         } else {
//             map.getSource(dataId).setData({
//                 type: 'FeatureCollection',
//                 features: map.getSource(dataId)._data.features.concat([
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ]),
//             });
//         }
//     } else {
//         map.addSource(dataId, {
//             type: 'geojson',
//             data: {
//                 type: 'FeatureCollection',
//                 features: [
//                     {
//                         type: 'Feature',
//                         geometry: { type: 'Point', coordinates: [longitude, latitude] },
//                         properties: { intensity: intensity },
//                     },
//                 ],
//             },
//         });

//         map.addLayer({
//             id: layerId,
//             type: 'heatmap',
//             source: dataId,
//             paint: {
//                 'heatmap-weight': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [1, 0],
//                         [250, 1],
//                     ],
//                 },
//                 'heatmap-color': [
//                     'interpolate',
//                     ['linear'],
//                     ['heatmap-density'],
//                     0,
//                     'rgba(236,222,239,0)',
//                     0.2,
//                     'rgb(0, 255, 0)', // Green
//                     0.4,
//                     'rgb(255, 255, 0)', // Yellow
//                     0.6,
//                     'rgb(255, 165, 0)', // Orange
//                     1,
//                     'rgb(255, 0, 0)', // Red
//                 ],
//                 'heatmap-radius': {
//                     stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
//                 },
//                 'heatmap-opacity': {
//                     default: 1,
//                     stops: [
//                         [17, 1],
//                         [18, 0],
//                     ],
//                 },
//             },
//         });

//         map.addLayer({
//             id: circleLayerId,
//             type: 'circle',
//             source: dataId,
//             paint: {
//                 'circle-radius': {
//                     property: 'intensity',
//                     type: 'exponential',
//                     stops: [
//                         [0, 15], // Minimum intensity value, circle radius in pixels
//                         [250, 30], // Maximum intensity value, circle radius in pixels
//                     ],
//                 },
//                 'circle-color': {
//                     property: 'intensity',
//                     type: 'interval',
//                     stops: [
//                         [0, 'rgb(0, 255, 0)'], // Green
//                         [150, 'rgb(255, 255, 0)'], // Yellow
//                         [250, 'rgb(255, 165, 0)'], // Orange
//                         [400, 'rgb(255, 0, 0)'], // Red
//                     ],
//                 },
//                 'circle-opacity': 0.8,
//             },
//         });

//         map.on('click', circleLayerId, (event) => {
//             const coords = event.features[0].geometry.coordinates;
//             const props = event.features[0].properties;
//             activePopup = new maplibregl.Popup()
//                 .setLngLat(coords)
//                 .setHTML(`<strong>VOC:</strong> ${props.intensity.toFixed(2)}`)
//                 .addTo(map);
//         });
//     }
// }


