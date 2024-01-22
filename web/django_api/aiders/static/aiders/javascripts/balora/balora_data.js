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
    );
    document.getElementById(baloraId + '_pm25').innerHTML = parseFloat(
        get_all_balora_info_array().find((balora) => balora.baloraID === baloraId).baloraInfo.pm25
    );
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
    console.log(snr);
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
// Create HeatMap Layer that used by balora and update by the longitude latitude and pm25
function updateHeatmapLayer(longitude, latitude, pm25, replace = true) {
    if (map.getSource('balora_pm25_data')) {
        matchingData = map.getSource('balora_pm25_data')._data.features.find(function (data) {
            return (
                data.geometry.type === 'Point' &&
                Math.abs(data.geometry.coordinates[0] - longitude) < 0.0001 &&
                Math.abs(data.geometry.coordinates[1] - latitude) < 0.0001
            );
        });
        if (matchingData) {
            if (replace) {
                matchingData.properties.intensity = pm25; // Replace newIntensity with the desired intensity value
            }
        } else {
            map.getSource('balora_pm25_data').setData({
                type: 'FeatureCollection',
                features: map.getSource('balora_pm25_data')._data.features.concat([
                    {
                        type: 'Feature',
                        geometry: {
                            type: 'Point',
                            coordinates: [longitude, latitude],
                        },
                        properties: {
                            intensity: pm25, // Replace newIntensity with the desired intensity value
                        },
                    },
                ]),
            });
        }
    } else {
        map.addSource('balora_pm25_data', {
            type: 'geojson',
            data: {
                type: 'FeatureCollection',
                features: [
                    {
                        type: 'Feature',
                        geometry: {
                            type: 'Point',
                            coordinates: [longitude, latitude],
                        },
                        properties: {
                            intensity: pm25,
                        },
                    },
                ],
            },
        });
        map.addLayer({
            id: 'balora_pm25_heat_layer',
            type: 'heatmap',
            source: 'balora_pm25_data',
            paint: {
                // increase weight as diameter breast height increases
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
                // increase radius as zoom increases
                'heatmap-radius': {
                    stops: Array.from({ length: 16 }, (_, i) => [i + 1, i + 1]),
                },
                // decrease opacity to transition into the circle layer
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
            id: 'balora_pm25_circles_layer',
            type: 'circle',
            source: 'balora_pm25_data',
            paint: {
                'circle-radius': {
                    property: 'intensity',
                    type: 'exponential',
                    stops: [
                        [0, 5], // Minimum intensity value, circle radius in meters (approximate)
                        [100, 5], // Minimum intensity value, circle radius in pixels
                    ],
                },
                'circle-color': {
                    property: 'intensity',
                    type: 'interval',
                    stops: [
                        [0, 'rgb(0, 255, 0)'], // Color for low intensity values
                        [12, 'rgb(255, 255, 0)'], // Color for medium intensity values
                        [35, 'rgb(255, 165, 0)'],
                        [55, 'rgb(255, 0, 0)'],
                        [150, 'rgb(128, 0, 128)'],
                        [250, 'rgb(0, 0, 0)'], // Color for high intensity values
                    ],
                },
                'circle-opacity': {
                    stops: [
                        [17, 0],
                        [18, 1],
                    ],
                },
            },
        });
        map.on('click', 'balora_pm25_circles_layer', (event) => {
            new maplibregl.Popup()
                .setLngLat(event.features[0].geometry.coordinates)
                .setHTML(`<strong>PM2.5:</strong> ${event.features[0].properties.intensity}`)
                .addTo(map);
        });
    }
}
function getBaloraPM25SensorData(element) {
    postElementId('PM25 Sensor Data', element.checked);
    if (element.checked) {
        $.ajax({
            type: 'GET',
            url: dutils.urls.resolve('balora_pm25', {
                operation_name: CURRENT_OP,
            }),
            success: function (response) {
                response.forEach((element) => {
                    updateHeatmapLayer(element.longitude, element.latitude, element.pm25, false);
                });
            },
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
