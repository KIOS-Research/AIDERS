{
    function droneClickInfoBox(droneId) {
        const pressed = document.getElementById('drone-info-' + droneId).checked;
        postElementId('drone-info-' + droneId, pressed);
        if (pressed) {
            for (let index = 1; index < 7; index++) {
                if (document.getElementById('Drone' + index).innerHTML.includes('<div> <b>' + droneId + '</b></div>')) {
                    break;
                } else {
                    if (document.getElementById('Drone' + index).innerHTML.includes('div') === false) {
                        box = document.getElementById('Drone' + index);
                        box.innerHTML =
                            `<div> <b>` +
                            droneId +
                            `</b></div>
                            <div> <b>Real-Time Drone Information </b></div>
                            <div><b>Time: </b> <span id='` +
                            droneId +
                            `_time'></span></div>
                            <div><b>Drone state: </b> <span id='` +
                            droneId +
                            `_drone_state'></span></div>
                            <div><b>Battery: </b><span id='` +
                            droneId +
                            `_battery_percentage'></span></div>
                            <div><b>Gps signal: </b> <span id='` +
                            droneId +
                            `_gps_signal'></span></div>
                            <div><b>Satellites: </b><span id='` +
                            droneId +
                            `_satellites'></span></div>
                            <div><b>Latitude: </b><span id='` +
                            droneId +
                            `_latitude'></span></div>
                            <div><b>Longitude: </b><span id='` +
                            droneId +
                            `_longitude'></span></div>
                            <div><b>Altitude: </b><span id='` +
                            droneId +
                            `_altitude'></span></div>
                            <div><b>Speed: </b><span id='` +
                            droneId +
                            `_velocity'></span></div>
                            <div><b>Bearing: </b><span id='` +
                            droneId +
                            `_heading'></span></div>
                            <div><b>Gimbal angle: </b><span id='` +
                            droneId +
                            `_gimbal_angle'></span></div>`;
                        for (let i = 0; i < webSocketMessage['drones'].length; i++) {
                            if (webSocketMessage['drones'][i]['drone_name'] === droneId) {
                                if (webSocketMessage['drones'][i]['water_sampler_available'] === true) {
                                    box.innerHTML =
                                        box.innerHTML +
                                        `<div><b>Sampler Under Water: </b><span id='` +
                                        droneId +
                                        `_water_sampler'></span></div>`;
                                }
                            }
                        }
                        jQuery(box).toggle();
                        jQuery(box).draggable();
                        break;
                    }
                }
            }
        } else {
            for (let index = 1; index < 7; index++) {
                if (document.getElementById('Drone' + index).innerHTML.includes(droneId)) {
                    box = document.getElementById('Drone' + index);
                    jQuery(box).toggle();
                    box.innerHTML = '';
                    break;
                }
            }
        }
    }

    function insertAfter(referenceNode, newNode) {
        referenceNode.parentNode.insertBefore(newNode, referenceNode.nextSibling);
    }

    function createWeatherBoxForDrone(drone_name) {
        platform_weather = document.getElementById('weatherDataBox');
        if (document.getElementById('weatherDataBox' + drone_name) === null) {
            let elem = document.createElement('div');
            elem.setAttribute('id', 'weatherDataBox' + drone_name);
            elem.setAttribute('class', 'weatherDataBox');
            elem.style.display = 'none';
            elem.innerHTML =
                `<div> <b>Weather Station <span id="` +
                'weatherDataBox' +
                drone_name +
                'drone' +
                `"></span> </b></div>
<div> <b>Real-Time Weather Information </b></div>
<div><b>Time: </b> <span id="` +
                'weatherDataBox' +
                drone_name +
                'time1' +
                `"></span></div>
<div><b>Wind Direction: </b> <span id="` +
                'weatherDataBox' +
                drone_name +
                'wDir' +
                `"></span></div>
<div><b>Wind Speed:</b> <span id="` +
                'weatherDataBox' +
                drone_name +
                'wSpeed' +
                `"></span></div>
<div><b>Temperature: </b> <span id="` +
                'weatherDataBox' +
                drone_name +
                'temp' +
                `"></span></div>
<div><b>Pressure: </b><span id="` +
                'weatherDataBox' +
                drone_name +
                'press' +
                `"></span></div>
<div><b>Humidity: </b><span id="` +
                'weatherDataBox' +
                drone_name +
                'humidity' +
                `"></span></div>
<div><b>Heading: </b><span id="` +
                'weatherDataBox' +
                drone_name +
                'heading' +
                `"></span></div>`;
            jQuery(elem).draggable();
            insertAfter(platform_weather, elem);
        }
    }

    weatherIntervalList = [];
    function droneClickWeatherBox(droneId) {
        const pressed = document.getElementById('drone-weather-' + droneId).checked;
        postElementId('drone-weather-' + droneId, pressed);
        console.log('weatherDataBox' + droneId + pressed);
        if (pressed) {
            let clickWeatherBox = $('#weatherDataBox' + droneId);
            clickWeatherBox.toggle();
            clickWeatherBox.draggable();
            console.log(clickWeatherBox);
            weatherIntervalList[droneId] = setInterval(function () {
                presentWeatherData();
            }, WEATHER_UPDATE_INTERVAL);
            function presentWeatherData() {
                for (let i = 0; i < webSocketMessage['drones'].length; i++) {
                    if (webSocketMessage['drones'][i]['drone_name'] === droneId) {
                        let weatherObj = {};
                        weatherObj.drone = droneId;
                        weatherObj.time = webSocketMessage['drones'][i]['weather']['current_time'];
                        weatherObj.windDir = parseFloat(
                            webSocketMessage['drones'][i]['weather']['wind_direction']
                        ).toFixed(2);
                        weatherObj.windSpeed = parseFloat(
                            webSocketMessage['drones'][i]['weather']['wind_speed']
                        ).toFixed(3);
                        weatherObj.temp = parseFloat(webSocketMessage['drones'][i]['weather']['temperature']).toFixed(
                            2
                        );
                        weatherObj.pressure = parseFloat(webSocketMessage['drones'][i]['weather']['pressure']).toFixed(
                            2
                        );
                        weatherObj.humidity = parseFloat(webSocketMessage['drones'][i]['weather']['humidity']).toFixed(
                            2
                        );
                        weatherObj.heading = parseFloat(webSocketMessage['drones'][i]['weather']['heading']).toFixed(3);
                        displayWeatherData(weatherObj, 'weatherDataBox' + droneId);
                        weatherObj = {};
                    }
                }
            }
        } else {
            clearInterval(weatherIntervalList[droneId]);
            let clickWeatherBox = $('#weatherDataBox' + droneId);
            clickWeatherBox.toggle();
        }
    }

    function displayDroneData(droneObj, droneId) {
        let timeDisplay = document.getElementById(droneId + '_time');
        let droneStateDisplay = document.getElementById(droneId + '_drone_state');
        let batteryPercentageDisplay = document.getElementById(droneId + '_battery_percentage');
        let gpsSignalDisplay = document.getElementById(droneId + '_gps_signal');
        let satellitesDisplay = document.getElementById(droneId + '_satellites');
        let latitudeDisplay = document.getElementById(droneId + '_latitude');
        let longitudeDisplay = document.getElementById(droneId + '_longitude');
        let altitudeDisplay = document.getElementById(droneId + '_altitude');
        let velocityDisplay = document.getElementById(droneId + '_velocity');
        let headingDisplay = document.getElementById(droneId + '_heading');
        let gimbalAngleDisplay = document.getElementById(droneId + '_gimbal_angle');

        timeDisplay.textContent = droneObj.time;
        droneStateDisplay.innerHTML = droneObj.drone_state;
        batteryPercentageDisplay.innerHTML = droneObj.battery_percentage + '%';
        gpsSignalDisplay.innerHTML = droneObj.gps_signal + '/5';
        satellitesDisplay.innerHTML = droneObj.satellites;
        latitudeDisplay.innerHTML = parseFloat(droneObj.latitude).toFixed(6) + '°';
        longitudeDisplay.innerHTML = parseFloat(droneObj.longitude).toFixed(6) + '°';
        altitudeDisplay.innerHTML = parseFloat(droneObj.altitude).toFixed(1) + ' m';
        velocityDisplay.innerHTML = parseFloat(droneObj.velocity).toFixed(1) + ' m/s';
        headingDisplay.innerHTML = parseFloat(droneObj.heading).toFixed(1) + '°';
        gimbalAngleDisplay.innerHTML = parseFloat(droneObj.gimbal_angle + '°').toFixed(1);

        for (let i = 0; i < webSocketMessage['drones'].length; i++) {
            if (webSocketMessage['drones'][i]['drone_name'] === droneId) {
                if (webSocketMessage['drones'][i]['water_sampler_available'] === true) {
                    let waterSamplerUnderWater = document.getElementById(droneId + '_water_sampler');
                    waterSamplerUnderWater.innerHTML = droneObj.under_water;
                }
            }
        }
    }
}
