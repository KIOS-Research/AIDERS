function createWeatherBoxForDrone(drone_name) {
    // console.log("createWeatherBoxForDrone");
    if (document.getElementById("weatherDataBox" + drone_name) === null) {
        let div = document.createElement("div");
        div.setAttribute("id", "weatherDataBox" + drone_name);
        div.setAttribute("class", "weatherDataBox overlay-popup");
        // div.classList.add("overlay-popup");
        div.style.display = "none";
        document.getElementsByClassName("overlay-section")[0].appendChild(div);
        div.innerHTML =
            `<div> <b>Weather ` +
            ` <span id="` +
            "weatherDataBox" +
            drone_name +
            "drone" +
            `"></span> </b></div>
                <div><b>Time: </b> <span id="` +
            "weatherDataBox" +
            drone_name +
            "time1" +
            `"></span></div>
                <div><b>Wind Direction: </b> <span id="` +
            "weatherDataBox" +
            drone_name +
            "wDir" +
            `"></span></div>
                <div><b>Wind Speed:</b> <span id="` +
            "weatherDataBox" +
            drone_name +
            "wSpeed" +
            `"></span></div>
                <div><b>Temperature: </b> <span id="` +
            "weatherDataBox" +
            drone_name +
            "temp" +
            `"></span></div>
                <div><b>Pressure: </b><span id="` +
            "weatherDataBox" +
            drone_name +
            "press" +
            `"></span></div>
                <div><b>Humidity: </b><span id="` +
            "weatherDataBox" +
            drone_name +
            "humidity" +
            `"></span></div>
                <div><b>Heading: </b><span id="` +
            "weatherDataBox" +
            drone_name +
            "heading" +
            `"></span></div>`;
    }
}

weatherIntervalList = [];
function droneClickWeatherBox(toggleId, droneId) {
    console.log("droneClickWeatherBox");
    const pressed = document.getElementById(toggleId).checked;
    postElementId("Weather " + droneId, pressed);
    if (pressed) {
        let clickWeatherBox = $("#weatherDataBox" + droneId);
        clickWeatherBox.toggle();
        clickWeatherBox.draggable();
        weatherIntervalList[droneId] = setInterval(function () {
            presentWeatherData();
        }, WEATHER_UPDATE_INTERVAL);
        function presentWeatherData() {
            for (let i = 0; i < g_websocketMessage["drones"].length; i++) {
                if (g_websocketMessage["drones"][i]["drone_name"] === droneId) {
                    let weatherObj = {};
                    weatherObj.drone = droneId;
                    weatherObj.time = g_websocketMessage["drones"][i]["weather"]["time"];
                    weatherObj.windDir = parseFloat(
                        g_websocketMessage["drones"][i]["weather"]["wind_direction"],
                    ).toFixed(2);
                    weatherObj.windSpeed = parseFloat(g_websocketMessage["drones"][i]["weather"]["wind_speed"]).toFixed(
                        3,
                    );
                    weatherObj.temp = parseFloat(g_websocketMessage["drones"][i]["weather"]["temperature"]).toFixed(2);
                    weatherObj.pressure = parseFloat(g_websocketMessage["drones"][i]["weather"]["pressure"]).toFixed(2);
                    weatherObj.humidity = parseFloat(g_websocketMessage["drones"][i]["weather"]["humidity"]).toFixed(2);
                    weatherObj.heading = parseFloat(g_websocketMessage["drones"][i]["weather"]["heading"]).toFixed(3);
                    displayWeatherData(weatherObj, "weatherDataBox" + droneId, false);
                    weatherObj = {};
                }
            }
        }
    } else {
        clearInterval(weatherIntervalList[droneId]);
        let clickWeatherBox = $("#weatherDataBox" + droneId);
        clickWeatherBox.toggle();
    }
}

function changeDroneTelemetryElementVisibility(toggleId, _droneName) {
    const toggleElement = document.getElementById(toggleId);
    const infoBox = document.getElementById("droneInfoBox" + _droneName);

    if (!toggleElement || !infoBox) {
        return;
    }

    if (toggleElement.checked) {
        infoBox.style.display = "block";
    } else {
        infoBox.style.display = "none";
    }
}

function createDroneTelemetryElement(_droneName) {
    let div = document.createElement("div");
    div.id = "droneInfoBox" + _droneName;
    div.classList.add("droneDataBox");
    div.classList.add("overlay-popup");
    div.style.position = "absolute";
    let offset = getNumberOfOverlayPanels() * 20;
    div.style.top = offset + "px";
    div.style.left = offset + "px";
    div.style.display = "none";
    document.getElementsByClassName("overlay-section")[0].appendChild(div);
    div.innerHTML = `
	<div> <b>Drone:${_droneName}</b></div>
	<div><b>Type: </b> <span id='${_droneName}_comm_type'></span></div>
	<div><b>Config: </b> <span id='${_droneName}_configuration'></span></div>
	<div><b>Time: </b> <span id='${_droneName}_time'></span></div>
	<div><b>State: </b> <span id='${_droneName}_drone_state'></span></div>
	<div id='${_droneName}_vtol_state_wrapper'><b>VTOL State: </b> <span id='${_droneName}_vtol_state'></span></div>
	<div><b>Battery: </b><span id='${_droneName}_battery_percentage'></span></div>
	<div id='${_droneName}_gps_signal_wrapper'><b>Gps signal: </b> <span id='${_droneName}_gps_signal'></span></div>
	<div><b>Satellites: </b><span id='${_droneName}_satellites'></span></div>
	<div><b>Latitude: </b><span id='${_droneName}_latitude'></span></div>
	<div><b>Longitude: </b><span id='${_droneName}_longitude'></span></div>
	<div><b>Altitude: </b><span id='${_droneName}_altitude'></span></div>
	<div><b>Speed: </b><span id='${_droneName}_velocity'></span></div>
	<div><b>Bearing: </b><span id='${_droneName}_heading'></span></div>
	<div><b>Gimbal: </b><span id='${_droneName}_gimbal_angle'></span></div>
	<div id='${_droneName}_crps_wrapper'><b>CRPS: </b><span id='${_droneName}_crps'></span></div>`;
    jQuery(div).draggable();
}

function updateDroneTelemetryElement(
    _droneName,
    _time,
    _droneState,
    _batteryPercentage,
    _satellites,
    _latitude,
    _longitude,
    _altitude,
    _velocity,
    _heading,
    _gimbalAngle,
    _droneType,
    _droneConfiguration,
    _vtolState,
    _gpsSignal,
    _crpsRequested,
    _crpsResponding,
) {
    const infoBox = document.getElementById("droneInfoBox" + _droneName);
    if (!infoBox || infoBox.style.display === "none") {
        return;
    }
    let date = new Date(_time);
    let formattedTime =
        date.getHours().toString().padStart(2, "0") +
        ":" +
        date.getMinutes().toString().padStart(2, "0") +
        ":" +
        date.getSeconds().toString().padStart(2, "0");
    document.getElementById(_droneName + "_time").textContent = formattedTime;
    document.getElementById(_droneName + "_drone_state").innerHTML = _droneState;
    document.getElementById(_droneName + "_battery_percentage").innerHTML =
        parseFloat(_batteryPercentage).toFixed(0) + "%";
    document.getElementById(_droneName + "_satellites").innerHTML = _satellites;
    document.getElementById(_droneName + "_latitude").innerHTML = parseFloat(_latitude).toFixed(6) + "°";
    document.getElementById(_droneName + "_longitude").innerHTML = parseFloat(_longitude).toFixed(6) + "°";
    document.getElementById(_droneName + "_altitude").innerHTML = parseFloat(_altitude).toFixed(1) + " m";
    document.getElementById(_droneName + "_velocity").innerHTML = parseFloat(_velocity).toFixed(1) + " m/s";
    document.getElementById(_droneName + "_heading").innerHTML = parseFloat(_heading).toFixed(1) + "°";
    document.getElementById(_droneName + "_gimbal_angle").innerHTML = parseFloat(_gimbalAngle).toFixed(1) + "°";
    document.getElementById(_droneName + "_comm_type").innerHTML = _droneType;
    document.getElementById(_droneName + "_configuration").innerHTML = _droneConfiguration;

    if (_droneType == "MAVLINK") {
        document.getElementById(_droneName + "_gps_signal_wrapper").style.display = "none";
        document.getElementById(_droneName + "_crps_wrapper").style.display = "none";
    } else {
        let crpsStatus = "Disengaged";
        if (_crpsRequested == true) {
            statusIcon = "fa fa-sitemap";
            statusIconColor = "orange";
            crpsStatus = "Requested";
        } else if (_crpsResponding == true) {
            statusIcon = "fa fa-sitemap";
            statusIconColor = "#4efcb4";
            crpsStatus = "Responding";
        }
        document.getElementById(_droneName + "_gps_signal").innerHTML = _gpsSignal + "/5";
        document.getElementById(_droneName + "_crps").innerHTML = crpsStatus;
    }

    // show VTOL state only if drone configuration is VTOL
    if (_droneConfiguration == "VTOL") {
        document.getElementById(_droneName + "_vtol_state").innerHTML = _vtolState;
    }
    else {
        document.getElementById(_droneName + "_vtol_state_wrapper").style.display = "none";
    }

    // force hide CRPS
    if(true) {
        document.getElementById(_droneName + "_crps_wrapper").style.display = "none";
    }
}

function deleteDroneTelemetryElement(_droneName) {
    if (document.getElementById("droneInfoBox" + _droneName)) {
        document.getElementById("droneInfoBox" + _droneName).remove();
    }
}
