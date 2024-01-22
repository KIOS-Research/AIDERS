{
	function droneClickInfoBox(droneId) {
		postElementId(
			"GPS " + droneId,
			document.getElementById("drone-info-" + droneId).checked
		);
		if (document.getElementById("drone-info-" + droneId).checked) {
			let div = document.createElement("div");
			div.id = "droneInfoBox" + droneId;
			div.classList.add("droneDataBox");
			div.classList.add("overlay-popup");
			document.getElementsByClassName("overlay-section")[0].appendChild(div);
			div.innerHTML =
				`<div> <b>Drone:` +
				droneId +
				`</b></div>
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

			jQuery(div).draggable();
		} else {
			document.getElementById("droneInfoBox" + droneId).remove();
		}
	}

	function insertAfter(referenceNode, newNode) {
		referenceNode.parentNode.insertBefore(newNode, referenceNode.nextSibling);
	}

	function createWeatherBoxForDrone(drone_name) {
		if (document.getElementById("weatherDataBox" + drone_name) === null) {
			let div = document.createElement("div");
			div.setAttribute("id", "weatherDataBox" + drone_name);
			div.setAttribute("class", "weatherDataBox");
			div.style.display = "none";
			document.getElementsByClassName("overlay-section")[0].appendChild(div);
			div.innerHTML =
				`<div> <b>Weather ` +
				drone_name +
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
	function droneClickWeatherBox(droneId) {
		const pressed = document.getElementById("drone-weather-" + droneId).checked;
		postElementId("Weather " + droneId, pressed);
		if (pressed) {
			let clickWeatherBox = $("#weatherDataBox" + droneId);
			clickWeatherBox.toggle();
			clickWeatherBox.draggable();
		} else {
			clearInterval(weatherIntervalList[droneId]);
			let clickWeatherBox = $("#weatherDataBox" + droneId);
			clickWeatherBox.toggle();
		}
	}

	function displayDroneData(droneObj, droneId) {
		document.getElementById(droneId + "_time").textContent = droneObj.time;
		document.getElementById(droneId + "_drone_state").innerHTML =
			droneObj.drone_state;
		document.getElementById(droneId + "_battery_percentage").innerHTML =
			droneObj.battery_percentage + "%";
		document.getElementById(droneId + "_gps_signal").innerHTML =
			droneObj.gps_signal + "/5";
		document.getElementById(droneId + "_satellites").innerHTML =
			droneObj.satellites;
		document.getElementById(droneId + "_latitude").innerHTML =
			parseFloat(droneObj.latitude).toFixed(6) + "°";
		document.getElementById(droneId + "_longitude").innerHTML =
			parseFloat(droneObj.longitude).toFixed(6) + "°";
		document.getElementById(droneId + "_altitude").innerHTML =
			parseFloat(droneObj.altitude).toFixed(1) + " m";
		document.getElementById(droneId + "_velocity").innerHTML =
			parseFloat(droneObj.velocity).toFixed(1) + " m/s";
		document.getElementById(droneId + "_heading").innerHTML =
			parseFloat(droneObj.heading).toFixed(1) + "°";
		document.getElementById(droneId + "_gimbal_angle").innerHTML = parseFloat(
			droneObj.gimbal_angle + "°"
		).toFixed(1);

		for (let i = 0; i < webSocketMessage["drones"].length; i++) {
			if (webSocketMessage["drones"][i]["drone_name"] === droneId) {
				if (webSocketMessage["drones"][i]["water_sampler_available"] === true) {
					ent.getElementById(droneId + "_water_sampler").innerHTML =
						droneObj.under_water;
				}
			}
		}
	}
}
