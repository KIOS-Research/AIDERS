{
	function droneClickInfoBox(toggleId, droneId) {
		postElementId(
			"GPS " + droneId,
			document.getElementById(toggleId).checked
		);
		if (document.getElementById(toggleId).checked) {
			let div = document.createElement("div");
			div.id = "droneInfoBox" + droneId;
			div.classList.add("droneDataBox");
			div.classList.add("overlay-popup");
			div.style.position = 'absolute';
			let offset = getNumberOfOverlayPanels() * 20;
			div.style.top = offset + 'px';
			div.style.left = offset + 'px';			
			document.getElementsByClassName("overlay-section")[0].appendChild(div);
			div.innerHTML =
				`<div> <b>Drone:` + droneId + `</b></div>
				<div><b>Time: </b> <span id='` + droneId + `_time'></span></div>
				<div><b>State: </b> <span id='` + droneId + `_drone_state'></span></div>
				<div><b>Battery: </b><span id='` + droneId + `_battery_percentage'></span></div>
				<div><b>Gps signal: </b> <span id='` + droneId + `_gps_signal'></span></div>
				<div><b>Satellites: </b><span id='` + droneId + `_satellites'></span></div>
				<div><b>Latitude: </b><span id='` + droneId + `_latitude'></span></div>
				<div><b>Longitude: </b><span id='` + droneId + `_longitude'></span></div>
				<div><b>Altitude: </b><span id='` + droneId + `_altitude'></span></div>
				<div><b>Speed: </b><span id='` + droneId + `_velocity'></span></div>
				<div><b>Bearing: </b><span id='` + droneId + `_heading'></span></div>
				<div><b>Gimbal: </b><span id='` + droneId + `_gimbal_angle'></span></div>
				<div><b>CRPS: </b><span id='` + droneId + `_crps'></span></div>`;
			// for (let i = 0; i < g_websocketMessage['drones'].length; i++) {
			//     if (g_websocketMessage['drones'][i]['drone_name'] === droneId) {
			//         if (g_websocketMessage['drones'][i]['water_sampler_available'] === true) {
			//             box.innerHTML = box.innerHTML + `<div><b>Sampler Under Water: </b><span id='` + droneId + `_water_sampler'></span></div>`;
			//         }
			//     }
			// }

			jQuery(div).draggable();

		} else {
			document.getElementById("droneInfoBox" + droneId).remove();
		}
	}

	// function insertAfter(referenceNode, newNode) {
	// 	referenceNode.parentNode.insertBefore(newNode, referenceNode.nextSibling);
	// }

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
				`<div> <b>Weather ` + ` <span id="` + "weatherDataBox" + drone_name + "drone" + `"></span> </b></div>
                <div><b>Time: </b> <span id="` + "weatherDataBox" + drone_name + "time1" + `"></span></div>
                <div><b>Wind Direction: </b> <span id="` + "weatherDataBox" + drone_name + "wDir" + `"></span></div>
                <div><b>Wind Speed:</b> <span id="` + "weatherDataBox" + drone_name + "wSpeed" + `"></span></div>
                <div><b>Temperature: </b> <span id="` + "weatherDataBox" + drone_name + "temp" + `"></span></div>
                <div><b>Pressure: </b><span id="` + "weatherDataBox" + drone_name + "press" + `"></span></div>
                <div><b>Humidity: </b><span id="` + "weatherDataBox" + drone_name + "humidity" + `"></span></div>
                <div><b>Heading: </b><span id="` + "weatherDataBox" + drone_name + "heading" + `"></span></div>`;
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
						weatherObj.windDir = parseFloat(g_websocketMessage["drones"][i]["weather"]["wind_direction"]).toFixed(2);
						weatherObj.windSpeed = parseFloat(g_websocketMessage["drones"][i]["weather"]["wind_speed"]).toFixed(3);
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

	function displayDroneData(droneObj, droneId) {
		let timestamp = droneObj.time;
		let date = new Date(timestamp);
		let formattedTime = date.getHours().toString().padStart(2, '0') + ":" +
			date.getMinutes().toString().padStart(2, '0') + ":" +
			date.getSeconds().toString().padStart(2, '0');	
		document.getElementById(droneId + "_time").textContent = formattedTime;
		document.getElementById(droneId + "_drone_state").innerHTML = droneObj.drone_state;
		document.getElementById(droneId + "_battery_percentage").innerHTML = parseFloat(droneObj.battery_percentage).toFixed(0) + "%";
		document.getElementById(droneId + "_gps_signal").innerHTML = droneObj.gps_signal + "/5";
		document.getElementById(droneId + "_satellites").innerHTML = droneObj.satellites;
		document.getElementById(droneId + "_latitude").innerHTML = parseFloat(droneObj.latitude).toFixed(6) + "°";
		document.getElementById(droneId + "_longitude").innerHTML = parseFloat(droneObj.longitude).toFixed(6) + "°";
		document.getElementById(droneId + "_altitude").innerHTML = parseFloat(droneObj.altitude).toFixed(1) + " m";
		document.getElementById(droneId + "_velocity").innerHTML = parseFloat(droneObj.velocity).toFixed(1) + " m/s";
		document.getElementById(droneId + "_heading").innerHTML = parseFloat(droneObj.heading).toFixed(1) + "°";
		document.getElementById(droneId + "_gimbal_angle").innerHTML = parseFloat(droneObj.gimbal_angle).toFixed(1) + "°";
		document.getElementById(droneId + "_crps").innerHTML = droneObj.crpsStatus;

		for (let i = 0; i < g_websocketMessage["drones"].length; i++) {
			if (g_websocketMessage["drones"][i]["drone_name"] === droneId) {
				if (g_websocketMessage["drones"][i]["water_sampler_available"] === true) {
					ent.getElementById(droneId + "_water_sampler").innerHTML =
						droneObj.under_water;
				}
			}
		}
	}
}
