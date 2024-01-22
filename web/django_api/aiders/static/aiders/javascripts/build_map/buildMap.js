const droneMinimumAltitude = 25;
const buildMapTimer = 1000;

let liveBuildMap = {};
let loadedBuildMapSession = [];

class BuildMap {
	constructor(_drone, _sessionId) {
		this.drone = _drone;
		this.sessionId = _sessionId;
		this.intervalTime = null;
		this.latestImageId = 0;
		this.loadedBuildMapSessionObject = {
			sessionId: _sessionId,
			droneName: _drone.droneID,
			startTime: getCurrentFormattedTime(),
			endTime: "∞",
			count: 0,
		};
	}
	start() {
		// Check if the interval is already running to avoid starting multiple intervals
		if (!this.intervalTime) {
			this.intervalTime = setInterval(() => {
				this.update(); // Function to make the POST request
			}, buildMapTimer);
		}
	}
	stop() {
		// Check if the interval is running before attempting to stop it
		if (this.intervalTime) {
			clearInterval(this.intervalTime);
			this.intervalTime = null;
			this.loadedBuildMapSessionObject.endTime = getCurrentFormattedTime();
			updateBuildMapSessionOnLoadedBuildMapSessionList(this.loadedBuildMapSessionObject);
		}
	}
	async update() {
		let droneId = this.drone.dronePK;
		let sessionId = this.sessionId;
		let latestImageId = this.latestImageId;
		let count = this.loadedBuildMapSessionObject.count;
		let settings = {
			url: "/api/operations/" + CURRENT_OP + "/getActiveBuildMapSessionImages",
			method: "POST",
			headers: {
				"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
				"Content-Type": "application/json",
			},
			data: JSON.stringify({
				droneId: droneId,
				latestReceivedImageId: latestImageId,
			}),
		};
		// Get the latest Images from Back-end
		await $.ajax(settings).done(function (_response) {
			let buildMapImages = _response.data;
			buildMapImages.forEach((image) => {
				let updateLatestImageId = addBuildMapImageLayerOnTheMap(image, sessionId);
				if (updateLatestImageId) {
					count = count + 1;
					latestImageId = image["id"];
				}
			});
		});
		this.latestImageId = latestImageId;
		this.loadedBuildMapSessionObject.count = count;
	}
}

function checkIfDronesAreSelected(_selectedDrones) {
	if (_selectedDrones.length < 1) {
		create_popup_for_a_little(WARNING_ALERT, "No Drone Found", 3000);
		return false;
	}
	return true;
}

function checkDroneAltitude(_droneName, _droneAltitude) {
	if (_droneAltitude < droneMinimumAltitude) {
		create_popup_for_a_little(
			WARNING_ALERT,
			"Drone " + _droneName + " altitude is " + _droneAltitude + " and needs to be above " + droneMinimumAltitude,
			3000
		);
		return false;
	}
	return true;
}

function updateBuildMapSessionOnLoadedBuildMapSessionList(loadedBuildMapSessionObject) {
	let existingIndex = loadedBuildMapSession.findIndex((obj) => obj.sessionId === loadedBuildMapSessionObject.sessionId);

	// If the object exists, replace it; otherwise, add it to the list
	if (existingIndex !== -1) {
		loadedBuildMapSession[existingIndex] = loadedBuildMapSessionObject;
	} else {
		loadedBuildMapSession.push(loadedBuildMapSessionObject);
	}
}

async function postStartOrStopBuildMapRequest(_droneName, _command) {
	// TODO: Multispectral update change start_multispectral_build_map
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/buildMapStartOrStop",
		method: "POST",
		headers: {
			"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
		},
		data: {
			drone_name: _droneName,
			start_build_map_boolean: _command,
			overlap: document.getElementById("overlapValue").innerHTML,
			start_multispectral_build_map: false,
		},
	};
	let sessionId;
	await $.ajax(settings).done(function (response) {
		sessionId = response.data;
	});
	return sessionId;
}

function addBuildMapImageLayerOnTheMap(_image, _sessionId) {
	let imageSourceId = "sourceBuildMapImage_" + _sessionId + "_" + _image["id"];
	let imageLayerId = "layerBuildMapImage_" + _sessionId + "_" + _image["id"];
	if (map.getSource(imageSourceId)) return false;
	let imageMediaUrlPath = "/media/" + _image["path"];
	map.addSource(imageSourceId, {
		type: "image",
		url: imageMediaUrlPath,
		coordinates: [
			[_image["top_right"][0], _image["top_right"][1]],
			[_image["bottom_right"][0], _image["bottom_right"][1]],
			[_image["bottom_left"][0], _image["bottom_left"][1]],
			[_image["top_left"][0], _image["top_left"][1]],
		],
	});
	map.addLayer({
		id: imageLayerId,
		type: "raster",
		source: imageSourceId,
		paint: { "raster-opacity": 1 },
	});
	updateAllDroneModelsToStayAbove();
	return true;
}

function removeImagesLayerOnTheMapFromByUsingSessionId(_sessionId) {
	map.getStyle().layers.forEach(function (layer) {
		if (layer.id.startsWith("layerBuildMapImage_" + _sessionId + "_")) {
			map.removeLayer(layer.id);
		}
	});
	Object.keys(map.getStyle().sources).forEach(function (sourceId) {
		if (sourceId.startsWith("sourceBuildMapImage_" + _sessionId + "_")) {
			map.removeSource(sourceId);
		}
	});
}

function getImagesByUsingSessionId(_session) {
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/getBuildMapImagesBySessionId",
		method: "POST",
		headers: {
			"Content-Type": "application/json",
			"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
		},
		data: JSON.stringify({
			buildMapSessionId: _session.sessionId,
		}),
	};
	$.ajax(settings).done(function (_response) {
		let buildMapImages = _response.data;
		buildMapImages.forEach((image) => {
			addBuildMapImageLayerOnTheMap(image, _session.sessionId);
		});
		updateBuildMapSessionOnLoadedBuildMapSessionList(_session);
	});
}

function getCurrentFormattedTime() {
	const now = new Date();

	const year = now.getFullYear();
	const month = (now.getMonth() + 1).toString().padStart(2, "0"); // Months are zero-based
	const day = now.getDate().toString().padStart(2, "0");
	const hours = now.getHours().toString().padStart(2, "0");
	const minutes = now.getMinutes().toString().padStart(2, "0");
	const seconds = now.getSeconds().toString().padStart(2, "0");

	const formattedTime = `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
	return formattedTime;
}

async function checkIfDroneBuildMapIsActivatedOrDeactivated(_drone, _activate) {
	if (_activate && liveBuildMap[_drone.droneID] === undefined) {
		create_popup_for_a_little(SUCCESS_ALERT, "Starting Build Map for drone " + _drone.droneID, 3000);
		let activatedSessionId = await postStartOrStopBuildMapRequest(_drone.droneID, true);
		liveBuildMap[_drone.droneID] = new BuildMap(_drone, activatedSessionId);
		liveBuildMap[_drone.droneID].start();
	} else if (!_activate && liveBuildMap[_drone.droneID] !== undefined) {
		create_popup_for_a_little(SUCCESS_ALERT, "Stopping Build Map for drone " + _drone.droneID, 3000);
		liveBuildMap[_drone.droneID].stop();
		delete liveBuildMap[_drone.droneID];
	}
}

function startDroneBuildMap() {
	let selectedDrones = get_selected_drones();
	if (!checkIfDronesAreSelected(selectedDrones)) return;
	selectedDrones.forEach(async function (drone) {
		droneAltitude = drone.droneInfo.currentCoordinate[2];
		// Check if build map for this drone is already active
		if (liveBuildMap[drone.droneID] !== undefined) return;
		// Check the Altitude of the drone
		if (!checkDroneAltitude(drone.droneID, droneAltitude)) return;
		create_popup_for_a_little(SUCCESS_ALERT, "Starting Build Map for drone " + drone.droneID, 3000);
		activatedSessionId = await postStartOrStopBuildMapRequest(drone.droneID, true);
		liveBuildMap[drone.droneID] = new BuildMap(drone, activatedSessionId);
		liveBuildMap[drone.droneID].start();
	});
	postElementId("Start Build Map", "Click");
}

function stopDroneBuildMap() {
	let selectedDrones = get_selected_drones();
	if (!checkIfDronesAreSelected(selectedDrones)) return;
	selectedDrones.forEach(function (drone) {
		// Check if build map for this drone is not active
		if (liveBuildMap[drone.droneID] === undefined) return;
		create_popup_for_a_little(SUCCESS_ALERT, "Stopping Build Map for drone " + drone.droneID, 3000);
		postStartOrStopBuildMapRequest(drone.droneID, false);
		liveBuildMap[drone.droneID].stop();
		delete liveBuildMap[drone.droneID];
	});
	postElementId("Stop Build Map", "Click");
}
function loadDroneBuildMap() {
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/getBuildMapSessions",
		method: "GET",
	};
	$.ajax(settings).done(function (_response) {
		// Remove loaded build map sessions
		let availableBuildMapSessionsNotLoadedYet = JSON.parse(_response).filter(
			(obj2) => !loadedBuildMapSession.some((obj1) => obj1.sessionId == obj2.sessionId)
		);

		createPopupDialogForLoadClearProcess(availableBuildMapSessionsNotLoadedYet, "Select Build Map Sessions to Load").then(function (
			selectedSessions
		) {
			selectedSessions.forEach((session) => {
				getImagesByUsingSessionId(session);
			});
		});
	});
}
function clearDroneBuildMap() {
	if (loadedBuildMapSession.length == 0) {
		create_popup_for_a_little(WARNING_ALERT, "There are no build map sessions loaded yet.", 3000);
		return;
	}
	createPopupDialogForLoadClearProcess(loadedBuildMapSession, "Select Build Map Sessions to Clear").then(function (selectedSessions) {
		selectedSessions.forEach((session) => {
			removeImagesLayerOnTheMapFromByUsingSessionId(session.sessionId);
			loadedBuildMapSession = loadedBuildMapSession.filter((obj) => obj.sessionId != session.sessionId);
		});
	});
}
