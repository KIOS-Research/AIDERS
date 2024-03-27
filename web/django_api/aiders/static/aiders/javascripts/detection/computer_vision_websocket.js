{
	const CV_WEB_SOCKET_TIMER = 1000;
	const CV_WEB_SOCKET_ADDRESS =
		"ws://" + window.location.hostname + ":" + WS_PORT + "/getCurrentActiveComputerVisionResultsByOperationId";
	const CROWD_LOCALIZATION = "CrowdLocalization";
	const DISASTER_CLASSIFICATION = "DisasterClassification";
	const VEHICLE_AND_PERSON_TRACKER = "VehicleAndPersonTracker";

	let CvWebSocket;
	let computerVisionWebSocketInterval;

	let crowdLocalizationLoaded = [];
	let disasterLoaded = [];
	let vehicleAndPersonTrackerLoaded = [];
	let activeDetectionTypes = {};

	// Init WebSocket connection
	function initWebsocketForCv(_address) {
		console.log(_address);
		let ws = new WebSocket(_address + "?token=" + encodeURIComponent(TOKEN));
		ws.addEventListener("open", function (event) {
			console.log("CV WebSocket connection established.");
		});
		ws.addEventListener("error", function (event) {
			console.error("CV Socket encountered error: Closing connection");
		});
		ws.addEventListener("message", function (e) {
			handleIncomingCvWebsocketMessage(JSON.parse(e.data));
		});
		return ws;
	}
	// Close WebSocket connection
	function closeWebsocketForCv(_websocket) {
		if (_websocket) {
			_websocket.close();
			console.log("Cv WebSocket connection closed.");
		}

		return undefined;
	}

	function saveOrUpdate(detectedList, droneId, id, droneName) {
		const index = detectedList.findIndex((obj) => obj.droneId === droneId);
		if (index !== -1) {
			// Update the existing object
			detectedList[index].id = id;
		} else {
			// Add a new object
			detectedList.push({ droneId, id, droneName });
		}
	}

	function checkIfDetectionIdExistsInList(_droneId, _list) {
		if (_list == null || _list.length === 0) {
			return false;
		}
		return _list.some((item) => item.droneId === _droneId);
	}

	function detectionHandler(data, loadedData, updateFunction, saveOrUpdateFunction) {
		loadedData.forEach((localDataLoaded) => {
			if (!checkIfDetectionIdExistsInList(localDataLoaded.droneId, data)) {
				updateFunction(undefined, localDataLoaded.droneName);
			}
		});
		if (data == null || data.length === 0) {
			return;
		}
		data.forEach((dataValue) => {
			updateFunction(dataValue, dataValue.droneName);
			saveOrUpdateFunction(loadedData, dataValue.droneId, dataValue.id, dataValue.droneName);
		});
	}
	function vehicleAndPersonTrackerHandler(_vehicleAndPersonTrackerData) {
		if (_vehicleAndPersonTrackerData !== null && _vehicleAndPersonTrackerData.length !== 0) {
			updatingVehicleAndPersonTrackerResultsOnMap(_vehicleAndPersonTrackerData);
		}
	}

	// Receive Web Socket
	function handleIncomingCvWebsocketMessage(_wsMessage) {
		// Crowd Localization
		detectionHandler(
			_wsMessage.crowd_localization,
			crowdLocalizationLoaded,
			updatingLoadCrowdLocResultsOnMap,
			saveOrUpdate
		);
		// Disaster Classification
		detectionHandler(
			_wsMessage.disaster_classification,
			disasterLoaded,
			updatingDisasterClassificationResultsOnMap,
			saveOrUpdate
		);
		// Vehicle And Person Tracker
		vehicleAndPersonTrackerHandler(_wsMessage.vehicle_and_person_tracker);
	}
	// Send Web socket
	function sendComputerVisionWebsocketMessage() {
		if (CvWebSocket.readyState == 1) {
			CvWebSocket.send(
				JSON.stringify({
					operationId: parseInt(OPERATION_ID),
					CrowdLocalizationLoaded: crowdLocalizationLoaded,
					DisasterLoaded: disasterLoaded,
					VehicleAndPersonTrackerLoaded: vehicleAndPersonTrackerLoaded,
					ActiveCrowdLocalization: activeDetectionTypes[CROWD_LOCALIZATION] || false,
					ActiveDisasterClassification: activeDetectionTypes[DISASTER_CLASSIFICATION] || false,
					ActiveVehicleAndPersonTracker: activeDetectionTypes[VEHICLE_AND_PERSON_TRACKER] || false,
				})
			);
		}
	}

	function manageWebsocketForCrowdLocalization(_checked) {
		manageWebsocket(CROWD_LOCALIZATION, _checked);
	}
	function manageWebsocketForDisasterClassification(_checked) {
		manageWebsocket(DISASTER_CLASSIFICATION, _checked);
	}
	function manageWebsocketForVehicleAndPersonTracker(_checked) {
		manageWebsocket(VEHICLE_AND_PERSON_TRACKER, _checked);
	}

	// Function to manage the WebSocket connection based on the detection type and toggle value
	function manageWebsocket(_detectionType, _checked) {
		// Add or remove the detection type from the activeDetectionTypes object
		if (_checked) {
			activeDetectionTypes[_detectionType] = true;
		} else {
			delete activeDetectionTypes[_detectionType];
		}
		// Check if there's at least one active detection
		const hasActiveDetections = Object.keys(activeDetectionTypes).length > 0;

		// If there are active detections, initialize or maintain the WebSocket connection
		if (hasActiveDetections) {
			// If the WebSocket is not initialized or not open, initialize it
			if (CvWebSocket == undefined) {
				CvWebSocket = initWebsocketForCv(CV_WEB_SOCKET_ADDRESS);
				// Start sending messages at regular intervals
				computerVisionWebSocketInterval = setInterval(sendComputerVisionWebsocketMessage, CV_WEB_SOCKET_TIMER);
			}
		} else {
			// No active detections, stop the connection and clear the interval
			clearInterval(computerVisionWebSocketInterval);
			computerVisionWebSocketInterval = undefined;
			CvWebSocket = closeWebsocketForCv(CvWebSocket);
		}
	}
}
