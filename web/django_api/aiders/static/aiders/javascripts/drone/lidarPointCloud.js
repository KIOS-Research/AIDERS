{
	const LIDAR_POINT_TIMER = 1000;
	const LIDAR_POINTS_PER_REQUEST = 50000;
	const LIDAR_WEB_SOCKET_ADDRESS = "ws://" + window.location.hostname + ":" + WS_PORT + "/getLidarPointsBySessionId";
	const LIDAR_POINTS_LAYER = "lidar_points_session_";

	let LIDAR_WEB_SOCKET;
	let lidarRequestingSessions = {};
	let loadedLidarPointSession = [];

	function initWebsocketForLidar(_address) {
		console.log(_address);
		let ws = new WebSocket(_address + '?token=' + encodeURIComponent(TOKEN));
		ws.addEventListener("open", function (event) {
			console.log("Lidar WebSocket connection established.");
		});
		ws.addEventListener("error", function (event) {
			console.error("Lidar Socket encountered error: Closing connection");
		});
		ws.addEventListener("message", function (e) {
			handleIncomingLidarWebsocketMessage(JSON.parse(e.data));
		});
		return ws;
	}

	function closeWebsocketForLidar(_websocket) {
		_websocket.close();
		console.log("Lidar WebSocket connection closed.");
		return undefined;
	}

	//////////////////
	// Lidar Points //
	//////////////////

	function handleIncomingLidarWebsocketMessage(_wsMessage) {
		let lidarSessionId = _wsMessage.lidar_session_id;
		let lidarPoints = _wsMessage.lidar_points;
		let lidarLocation = _wsMessage.lidar_origin_coordinates;
		if (lidarPoints === null) {
			if (!lidarRequestingSessions[lidarSessionId].getIsLive) {
				lidarRequestingSessions[lidarSessionId].stop();
			}
			return;
		}
		if (lidarLocation !== null) {
			lidarRequestingSessions[lidarSessionId].setLocation(lidarLocation.latitude, lidarLocation.longitude);
		}
		loadLidarPointOnMap(lidarPoints, lidarSessionId, lidarRequestingSessions[lidarSessionId].getLocation());
		lidarRequestingSessions[lidarSessionId].setLatestPointId(lidarPoints[lidarPoints.length - 1].id);
		lidarRequestingSessions[lidarSessionId].setCountNumberOfPoints(
			lidarRequestingSessions[lidarSessionId].getCountNumberOfPoints() + lidarPoints.length
		);
	}

	class LidarPoints {
		constructor(_lidarSessionId, _droneName, _isLive) {
			this.lidarSessionId = _lidarSessionId;
			this._isLive = _isLive;
			this.latestPointId = 0;
			this.longitude = null;
			this.latitude = null;
			this.intervalTime = null;
			this.loadedLidarPointSessionObject = {
				sessionId: _lidarSessionId,
				droneName: _droneName,
				startTime: getCurrentFormattedTime(),
				endTime: "∞",
				count: 0,
			};
		}
		start() {
			if (LIDAR_WEB_SOCKET === undefined) {
				LIDAR_WEB_SOCKET = initWebsocketForLidar(LIDAR_WEB_SOCKET_ADDRESS);
			}
			// Check if the interval is already running to avoid starting multiple intervals
			if (!this.intervalTime) {
				this.intervalTime = setInterval(() => {
					this.update(); // Function to make the POST request
				}, LIDAR_POINT_TIMER);
			}
		}
		stop() {
			// Check if the interval is running before attempting to stop it
			if (this.intervalTime) {
				clearInterval(this.intervalTime);
				this.intervalTime = null;
				this.loadedLidarPointSessionObject.endTime = getCurrentFormattedTime();
				updateLidarPointSessionOnLoadedLidarPointSessionList(this.loadedLidarPointSessionObject);
				LIDAR_WEB_SOCKET = closeWebsocketForLidar(LIDAR_WEB_SOCKET);
			}
		}
		update() {
			if (LIDAR_WEB_SOCKET === undefined) {
				LIDAR_WEB_SOCKET = initWebsocketForLidar(LIDAR_WEB_SOCKET_ADDRESS);
			}
			if (LIDAR_WEB_SOCKET.readyState == 1) {
				LIDAR_WEB_SOCKET.send(
					JSON.stringify({
						lidar_session_id: this.lidarSessionId,
						latest_point_id: this.latestPointId,
						number_of_points: LIDAR_POINTS_PER_REQUEST,
					})
				);
			}
		}
		getIsLive() {
			return this._isLive;
		}
		setLatestPointId(_latestPointId) {
			this.latestPointId = _latestPointId;
		}
		setLocation(_latitude, _longitude) {
			this.latitude = _latitude;
			this.longitude = _longitude;
		}
		getLocation() {
			return [this.longitude, this.latitude];
		}
		setCountNumberOfPoints(_countNumberOfPoints) {
			this.loadedLidarPointSessionObject.count = _countNumberOfPoints;
		}
		getCountNumberOfPoints() {
			return this.loadedLidarPointSessionObject.count;
		}
	}

	function createMapLayer(_lidarPoints, _lidarSessionId, _coordinates, maxZ, minZ) {
		const { MapboxLayer, PointCloudLayer, COORDINATE_SYSTEM } = deck;
		return new MapboxLayer({
			id: LIDAR_POINTS_LAYER + _lidarSessionId,
			type: PointCloudLayer,
			coordinateSystem: COORDINATE_SYSTEM.METER_OFFSETS,
			coordinateOrigin: _coordinates,
			data: _lidarPoints,
			getPosition: (d) => [d.y, d.x, -d.z],
			getColor: (d) => {
				// Normalize z values to [0, 1] for color mapping
				let min_z = minZ;
				let max_z = maxZ;
				let normalized_z = (d.z - min_z) / (max_z - min_z);

				// Calculate R, G, B values
				let r, g, b;
				if (normalized_z < 0.3) {
					// Transition from red to green in the first half of the range
					r = (1 - 2 * normalized_z) * 255;
					g = 2 * normalized_z * 255;
					b = 0;
				} else {
					// Transition from green to blue in the second half of the range
					r = 0;
					g = (2 - 2 * normalized_z) * 255;
					b = (2 * normalized_z - 1) * 255;
				}

				// Return color as [R, G, B]
				return [r, g, b];
			},
			sizeUnits: "meters",
			pointSize: 0.5,
			opacity: 0.8,
		});
	}
	function loadLidarPointOnMap(_lidarPoints, _lidarSessionId, _coordinates) {
		var maxZ = _lidarPoints[0].z;
		var minZ = _lidarPoints[0].z;

		// Iterate through the list to find max and min 'z' values
		for (var i = 1; i < _lidarPoints.length; i++) {
			var currentZ = _lidarPoints[i].z;
			// Update max if currentZ is greater
			if (currentZ > maxZ) {
				maxZ = currentZ;
			}
			// Update min if currentZ is smaller
			if (currentZ < minZ) {
				minZ = currentZ;
			}
		}
		if (!map.getLayer(LIDAR_POINTS_LAYER + _lidarSessionId)) {
			pointCloudLayer = createMapLayer(_lidarPoints, _lidarSessionId, _coordinates, maxZ, minZ);
			map.flyTo({
				center: _coordinates,
				zoom: 20,
			});
		} else {
			pointCloudLayer = createMapLayer(
				[...map.getLayer(LIDAR_POINTS_LAYER + _lidarSessionId).implementation.props.data, ..._lidarPoints],
				_lidarSessionId,
				_coordinates,
				maxZ,
				minZ
			);
			map.removeLayer(LIDAR_POINTS_LAYER + _lidarSessionId);
		}
		map.addLayer(pointCloudLayer);
	}

	function getLidarPointsByUsingSessionId(_session, _latestLoadedPointId, _coordinates) {
		let settings = {
			url: "/api/operations/" + CURRENT_OP + "/getLidarSessionPointsBySessionId",
			method: "POST",
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
			},
			data: JSON.stringify({
				lidar_point_session_id: _session.sessionId,
				latest_point_id: _latestLoadedPointId,
				number_of_points: LIDAR_POINTS_PER_REQUEST,
			}),
		};
		$.ajax(settings).done(function (_response) {
			let lidarPoints = _response.data;
			// If there are no points left stop the requesting
			if (lidarPoints[0].length === 0) {
				return;
			}
			// Set origin coordinates
			if (_response.coordinates !== undefined) {
				_coordinates = _response.coordinates;
				updateLidarPointSessionOnLoadedLidarPointSessionList(_session);
			}
			loadLidarPointOnMap(lidarPoints[0], _session.sessionId, _coordinates);
			_latestLoadedPointId = lidarPoints[0][lidarPoints[0].length - 1].id;
			getLidarPointsByUsingSessionId(_session, _latestLoadedPointId, _coordinates);
		});
	}

	function removeLidarPointLayerOnTheMapByUsingSessionId(_sessionId) {
		if (map.getLayer(LIDAR_POINTS_LAYER + _sessionId)) {
			map.removeLayer(LIDAR_POINTS_LAYER + _sessionId);
		}
		if (map.getSource(LIDAR_POINTS_LAYER + _sessionId)) {
			map.removeSource(LIDAR_POINTS_LAYER + _sessionId);
		}
	}

	function updateLidarPointSessionOnLoadedLidarPointSessionList(loadedLidarPointSessionObject) {
		let existingIndex = loadedLidarPointSession.findIndex(
			(obj) => obj.sessionId === loadedLidarPointSessionObject.sessionId
		);
		// If the object exists, replace it; otherwise, add it to the list
		if (existingIndex !== -1) {
			loadedLidarPointSession[existingIndex] = loadedLidarPointSessionObject;
		} else {
			loadedLidarPointSession.push(loadedLidarPointSessionObject);
		}
	}

	function startLidarPoints(_droneName) {
		postElementId("Start Lidar " + _droneName, "Click");
		const data = {
			droneName: _droneName,
			operationName: CURRENT_OP,
			command: "START",
		};
		const csrfToken = document.getElementById("csrf").querySelector("input").value;
		fetch("/api/operations/" + CURRENT_OP + "/drones/" + _droneName + "/lidarStartOrStop", {
			method: "POST",
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": csrfToken,
			},
			body: JSON.stringify(data),
		})
			.then((response) => {
				return response.json();
			})
			.then((responseData) => {
				create_popup_for_a_little(SUCCESS_ALERT, responseData.message, 2000);
				lidarRequestingSessions[responseData.lidar_session_id] = new LidarPoints(
					responseData.lidar_session_id,
					_droneName,
					true
				);
				lidarRequestingSessions[responseData.lidar_session_id].start();
			})
			.catch((error) => {
				console.error("Fetch error:", error);
			});
	}

	function stopLidarPoints(_droneName) {
		postElementId("Stop Lidar " + _droneName, "Click");
		const data = {
			droneName: _droneName,
			operationName: CURRENT_OP,
			command: "STOP",
		};
		const csrfToken = document.getElementById("csrf").querySelector("input").value;
		fetch("/api/operations/" + CURRENT_OP + "/drones/" + _droneName + "/lidarStartOrStop", {
			method: "POST",
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": csrfToken,
			},
			body: JSON.stringify(data),
		})
			.then((response) => {
				return response.json();
			})
			.then((responseData) => {
				create_popup_for_a_little(SUCCESS_ALERT, responseData.message, 2000);
				if (
					responseData.lidar_session_id === undefined ||
					lidarRequestingSessions[responseData.lidar_session_id] === undefined
				) {
					return;
				}
				lidarRequestingSessions[responseData.lidar_session_id].stop();
				delete lidarRequestingSessions[responseData.lidar_session_id];
			})
			.catch((error) => {
				console.error("Fetch error:", error);
			});
	}

	function loadLidarPoints() {
		postElementId("Load Lidar Points", "Click");
		let settings = {
			url: "/api/operations/" + CURRENT_OP + "/getLidarSessionOfPoints",
			method: "GET",
		};
		$.ajax(settings).done(function (_response) {
			// Remove loaded Lidar Point sessions
			let availableLidarPointSessionsNotLoadedYet = JSON.parse(_response).filter(
				(obj2) => !loadedLidarPointSession.some((obj1) => obj1.sessionId == obj2.sessionId)
			);
			createPopupDialogForLoadClearProcess(
				availableLidarPointSessionsNotLoadedYet,
				"Select Lidar Point Cloud Sessions to Load"
			).then(function (selectedSessions) {
				selectedSessions.forEach((session) => {
					getLidarPointsByUsingSessionId(session, 0, undefined);
				});
			});
		});
	}
	function clearLidarPoints() {
		if (loadedLidarPointSession.length == 0) {
			create_popup_for_a_little(WARNING_ALERT, "There are no Lidar Point Cloud Sessions loaded yet.", 3000);
			return;
		}
		createPopupDialogForLoadClearProcess(loadedLidarPointSession, "Select Lidar Point Cloud Sessions to Clear").then(
			function (selectedSessions) {
				selectedSessions.forEach((session) => {
					removeLidarPointLayerOnTheMapByUsingSessionId(session.sessionId);
					loadedLidarPointSession = loadedLidarPointSession.filter((obj) => obj.sessionId != session.sessionId);
				});
			}
		);
	}

	//////////////////////////////////////
	/// Lidar Process Points to a Mesh ///
	//////////////////////////////////////

	function processLidarPointsToMesh() {
		postElementId("Process Lidar Points", "Click");
		let settings = {
			url: "/api/operations/" + CURRENT_OP + "/getLidarSessionPointsThatAreNotProcessed",
			method: "GET",
		};
		$.ajax(settings).done(function (_response) {
			createPopupDialogForLoadClearProcess(JSON.parse(_response), "Select Lidar Point Cloud Sessions to Process").then(
				function (selectedSessions) {
					selectedSessions.forEach((session) => {
						requestLidarProcessingBySessionId(session.sessionId);
					});
				}
			);
		});
	}

	function requestLidarProcessingBySessionId(_sessionId) {
		let settings = {
			url: "/api/operations/" + CURRENT_OP + "/processLidarSessionPointsBySessionId",
			method: "POST",
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
			},
			data: JSON.stringify({ sessionId: _sessionId }),
		};
		$.ajax(settings).done(function (_response) {
			create_popup_for_a_little(SUCCESS_ALERT, _response.message, 3000);
		});
	}
}
