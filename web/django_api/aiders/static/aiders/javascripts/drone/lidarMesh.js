//////////////////
/// Lidar Mesh ///
//////////////////

const LIDAR_MESH_LAYER = "lidar_mesh_session_";
let loadedLidarMeshSession = [];

function updateLidarMeshSessionOnLoadedLidarMeshSessionList(loadedLidarMeshSessionObject) {
	let existingIndex = loadedLidarMeshSession.findIndex(
		(obj) => obj.sessionId === loadedLidarMeshSessionObject.sessionId
	);
	// If the object exists, replace it; otherwise, add it to the list
	if (existingIndex !== -1) {
		loadedLidarMeshSession[existingIndex] = loadedLidarMeshSessionObject;
	} else {
		loadedLidarMeshSession.push(loadedLidarMeshSessionObject);
	}
}
function loadLidarMesh() {
	postElementId("Process Lidar Points", "Click");
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/getLidarSessionWithProcessMesh",
		method: "GET",
	};
	$.ajax(settings).done(function (_response) {
		// Remove loaded Lidar Point sessions
		let availableLidarMeshSessionsNotLoadedYet = JSON.parse(_response).filter(
			(obj2) => !loadedLidarMeshSession.some((obj1) => obj1.sessionId == obj2.sessionId)
		);
		createPopupDialogForLoadClearProcess(
			availableLidarMeshSessionsNotLoadedYet,
			"Select Lidar Point Cloud Sessions to Load"
		).then(function (selectedSessions) {
			selectedSessions.forEach((_session) => {
				requestLidarMeshDataForVisualization(_session.sessionId);
				updateLidarMeshSessionOnLoadedLidarMeshSessionList(_session);
			});
		});
	});
}
function requestLidarMeshDataForVisualization(_sessionId) {
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/getLidarMeshDataNeededForVisualizationBySessionId",
		method: "POST",
		headers: {
			"Content-Type": "application/json",
			"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
		},
		data: JSON.stringify({ sessionId: _sessionId }),
	};
	$.ajax(settings).done(function (_response) {
		loadLidarMeshOnMap(_sessionId, _response.data.latitude, _response.data.longitude, _response.data.path);
	});
}
function loadLidarMeshOnMap(_sessionId, _latitude, _longitude, _filePath) {
	let mesh3dObject;
	if (!map.getLayer(LIDAR_MESH_LAYER + _sessionId)) {
		map.addLayer({
			id: LIDAR_MESH_LAYER + _sessionId,
			type: "custom",
			renderingMode: "3d",
			onAdd: function (map, mbxContext) {
				let options = {
					obj: "media/" + _filePath,
					type: "gltf",
					scale: 1,
					units: "meters",
					rotation: { x: 0, y: 0, z: 180 }, //default rotation
					anchor: "center",
				};

				tb.loadObj(options, function (model) {
					mesh3dObject = model.setCoords([_longitude, _latitude, 0]);
					tb.add(mesh3dObject);
				});
			},
			onRemove: function (map, mbxContext) {
				tb.remove(mesh3dObject);
			},
			render: function (gl, matrix) {
				tb.update();
			},
		});
		map.flyTo({
			center: [_longitude, _latitude],
			zoom: 20,
		});
	}
}

function removeLidarMeshLayerOnTheMapByUsingSessionId(_sessionId) {
	if (map.getLayer(LIDAR_MESH_LAYER + _sessionId)) {
		map.removeLayer(LIDAR_MESH_LAYER + _sessionId);
	}
	if (map.getSource(LIDAR_MESH_LAYER + _sessionId)) {
		map.removeSource(LIDAR_MESH_LAYER + _sessionId);
	}
}

function clearLidarMesh() {
	if (loadedLidarMeshSession.length == 0) {
		create_popup_for_a_little(WARNING_ALERT, "There are no Lidar Mesh Sessions loaded yet.", 3000);
		return;
	}
	createPopupDialogForLoadClearProcess(loadedLidarMeshSession, "Select Lidar Mesh Sessions to Clear").then(function (
		selectedSessions
	) {
		selectedSessions.forEach((session) => {
			removeLidarMeshLayerOnTheMapByUsingSessionId(session.sessionId);
			loadedLidarMeshSession = loadedLidarMeshSession.filter((obj) => obj.sessionId != session.sessionId);
		});
	});
}
