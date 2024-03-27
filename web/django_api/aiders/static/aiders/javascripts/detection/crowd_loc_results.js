
// Array to keep track of data counts for each drone
let dataCounterCrowdLocalization = [];

// Function to check if the coordinates of a map source are empty
function checkIfMapSourceCoordinatesAreEmpty(mapLayerId) {
	const source = map.getSource(mapLayerId);
	return source._data.geometry.coordinates.length === 0;
}

// Function to initialize the map source and layer for a drone
function initializeMapSourceAndLayer(droneName) {
	const sourceId = "crowd-loc-points-for-drone-" + droneName;
	if (!map.getSource(sourceId)) {
		// Add geojson source with empty MultiPoint geometry
		map.addSource(sourceId, {
			type: "geojson",
			data: {
				type: "Feature",
				geometry: {
					type: "MultiPoint",
					coordinates: [],
				},
			},
		});

		// Add a circle layer for the drone's crowd location points
		map.addLayer({
			id: sourceId,
			type: "circle",
			source: sourceId,
			paint: {
				"circle-radius": 3,
				"circle-color": "#F50",
			},
		});
		// Function to update the drone object in the map, ensuring it is rendered above all other objects
		updateAllDroneModelsToStayAbove()
	}
}

// Function to reset the data of a map source and its associated polygon
function resetMapSourceData(droneName) {
	const sourceId = "crowd-loc-points-for-drone-" + droneName;
	const emptyData = {
		type: "Feature",
		geometry: {
			type: "MultiPoint",
			coordinates: [],
		},
	};
	// Set empty data for crowd location points
	map.getSource(sourceId).setData(emptyData);
	const fovSourceId = "crowd-loc-fov-polygon-for-drone-" + droneName;
	const emptyFovData = {
		type: "Feature",
		properties: {
			label: "",
		},
		geometry: {
			type: "Polygon",
			coordinates: [],
		},
	};
	// Set empty data for the drone's field of view polygon
	if (map.getSource(fovSourceId)) {
		map.getSource(fovSourceId).setData(emptyFovData);
	}
}

// Function to update crowd location results on the map
function updatingLoadCrowdLocResultsOnMap(detectionData, droneName) {
	// Initialize map source and layer for the drone
	initializeMapSourceAndLayer(droneName);

	// Check if data counter for the drone is undefined, and initialize it
	if (dataCounterCrowdLocalization[droneName] === undefined) {
		dataCounterCrowdLocalization[droneName] = 0;
	}

	// Check if detection data is undefined
	if (detectionData === undefined) {
		// Increment data counter and check threshold for resetting data
		dataCounterCrowdLocalization[droneName]++;
		const crowdLocSourceId = "crowd-loc-points-for-drone-" + droneName;

		if (dataCounterCrowdLocalization[droneName] > 4 && !checkIfMapSourceCoordinatesAreEmpty(crowdLocSourceId)) {
			// Reset map source data and return
			resetMapSourceData(droneName);
		}
		return;
	}

	// Reset data counter for the drone since new data is available
	dataCounterCrowdLocalization[droneName] = 0;

	// Parse coordinates from detection data
	const coordinates = JSON.parse(detectionData.coordinates);
	const points = coordinates.map((point) => [point[1], point[0]]);

	// Update data for crowd location points on the map
	const newData = {
		type: "Feature",
		geometry: {
			type: "MultiPoint",
			coordinates: points,
		},
	};
	map.getSource("crowd-loc-points-for-drone-" + droneName).setData(newData);

	// Get if exists drone fov layer
	if (map.getLayer("fov-polygon-layer-"+droneName) && map.getLayoutProperty("fov-polygon-layer-"+droneName, 'visibility') == "visible") {
		let fov_coordinates = map.getSource("fov-polygon-" + droneName)._data.geometry.coordinates[0];
		// Update data for FOV polygon on the map
		const newDataFov = {
			type: "Feature",
			properties: {
				label: coordinates.length,
			},
			geometry: {
				type: "Polygon",
				coordinates: [fov_coordinates.concat([fov_coordinates[0]])],
			},
		};

		const fovSourceId = "crowd-loc-fov-polygon-for-drone-" + droneName;

		if (map.getSource(fovSourceId)) {
			map.getSource(fovSourceId).setData(newDataFov);
		} else {
			//If the FOV source doesn't exist, add it to the map
			map.addSource(fovSourceId, {
				type: "geojson",
				data: newDataFov,
			});
			// Add a layer for the FOV polygon
			map.addLayer({
				id: "crowd-loc-fov-polygon-layer-for-drone-" + droneName,
				type: "fill",
				source: fovSourceId,
				layout: {},
				paint: {
					"fill-color": "#FF8888",
					"fill-opacity": 0.3,
				},
			});

			// Add a layer for the FOV polygon label
			map.addLayer({
				id: "crowd-loc-polygon-label-for-drone-" + droneName,
				type: "symbol",
				source: fovSourceId,
				layout: {
					"text-field": ["get", "label"],
					"text-size": 18,
					"text-font": ["Open Sans Bold"],
					"text-allow-overlap": true,
					"text-offset": [0, 0],
				},
				paint: {
					"text-color": "#000",
				},
			});
			// Function to update the drone object in the map, ensuring it is rendered above all other objects
			updateAllDroneModelsToStayAbove()
		}
	}else{
		const newDataFov = {
			type: "Feature",
			properties: {
				label: "",
			},
			geometry: {
				type: "Polygon",
				coordinates: [],
			},
		};

		const fovSourceId = "crowd-loc-fov-polygon-for-drone-" + droneName;

		if (map.getSource(fovSourceId)) {
			map.getSource(fovSourceId).setData(newDataFov);
		}
	}
}

function showOrHideCrowdLocalizationResultsLayer(_show) {
	crowdLocalizationLayers = map.getStyle().layers.filter(layer => layer.id.startsWith("crowd-loc-"))
	if(_show){
		crowdLocalizationLayers.forEach(layer => {
			map.setLayoutProperty(layer.id, 'visibility', 'visible');
		});
	}else{
		crowdLocalizationLayers.forEach(layer => {
			map.setLayoutProperty(layer.id, 'visibility', 'none');
		});
	}
 }
