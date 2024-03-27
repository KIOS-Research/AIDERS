// URL Paths for disaster images
const EMPTY_IMAGE_PATH = "/static/aiders/imgs/empty.png";
const FIRE_IMAGE_PATH = "/static/aiders/imgs/fire.png";
const EARTHQUAKE_IMAGE_PATH = "/static/aiders/imgs/earthquake.png";
const FLOOD_IMAGE_PATH = "/static/aiders/imgs/flood.png";

// Map Image names for disaster classifications
const MAP_DISASTER_CLASSIFICATION_IMAGE_EMPTY = "disaster-icon-empty";
const MAP_DISASTER_CLASSIFICATION_IMAGE_FIRE = "disaster-icon-fire";
const MAP_DISASTER_CLASSIFICATION_IMAGE_EARTHQUAKE = "disaster-icon-earthquake";
const MAP_DISASTER_CLASSIFICATION_IMAGE_FLOOD = "disaster-icon-flood";

// Counter and flag for image initialization
let dataCounterDisasterClassification = [];
let imagesInitialized = false;

// Function to initialize disaster images
function initializeDisasterImages() {
	map.loadImage(EMPTY_IMAGE_PATH, function (error, image) {
		map.addImage(MAP_DISASTER_CLASSIFICATION_IMAGE_EMPTY, image);
	});
	map.loadImage(FIRE_IMAGE_PATH, function (error, image) {
		map.addImage(MAP_DISASTER_CLASSIFICATION_IMAGE_FIRE, image);
	});
	map.loadImage(EARTHQUAKE_IMAGE_PATH, function (error, image) {
		map.addImage(MAP_DISASTER_CLASSIFICATION_IMAGE_EARTHQUAKE, image);
	});
	map.loadImage(FLOOD_IMAGE_PATH, function (error, image) {
		map.addImage(MAP_DISASTER_CLASSIFICATION_IMAGE_FLOOD, image);
	});
	imagesInitialized = true;
}

// Function to check if the disaster classification layer is hidden
function checkIfLayerIsHidden(droneId) {
	const layerIconImage = map.getLayoutProperty("disaster-class-result-for-drone-" + droneId, "icon-image");
	return layerIconImage === MAP_DISASTER_CLASSIFICATION_IMAGE_EMPTY;
}

// Function to update the image and its position
function updateImageAndPosition(mapImage, coordinates, droneId) {
	map.getSource("disaster-class-result-for-drone-" + droneId).setData({
		type: "Feature",
		geometry: {
			type: "Point",
			coordinates: coordinates,
		},
	});
	map.setLayoutProperty("disaster-class-result-for-drone-" + droneId, "icon-image", mapImage);
}


// Function to update disaster classification results on the map
function updatingDisasterClassificationResultsOnMap(detectionData, droneId) {
	// Initialize disaster images if not already done
	if (!imagesInitialized) {
		initializeDisasterImages();
	}

	const sourceName = "disaster-class-result-for-drone-" + droneId;

	// Add source and layer if not already present
	if (!map.getSource(sourceName)) {
		map.addSource(sourceName, {
			type: "geojson",
			data: {
				type: "Feature",
				geometry: {
					type: "Point",
					coordinates: [0, 0], // Initial coordinates
				},
			},
		});

		map.addLayer({
			id: sourceName,
			type: "symbol",
			source: sourceName,
			layout: {
				"icon-image": MAP_DISASTER_CLASSIFICATION_IMAGE_EMPTY,
				"icon-size": 0.25,
			},
		});
		// Function to update the drone object in the map, ensuring it is rendered above all other objects
		updateAllDroneModelsToStayAbove()
	}

	// Initialize data counter for the drone
	dataCounterDisasterClassification[droneId] = dataCounterDisasterClassification[droneId] || 0;

	// No data to update
	if (detectionData === undefined) {
		dataCounterDisasterClassification[droneId]++;
		if (dataCounterDisasterClassification[droneId] > 4 && !checkIfLayerIsHidden(droneId)) {
			updateImageAndPosition(MAP_DISASTER_CLASSIFICATION_IMAGE_EMPTY, [0, 0], droneId);
		}
		return;
	}

	// Reset data counter since new data is available
	dataCounterDisasterClassification[droneId] = 0;
	lastDisasterClassificationResultId = detectionData.id;

	let mapImage = "";

	// Determine the appropriate disaster image based on probabilities
	if (detectionData.fireProbability > 50) {
		mapImage = MAP_DISASTER_CLASSIFICATION_IMAGE_FIRE;
	} else if (detectionData.earthquakeProbability > 50) {
		mapImage = MAP_DISASTER_CLASSIFICATION_IMAGE_EARTHQUAKE;
	} else if (detectionData.floodProbability > 50) {
		mapImage = MAP_DISASTER_CLASSIFICATION_IMAGE_FLOOD;
	}

	// Update image and position on the map
	updateImageAndPosition(mapImage, [detectionData.longitude, detectionData.latitude], droneId);
}

function showOrHideDisasterClassificationResultsLayer(_show){
	disasterClassificationLayers = map.getStyle().layers.filter(layer => layer.id.startsWith("disaster-class-result-for-drone-"))
	if(_show){
		disasterClassificationLayers.forEach(layer => {
			map.setLayoutProperty(layer.id, 'visibility', 'visible');
		});
	}else{
		disasterClassificationLayers.forEach(layer => {
			map.setLayoutProperty(layer.id, 'visibility', 'none');
		});
	}
}