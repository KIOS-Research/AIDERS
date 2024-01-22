const pointOfInterestMarkerUpdateTimer = 1000;
var pointOfInterestMarkerIntervalTime = null;

function startPointOfInterestMarker() {
	console.log("Start Point Of Interest Marker");
	// Check if the interval is already running to avoid starting multiple intervals
	if (!pointOfInterestMarkerIntervalTime) {
		pointOfInterestMarkerIntervalTime = setInterval(() => {
			updatePointOfInterestMarkers(); // Function to make the POST request
		}, pointOfInterestMarkerUpdateTimer);
	}
}
async function updatePointOfInterestMarkers() {
    // console.log("Updating Point Of Interest Mark")
	let settings = {
		url: "/api/operations/" + CURRENT_OP + "/getLatestManualObjects",
		method: "POST",
		headers: {
			"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
			"Content-Type": "application/json",
		},
		data: JSON.stringify({
			operationId: OPERATION_ID,
		}),
	};
	await $.ajax(settings).done((_response) => {
        MSOUpdate(_response.data)
	});
}
function stopPointOfInterestMarkers(){
    console.log("Stop Point Of Interest Marker");
    // Check if the interval is running before attempting to stop it
    if (pointOfInterestMarkerIntervalTime) {
        clearInterval(pointOfInterestMarkerIntervalTime);
        pointOfInterestMarkerIntervalTime = null;
    }
}