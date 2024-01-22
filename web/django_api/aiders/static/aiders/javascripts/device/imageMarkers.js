const imageMarkerUpdateTimer = 1000;
const maxShownImages = 50;

let liveDeviceImageMarkers = {};

class ImageMarker {
	constructor(_deviceId) {
		this.deviceId = _deviceId;
		this.intervalTime = null;
		this.latestReceiveImageId = 0;
		this.localImageMarkers = [];
	}
	start() {
		console.log("Start Device", this.deviceId);
		// Check if the interval is already running to avoid starting multiple intervals
		if (!this.intervalTime) {
			this.intervalTime = setInterval(() => {
				this.update(); // Function to make the POST request
			}, imageMarkerUpdateTimer);
		}
	}
	stop() {
		console.log("Stop Device", this.deviceId);
		// Check if the interval is running before attempting to stop it
		if (this.intervalTime) {
			clearInterval(this.intervalTime);
			this.intervalTime = null;
		}
	}
	async update() {
		let latestReceivedImageId = this.latestReceiveImageId;
		let localImageMarkers = this.localImageMarkers;
		let settings = {
			url: "devices/getActiveDeviceSessionImages",
			method: "POST",
			headers: {
				"X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
				"Content-Type": "application/json",
			},
			data: JSON.stringify({
				deviceId: this.deviceId,
				latestReceivedImageId: this.latestReceiveImageId,
				maxShownImages: maxShownImages,
			}),
		};
		await $.ajax(settings).done((_response) => {
			let images = _response.data;
			images.forEach((image) => {
				localImageMarkers.push(this.createImageMarkerOnMap(image));
				if (localImageMarkers.length > maxShownImages) {
					localImageMarkers[0].remove();
					localImageMarkers.shift();
				}
				if (image.id > latestReceivedImageId) {
					latestReceivedImageId = image.id;
				}
			});
		});
		this.latestReceiveImageId = latestReceivedImageId;
		this.localImageMarkers = localImageMarkers;
	}
	createImageMarkerOnMap(_image) {
		var path = "/media/" + _image["path"];
		var popup = new maplibregl.Popup({ offset: 10, maxWidth: "inherit" }).setHTML(
			"<div class='device-image-popup'><img width='400' height='auto' style='padding-bottom:5px;' src='" +
				path +
				"'></img>" +
				"<div class='row'>" +
				"<div class='col-md-1' style='text-align: left;'><a href='" +
				path +
				"' target='_blank' style='outline: none;'><i class='fa-solid fa-arrow-up-right-from-square'></i></a></div>" +
				"<div class='col-md-8' style='text-align: center;'><i class='fa fa-location-dot' style='font-size: 15px'></i> " +
				_image["latitude"].toFixed(12) +
				", " +
				_image["longitude"].toFixed(12) +
				"</div>" +
				"<div class='col-md-3' style='text-align: right;'><i class='fa-regular fa-clock' style='font-size: 15px'></i> " +
				_image["time"] +
				"</div>" +
				"</div>" +
				"</div>"
		);
		// create a DOM element for the marker
		var el = document.createElement("div");
		el.className = "marker";
		var markerImageUrl = "/static/aiders/imgs/marker_images.jpg";
		el.style.backgroundImage = "url(" + markerImageUrl + ")";
		el.style.backgroundSize = "35px 35px";
		el.style.width = "35px";
		el.style.height = "35px";

		// add marker to map
		return new maplibregl.Marker(el).setLngLat([_image.longitude, _image.latitude]).setPopup(popup).addTo(map);
	}
}

function startDeviceImageMarkers(_deviceId) {
	if (liveDeviceImageMarkers[_deviceId] === undefined) {
		liveDeviceImageMarkers[_deviceId] = new ImageMarker(_deviceId);
	}
	liveDeviceImageMarkers[_deviceId].start();
}

function stopDeviceImageMarkers(_deviceId) {
	liveDeviceImageMarkers[_deviceId].stop();
}
