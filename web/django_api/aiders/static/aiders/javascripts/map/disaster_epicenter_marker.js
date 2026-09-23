
function createBooMarker(map){
    if (BOO_LATITUDE !== "" || BOO_LONGITUDE !== "" ) {
    // create the popup
    var popup = new maplibregl.Popup({ offset: 0, maxWidth: 'inherit' }).setHTML(
        `<div class="disaster-epicenter-popup">
        <div><b>Base of Operations</b></div>
        <div>Latitude: ${BOO_LATITUDE} </div>
        <div>Longutude: ${BOO_LONGITUDE}</div>
        </div>`
    );
    // create a DOM element for the marker
    var el = document.createElement('div');
    el.className = 'marker';
    el.style.backgroundImage = 'url(' + dutils.urls.resolve('static_images', { file_name: 'boo-icon.png' }) + ')';
    el.style.backgroundSize = '35px 35px';
    el.style.width = '35px';
    el.style.height = '35px';

    // add marker to map
    let booMarker = new maplibregl.Marker(el).setLngLat([BOO_LONGITUDE, BOO_LATITUDE]).setPopup(popup).addTo(map);
    }
}

function createDisasterEpicenterMarker(map){
    if (DISASTER_EPICENTER_LATITUDE !== "" || DISASTER_EPICENTER_LONGITUDE !== "" ) {
    // create the popup
    var popup = new maplibregl.Popup({ offset: 0, maxWidth: 'inherit' }).setHTML(
        `<div class="disaster-epicenter-popup">
        <div><b>Disaster Epicenter</b></div>
        <div>Latitude: ${DISASTER_EPICENTER_LATITUDE} </div>
        <div>Longutude: ${DISASTER_EPICENTER_LONGITUDE}</div>
        </div>`
    );
    // create a DOM element for the marker
    var el = document.createElement('div');
    el.className = 'marker';
    el.style.backgroundImage = 'url(' + dutils.urls.resolve('static_images', { file_name: 'danger_marker.png' }) + ')';
    el.style.backgroundSize = '35px 35px';
    el.style.width = '35px';
    el.style.height = '35px';

    // add marker to map
    let disasterEpicenterMarker = new maplibregl.Marker(el).setLngLat([DISASTER_EPICENTER_LONGITUDE, DISASTER_EPICENTER_LATITUDE]).setPopup(popup).addTo(map);
    }
}