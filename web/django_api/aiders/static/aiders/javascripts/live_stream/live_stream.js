function touchHandler(event) {
    var touch = event.changedTouches[0];

    var simulatedEvent = document.createEvent('MouseEvent');
    console.log('MOUSE EVENT OCCURED!');
    simulatedEvent.initMouseEvent(
        {
            touchstart: 'mousedown',
            touchmove: 'mousemove',
            touchend: 'mouseup',
        }[event.type],
        true,
        true,
        window,
        1,
        touch.screenX,
        touch.screenY,
        touch.clientX,
        touch.clientY,
        false,
        false,
        false,
        false,
        0,
        null
    );

    touch.target.dispatchEvent(simulatedEvent);
    event.preventDefault();
}

function makeItTouchDraggable(element_id) {
    document.getElementById(element_id).addEventListener('touchstart', touchHandler, true);
    document.getElementById(element_id).addEventListener('touchmove', touchHandler, true);
    document.getElementById(element_id).addEventListener('touchend', touchHandler, true);
    document.getElementById(element_id).addEventListener('touchcancel', touchHandler, true);
}

/*
 * Dynamic Creation of video elements to show the live stream for each drone
 * */
// Tracks active MediaMTX readers so they can be closed when removed
const _mtxReaders = {};

function create_video_elements(_droneName) {
    let whepUrl = `http://${window.location.hostname}:8889/live/${_droneName}/whep`;
    let directUrl = `http://${window.location.hostname}:8889/live/${_droneName}?controls=0`;

    let outer_div = document.createElement("div");
    $(outer_div).addClass("outer-img-container");
    $(outer_div).attr("id", "live-stream-wrapper-div-" + _droneName);
    // $(outer_div).attr("style", "width: 400px; height: 227px; ");
    $(outer_div).addClass("overlay-popup");
    $("#overlay").append($(outer_div).resizable({handles: "se"}));
    $(outer_div).draggable();

    let video_div = document.createElement("div");
    video_div.id = "live-video-div-" + _droneName;
    document.body.appendChild(video_div);
    $(video_div).addClass("img-container");
    outer_div.appendChild(video_div);

    let top_bar_div = document.createElement("div");
    top_bar_div.id = "live-video-top-bar-div-" + _droneName;
    top_bar_div.setAttribute("style", "background-color: rgba(0, 0, 0, 0.6); height: 23px; position:absolute; font-size: 12px; font-weight: bold; color: white; padding: 2px 5px; ");
    top_bar_div.innerHTML = '<div style="float:left;">' + _droneName + ' </div>'

    // Create video element (replaces iframe, allows passing credentials via reader.js)
    let video_elem = document.createElement("video");
    video_elem.id = "live-stream-video-" + _droneName;
    video_elem.style.width = "100%";
    video_elem.style.height = "100%";
    video_elem.style.border = "none";
    video_elem.style.objectFit = "contain";
    video_elem.style.position = "absolute";
    video_elem.setAttribute("autoplay", "");
    video_elem.setAttribute("muted", "");
    video_elem.setAttribute("playsinline", "");

    // Initialize the MediaMTX WebRTC reader with credentials
    _mtxReaders[_droneName] = new MediaMTXWebRTCReader({
        url: whepUrl,
        user: (typeof MTX_USER !== 'undefined') ? MTX_USER : "",
        pass: (typeof MTX_PASS !== 'undefined') ? MTX_PASS : "",
        onError: (err) => {
            console.error("MediaMTX stream error for " + _droneName + ":", err);
        },
        onTrack: (evt) => {
            video_elem.srcObject = evt.streams[0];
        },
    });

    let video_overlay_div = document.createElement("div");
    video_overlay_div.className = "video-overlay-div";
    video_overlay_div.id = "video-overlay-div-" + _droneName;
    video_overlay_div.style.height = "100%";
    video_overlay_div.style.width = "100%";
    video_overlay_div.style.backgroundColor = "None";
    video_overlay_div.style.position = "absolute";

    let fullscreenButton = document.createElement("button");
    fullscreenButton.id = "fullscreen-btn-" + _droneName;
    fullscreenButton.className = "stream-fullscreen-btn";
    fullscreenButton.innerHTML = '<i class="fa-solid fa-arrow-up-right-from-square"></i>';
    fullscreenButton.onclick = function () {
        let newWindow = window.open(directUrl, '_blank');
        if (newWindow) {
            newWindow.focus();
        }
    };

    // fallback
    let cover_img_elem = document.createElement("img");
    cover_img_elem.setAttribute("src", VIDEO_COVER_PHOTO_DRONE);
    cover_img_elem.id = "live-stream-img-" + _droneName;

    top_bar_div.appendChild(fullscreenButton);
    video_overlay_div.appendChild(top_bar_div);
    video_div.appendChild(video_elem);
    video_div.appendChild(video_overlay_div);
    video_div.appendChild(cover_img_elem);
    makeItTouchDraggable(video_div.id);

}

function remove_video_element(_deleteDroneName) {
    if (_mtxReaders[_deleteDroneName]) {
        _mtxReaders[_deleteDroneName].close();
        delete _mtxReaders[_deleteDroneName];
    }
    $("#live-video-div-" + _deleteDroneName).remove();
    $("#live-stream-wrapper-div-" + _deleteDroneName).remove();
}
