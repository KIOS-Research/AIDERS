
let fallbackLiveStream = LEGACY_LSC; // legacyLSC - if false, the live stream will be displayed as webRTC, otherwise as jpg images

const framesWsInterval = 200; // in milliseconds

const framesWsAddress = 'ws://' + window.location.hostname + ':' + NGINX_PORT + "/ws/getFrames"; // go ws

let framesWebsocketMessage;
let framesWebsocket = initFramesWebsocket(framesWsAddress);

let video_and_cv_url;
if (VIDEO_AND_CV_REMOTE == true) {
    video_and_cv_url = 'http://' + window.location.hostname + ':' + NGINX_PORT + "/remote";
} else {
    video_and_cv_url = 'http://' + window.location.hostname + ':' + NGINX_PORT;
}


map.on('load', function () {
    setInterval(sendFramesWebsocketMessage, framesWsInterval); // init websocket data timer
});


// establish websocket connection and set up event listeners
function initFramesWebsocket(_address) {
    console.log(_address);
    let ws = new WebSocket(_address + '?token=' + encodeURIComponent(TOKEN));
    ws.addEventListener('open', function (event) {
        console.log('Frames WebSocket connection established.');
    });
    ws.addEventListener('close', function (event) {
        console.log('Frames Socket is closed. Reconnect will be attempted in 1 second.');
        setTimeout(function () {
            framesWebsocket = initFramesWebsocket(framesWsAddress);
        }, 1000);
    });
    ws.addEventListener('error', function (event) {
        console.error('Frames Socket encountered error: Closing connection');
    });
    ws.addEventListener('message', function (e) {
        framesWebsocketMessage = JSON.parse(e.data);       
        handleIncomingFramesWebsocketMessage(framesWebsocketMessage);
    });
    return ws;
}


// send a message to the websocket server
function sendFramesWebsocketMessage() {
    if (framesWebsocket.readyState == 1) {
        const jsonData = {
            operation_id: parseInt(OPERATION_ID),
        };
        framesWebsocket.send(JSON.stringify(jsonData));
    }
}



// triggered when a websocket message is received
function handleIncomingFramesWebsocketMessage(_wsMessage) {
    updateStreamFrames(_wsMessage['drones']);   // update the stream frames
}


// change the src attribute of the stream image if the stream is visible
function updateStreamFrames(drones) {
    drones.forEach(function (drone) {
        let droneName = drone['drone_name'];
        // console.log("LIVE:",drone['video_frame_url']);
        // console.log("DET:",drone['detected_frame_url']);

        if(fallbackLiveStream == true) {
            try {
                let webRTCiframe = document.getElementById('live-stream-iframe-' + droneName);
                webRTCiframe.style.display = "none"; // hide the iframe if it exists
            } catch (error) {
                console.log("WebRTC iframe for " + droneName + " not yet initialized");
            }
            changeStreamImageAndStatus(droneName, drone['video_frame_url'], "live-stream");
        }
        changeStreamImageAndStatus(droneName, drone['detected_frame_url'], "detection-stream");
    });
}




function changeStreamImageAndStatus(droneName, framePath, elementIdPrefix) {
    try {
        let wrapper_div = document.getElementById(elementIdPrefix + '-wrapper-div-' + droneName);
        if ($(wrapper_div).is(":visible")) {

            let status_div = document.getElementById(elementIdPrefix + '-status-div-' + droneName);
            let img_element = document.getElementById(elementIdPrefix + '-img-' + droneName);

            // Attach the load-failure handler once per element. If a request
            // for the current URL fails (e.g. a transient 404 while a
            // start/stop was in flight), clear the "last applied src" marker
            // so the next poll retries the same URL instead of being skipped
            // forever by the de-dup check below.
            if (img_element.dataset.errorHandlerAttached !== "1") {
                img_element.addEventListener("error", function () {
                    img_element.dataset.lastAppliedSrc = "";
                });
                img_element.dataset.errorHandlerAttached = "1";
            }

            let new_src;
            if (framePath.startsWith("/")) {
                new_src = video_and_cv_url + framePath;
                // status_div.style.color = 'lawngreen';
                // status_div.innerHTML = 'Connected';
            }
            else {
                // status_div.style.color = 'red';
                // status_div.innerHTML = 'Disconnected';
                new_src = framePath;
                // status_div.style.color = 'lawngreen';
                // status_div.innerHTML = "External";
            }

            // Avoid resetting the src to the same URL: for an MJPEG endpoint
            // this would tear down and reopen the multipart connection. This
            // is tracked separately from img_element.src (rather than reading
            // it back) so that a failed load can be retried on the next poll
            // even though the requested URL itself hasn't changed.
            if (img_element.dataset.lastAppliedSrc !== new_src) {
                img_element.src = new_src;
                img_element.dataset.lastAppliedSrc = new_src;
            }
        }
    }
    catch (error) {
        console.log("Video elements for " + droneName + " not yet initialized");
        console.log(error);

    }
}
