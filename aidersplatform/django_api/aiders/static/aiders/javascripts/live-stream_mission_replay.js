/*
 * Dynamic Creation of video elements to show the live stream for each drone
 * */
function create_video_elements(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let outer_div = document.createElement('div');
        $(outer_div).addClass('outer-img-container');
        $(outer_div).attr('id', 'outer-live-stream-img-div-' + drone_ids[i]);

        $('body').append($(outer_div).resizable());
        $(outer_div).draggable();

        let det_div = document.createElement('div');
        det_div.id = 'live-video-div-' + drone_ids[i];
        document.body.appendChild(det_div);
        $(det_div).addClass('img-container');
        outer_div.appendChild(det_div);
        let cover_img_elem = document.createElement('img');
        cover_img_elem.setAttribute('src', VIDEO_COVER_PHOTO_DRONE);
        let actionButtonID = 'initial-action-btn-' + drone_ids[i];
        let statusDivID = 'status-div-' + drone_ids[i];
        det_div.innerHTML =
            '<br/><button class="btn"  id="' +
            actionButtonID +
            '" > &#9658 </button><div class="top-left">Drone: ' +
            drone_ids[i] +
            '<br/> Status: <div id="' +
            statusDivID +
            '" class="top-left-status">Disconnected</div></div>';
        det_div.prepend(cover_img_elem);
        makeItTouchDraggable(det_div.id);
    }
}

function remove_video_element(deleted_drone_ids) {
    for (let i = 0; i < deleted_drone_ids.length; i++) {
        $('#live-video-div-' + deleted_drone_ids[i]).remove();
        $('#outer-live-stream-img-div-' + deleted_drone_ids[i]).remove();
    }
}

function start_live_stream(droneID) {
    let live_vid_element = document.getElementById('live-video-div-' + droneID);
    var playing = true;
    var timer;
    let actionButtonID = 'live-feed-action-btn-' + droneID;
    let statusDivID = 'live-feed-status-div-' + droneID;
    launch();

    console.log('STARTING LIVE VID. ... ');

    function launch() {
        let latest_frame_path;
        if (frame_obj['latest_frame_url'] === NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE) {
            latest_frame_path = VIDEO_LOADING_PHOTO;
        } else {
            latest_frame_path = frame_obj['latest_frame_url'];
        }
        let imgID = 'det-img-' + droneID;
        live_vid_element.innerHTML =
            '<img id="' +
            imgID +
            '" src=' +
            latest_frame_path +
            '/><br/><button class="btn"  id="' +
            actionButtonID +
            '" > &#9658 </button><div class="top-left">Drone: ' +
            droneID +
            '<br/> Status: <div id="' +
            statusDivID +
            '" class="top-left-status"></div></div>';
        document.getElementById(actionButtonID).addEventListener('click', play);
        play();
    }

    function change() {
        if (webSocketMessage['drones'][droneID] !== undefined) {
            latest_frame_path = webSocketMessage['drones'][droneID]['video_frame']['frame'];
        } else {
            latest_frame_path = NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE;
        }
        let currentDrone = get_drone_object(droneID);
        let status_div_id = document.getElementById('live-feed-status-div-' + droneID);
        if (latest_frame_path === NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE) {
            status_div_id.style.color = 'red';
            status_div_id.innerHTML = 'Disconnected';
        } else {
            status_div_id.style.color = 'lawngreen';
            status_div_id.innerHTML = 'Connected';
        }

        if (latest_frame_path === NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE) {
            document.getElementById('det-img-' + droneID).src = VIDEO_COVER_PHOTO_DRONE;
        } else {
            document.getElementById('det-img-' + droneID).src = dutils.urls.resolve('live_frame_path', {
                frame_path: latest_frame_path,
            });
        }
        if (playing) {
            timer = setTimeout(change, 50);
        }
    }

    function play() {
        let actionBtn = document.getElementById(actionButtonID);
        actionBtn.removeEventListener('click', play);
        actionBtn.addEventListener('click', pause);
        actionBtn.innerHTML = '&#9612;' + '&#9612'; //SHOW THE PAUSE BUTTON
        actionBtn.style.top = '93%';
        actionBtn.style.left = '6.5%';
        clearInterval(timer);
        playing = true;
        change();
    }

    function pause() {
        console.log('WILL NOW PAUSE');
        let actionBtn = document.getElementById(actionButtonID);
        actionBtn.removeEventListener('click', pause);
        actionBtn.addEventListener('click', play);
        actionBtn.innerHTML = '&#9658'; //PLAY
        actionBtn.style.top = '93%';
        actionBtn.style.left = '5%';
        playing = false;
    }
}
