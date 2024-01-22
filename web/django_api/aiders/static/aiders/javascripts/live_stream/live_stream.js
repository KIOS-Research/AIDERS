/*
 * Dynamic Creation of video elements to show the live stream for each drone
 * */
function create_video_elements(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let outer_div = document.createElement('div');
        $(outer_div).addClass('outer-img-container');
        $(outer_div).attr('id', 'live-stream-wrapper-div-' + drone_ids[i]);
        $(outer_div).addClass('overlay-popup');
        $('#overlay').append($(outer_div).resizable());
        $(outer_div).draggable();

        let det_div = document.createElement('div');
        det_div.id = 'live-video-div-' + drone_ids[i];
        document.body.appendChild(det_div);
        $(det_div).addClass('img-container');
        outer_div.appendChild(det_div);
        let cover_img_elem = document.createElement('img');
        cover_img_elem.setAttribute('src', VIDEO_COVER_PHOTO_DRONE);
        cover_img_elem.id = 'live-stream-img-' + drone_ids[i];
        let actionButtonID = 'initial-action-btn-' + drone_ids[i];
        let statusDivID = 'live-stream-status-div-' + drone_ids[i];
        det_div.innerHTML =
            '<div class="top-left video-div-info">Live Stream: ' +
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
        $('#live-stream-wrapper-div-' + deleted_drone_ids[i]).remove();
    }
}


