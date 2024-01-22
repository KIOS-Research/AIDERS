// SafeML
function DetectionSafeMLStartOrStop(_droneName, _detectionStatus, _detectionType){
    let baseUrl = window.location.protocol + '//' + window.location.host;
    let startDetectionSafeMLUrl = baseUrl + '/api/operations/' + CURRENT_OP + '/drones/' + _droneName + '/detectionSafeMLStartOrStop'
    const data = {
        droneName: _droneName,
        operationName: CURRENT_OP,
        detectionStatus: _detectionStatus,
        detectionType: _detectionType
    };
    const csrfToken = document.getElementById('csrf').querySelector('input').value
    fetch(startDetectionSafeMLUrl, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': csrfToken,
        },
        body: JSON.stringify(data)
    })
}
function create_or_show_safeml_video_elements(droneId, type) {
    if (document.getElementById("yourElementId")== null){
        let outer_div = document.createElement('div');
        $(outer_div).addClass('outer-img-container');
        $(outer_div).attr('id', 'detection-stream-'+type+'-wrapper-div-' + droneId);
        $(outer_div).addClass('overlay-popup');
        $('#overlay').append($(outer_div).resizable());
        $(outer_div).draggable();
    
        let det_div = document.createElement('div');
        det_div.id = 'det-video-'+type+'-div-' + droneId
        document.body.appendChild(det_div);
        $(det_div).addClass('img-container');
        outer_div.appendChild(det_div);
        let cover_img_elem = document.createElement('img');
        cover_img_elem.setAttribute('src', VIDEO_COVER_PHOTO_DRONE);
        cover_img_elem.id = 'detection-stream-'+type+'-img-' + droneId;
        let actionButtonID = 'initial-action-btn-' + droneId;
        let statusDivID = 'detection-stream-'+type+'-status-div-' + droneId;
        det_div.innerHTML =
            '<div class="top-left video-div-info">Detection: ' +
            droneId +
            '<br/> Status: <div id="' +
            statusDivID +
            '" class="top-left-status">Disconnected</div></div>';
        det_div.prepend(cover_img_elem);
        makeItTouchDraggable_safeml(det_div.id);
    }
    document.getElementById('detection-stream-'+type+'-wrapper-div-' + droneId).style.display = 'block';
}


function remove_det_video_elements_safeml(deleted_drone_id, type) {
        $('#det-video-'+type+'-div-' + deleted_drone_id).remove();
        $('#detection-stream-'+type+'-wrapper-div-' + deleted_drone_id).remove();
}


function touchHandler_safeml(event) {
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

function makeItTouchDraggable_safeml(element_id) {
    document.getElementById(element_id).addEventListener('touchstart', touchHandler_safeml, true);
    document.getElementById(element_id).addEventListener('touchmove', touchHandler_safeml, true);
    document.getElementById(element_id).addEventListener('touchend', touchHandler_safeml, true);
    document.getElementById(element_id).addEventListener('touchcancel', touchHandler_safeml, true);
}


function formDataToJson(serializedFormData){
    let object = {};
    serializedFormData.forEach(function(el, i){
        let key = el.name;
        let val = el.value;
        object[key] = val;
    });
    return JSON.parse(JSON.stringify(object));
}


function CancelSetDetectedObjectData()
{
    $("#setDetectedObjectModal").hide() ;
}