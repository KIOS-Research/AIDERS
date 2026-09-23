{
    function update_drone_detection_state_on_api(_droneID, _detectionStatus, _detectionType = "None") {
        const url = dutils.urls.resolve("detectionStartOrStop", {
            operation_name: CURRENT_OP,
            drone_name: _droneID,
        });
        const data = {
            droneName: _droneID,
            operationName: CURRENT_OP,
            detectionStatus: _detectionStatus,
            detectionType: _detectionType,
        };
        const csrfToken = document.getElementById("csrf").querySelector("input").value;
        fetch(url, {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
                "X-CSRFToken": csrfToken,
            },
            body: JSON.stringify(data),
        })
            .then((response) => {
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                } else {
                    return response.json();
                }
            })
            .then((data) => {
                create_popup_for_a_little(SUCCESS_ALERT, "Detector Status has change to " + _detectionStatus, 2000);
            })
            .catch((error) => {
                create_popup_for_a_little(WARNING_ALERT, "There is an error on with " + _detectionType, 2000);
            });
    }

    function toggleDetectionFunctionality(toggleID, droneID) {
        let allDrones = get_all_drone_info_array();
        let droneIndex = -1;
        for (let i = 0; i < allDrones.length; i++) {
            if (droneID === allDrones[i].droneID) {
                droneIndex = i;
                break;
            }
        }
        if (droneIndex === -1) {
            return;
        }
        var pressed = $("#" + toggleID).is(":checked");

        if (pressed) {
            if (droneIndex > -1) {
                let message =
                    `${DETECTION_TYPES.WALDO_DETECTOR.htmlDescr}` +
                    "<br><br>" +
                    // `${DETECTION_TYPES.VEHICLE_DETECTOR.htmlDescr}` +
                    // '<br><br>' +
                    // `${DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.htmlDescr}` +
                    // '<br><br>' +
                    `${DETECTION_TYPES.DISASTER_CLASSIFICATION.htmlDescr}` +
                    "<br><br>" +
                    `${DETECTION_TYPES.CROWD_LOCALIZATION.htmlDescr}`;

                let title = "Choose Detector type";
                create_detection_confirmation_dialog(DETECTION_TYPES, message, title).then(function (detectorType) {
                    proceed(detectorType);
                    postElementId("Start Detection", pressed);
                });

                function proceed(detectorType) {
                    // update_drone_detection_status_locally(droneIndex, DETECTION_WANT_TO_CONNECT)
                    update_drone_detection_state_on_api(
                        allDrones[droneIndex].droneID,
                        DETECTION_WANT_TO_CONNECT,
                        detectorType,
                    );
                }
            } else {
                console.log("DRONE ID NOT FOUND");
            }
        } else {
            console.log("ABOUT TO DEACTIVATE DETECTION...");
            update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_DISCONNECT);
        }
    }

    function getDroneDetectionState(droneName) {
        let returnObjectStatus;
        g_websocketMessage["drones"].forEach(function (drone) {
            if (drone["drone_name"] == droneName) {
                objectDetected = drone["detection"];
                returnObjectStatus = objectDetected["detection_status"];
            }
        });
        return returnObjectStatus;
    }

    function create_det_video_elements(_droneName) {
        let outer_div = document.createElement("div");
        $(outer_div).addClass("outer-img-container");
        $(outer_div).attr("id", "detection-stream-wrapper-div-" + _droneName);
        $(outer_div).addClass("overlay-popup");
        $("#overlay").append($(outer_div).resizable({ handles: "se" }));
        $(outer_div).draggable();

        let det_div = document.createElement("div");
        det_div.id = "det-video-div-" + _droneName;
        document.body.appendChild(det_div);
        $(det_div).addClass("img-container");
        outer_div.appendChild(det_div);
        let cover_img_elem = document.createElement("img");
        cover_img_elem.setAttribute("src", VIDEO_COVER_PHOTO_DRONE);
        cover_img_elem.id = "detection-stream-img-" + _droneName;
        let actionButtonID = "initial-action-btn-" + _droneName;
        let statusDivID = "detection-stream-status-div-" + _droneName;
        det_div.innerHTML =
            '<div class="top-left video-div-info">Detection: ' + _droneName + '</div>';
        det_div.prepend(cover_img_elem);
        makeItTouchDraggable(det_div.id);
    }

    function remove_det_video_elements(_deleteDroneName) {
        $('#det-video-div-' + _deleteDroneName).remove();
        $('#detection-stream-wrapper-div-' + _deleteDroneName).remove();
    }
}
