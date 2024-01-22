{
    let detection_timeout;
    let added_drone_api_objs = [];
    let added_drone_models = [];

    start_checking_for_detection_state();

    /*
     * Check periodically if detection is active and handle it accordingly
     * */
    function start_checking_for_detection_state() {
        setInterval(function () {
            //Get the detected objects from API. They are not related to drone id
            //Every detected object from every drone is written on the api and we visualize it.
            if(g_websocketMessage===undefined) {
                return;
            }
            g_websocketMessage['drones'].forEach(function (drone) {
                if(!drone.hasOwnProperty("detection")) {
                    return;
                }
            });

            let allDronesArr = get_all_drone_info_array();
            for (let i = 0; i < allDronesArr.length; i++) {
                g_websocketMessage['drones'].forEach(function (drone) {
                    if (drone['drone_name'] == allDronesArr[i].droneID) {
                        if(drone.hasOwnProperty("detected_objects")) {
                            objectDetected = drone['detected_objects'];
                            if (objectDetected !==null && objectDetected.length > 0){
                                visualize_detected_objects(objectDetected, drone['drone_name']);
                            }
                        }
                    }
                });
                get_drone_detection_state(allDronesArr[i].droneID).then(function (
                    detection_drone_state //Check if the detection stream is connected. We can check that from the API
                ) {
                    if (detection_drone_state === DETECTION_CONNECTED && !allDronesArr[i].detVideoStarted) {
                        let toggle = document.getElementById('detection-toggle-' + allDronesArr[i].droneID);
                        // update_drone_detection_status_locally(i, DETECTION_CONNECTED);
                        setDroneAttributeValueByIndex(i, "droneDetectionStatus", DETECTION_CONNECTED);
                        $(toggle).bootstrapToggle('on');
                        // start_detection_vid(allDronesArr[i].droneID);
                        hidePopup('#detection_loading_box');
                        hidePopup('#detection_fail_box');
                        let msg = 'Detection successfully started for drone ' + allDronesArr[i].droneID + '!';
                        showPopupForALittle('#detection_success_box', msg, 3000);
                        clear_detection_timeout();
                        allDronesArr[i].detVideoStarted = true;
                    } else if (detection_drone_state === DETECTION_DISCONNECTED && allDronesArr[i].detVideoStarted) {
                        clear_detection_timeout();
                        // update_drone_detection_status_locally(i, DETECTION_DISCONNECTED);
                        setDroneAttributeValueByIndex(i, "droneDetectionStatus", DETECTION_DISCONNECTED);
                        let toggleID = 'detection-toggle-' + allDronesArr[i].droneID;
                        let toggle = document.getElementById(toggleID);
                        $(toggle).bootstrapToggle('off');
                        hidePopup('#detection_loading_box');
                        let msg = 'Live stream for drone ' + allDronesArr[i].droneID + ' just disconnected!';
                        showPopupForALittle('#detection_fail_box', msg, 4000);
                        allDronesArr[i].detVideoStarted = false;
                    }
                });
            }
        }, 1000);
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
        var pressed = $('#' + toggleID).is(':checked');

        if (pressed) {
            if (droneIndex > -1) {
                if (allDrones[droneIndex].droneDetectionStatus === DETECTION_CONNECTED) {
                    //Already connected. Nothing to do here
                    return;
                }
                // SafeML
                let message =
                    `${DETECTION_TYPES.VEHICLE_DETECTOR.htmlDescr}` +
                    '<br><br>' +
                    `${DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.htmlDescr}` +
                    '<br><br>' +
                    `${DETECTION_TYPES.DISASTER_CLASSIFICATION.htmlDescr}` +
                    '<br><br>' +                    
                    `${DETECTION_TYPES.CROWD_LOCALIZATION.htmlDescr}` +
                    '<br><br>' +

                    `<label for="safemlCheckbox"><input type="checkbox" id="safemlCheckbox"> SafeML (<b>ONLY WORKS</b> with <b>Vehicle-Person Detector</b>)</input></label>`+
                    `<label for="deepKnowledgeCheckbox"><input type="checkbox" id="deepKnowledgeCheckbox"> DeepKowledge (<b>ONLY WORKS</b> with <b>Vehicle-Person Detector</b>)</input></label>`;//SafeML

                let title = 'Choose Detector type';
                // SafeML remove second arg and third arg
                create_detection_confirmation_dialog(DETECTION_TYPES, message, title).then(function (detectorType, safeMLActivate, deepKowledgeActivate) {
                    // SafeML
                    if (safeMLActivate == true){
                        DetectionSafeMLStartOrStop(droneID, 'Start', 'SAFEML')
                        create_or_show_safeml_video_elements(droneID, 'safeml')
                    }
                    if (deepKowledgeActivate == true){
                        DetectionSafeMLStartOrStop(droneID, 'Start', 'DEEPKNOWLEDGE')
                        create_or_show_safeml_video_elements(droneID, 'deepknowledge')
                    }
                    proceed(detectorType);
                    postElementId('Start Detection', pressed);
                });

                function proceed(detectorType) {
                    // update_drone_detection_status_locally(droneIndex, DETECTION_WANT_TO_CONNECT)
                    update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_CONNECT, detectorType);
                    let msg =
                        'Starting Detection for drone ' +
                        droneID +
                        '...' +
                        '<button id="detection_loading_box_btn" type="button" class="close" data-dismiss="alert" aria-label="Close">\n' +
                        '    <span aria-hidden="true">&times;</span>\n' +
                        '  </button>';
                    showPopup('#detection_loading_box', msg);
                    let timeoutMs = 60000;

                    detection_timeout = setTimeout(
                        function () //If, even after X seconds, the loading box is still on, remove it, because drone's stream is probably disconnected
                        {
                            let allDrones = get_all_drone_info_array();
                            get_drone_detection_state(droneID).then(function (detection_drone_state) {
                                if (detection_drone_state !== DETECTION_CONNECTED) {
                                    let toggle = document.getElementById(toggleID);
                                    $(toggle).bootstrapToggle('off');
                                    hidePopup('#detection_loading_box');
                                    let msg = 'Timeout error: Drone ' + droneID + ' failed to start detection after ' + timeoutMs / 1000 + ' seconds';
                                    showPopupForALittle('#detection_fail_box', msg, 4000);
                                    update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_DISCONNECT);
                                }
                            });
                        },
                        timeoutMs
                    );
                }
            } else {
                console.log('DRONE ID NOT FOUND');
            }
        } else {
            postElementId('Start Detection', pressed);
            if (allDrones[droneIndex].droneDetectionStatus === DETECTION_DISCONNECTED) {
                return;
            }
            console.log('ABOUT TO DEACTIVATE DETECTION...');
            update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_DISCONNECT);
            let msg =
                'Cancelling Detection for drone ' +
                droneID +
                '...' +
                '<button id="detection_loading_box_btn" type="button" class="close" data-dismiss="alert" aria-label="Close">\n' +
                '    <span aria-hidden="true">&times;</span>\n' +
                '  </button>';
            showPopup('#detection_loading_box', msg);

            // SafeML
            DetectionSafeMLStartOrStop(allDrones[droneIndex].droneID, 'Stop', 'SAFEML')
            DetectionSafeMLStartOrStop(allDrones[droneIndex].droneID, 'Stop', 'DEEPKNOWLEDGE')
            remove_det_video_elements_safeml(allDrones[droneIndex].droneID, 'safeml')
            remove_det_video_elements_safeml(allDrones[droneIndex].droneID, 'deepknowledge')
        }
    }

    /*
     * Visualizes on the map the specified detected objects found by drone.
     * */
    function visualize_detected_objects(detected_cars) {
        initialize_detected_objects_counter();

        for (let i = 0; i < detected_cars.length; i++)
        {
            let objID = detected_cars[i]["track_id"]
            let found = false
            let location = [detected_cars[i]["lat"], detected_cars[i]["lon"]]
            let objType = detected_cars[i]["label"]
            let description = ''
            let updated_by_username = ''

            if ( detected_cars[i]["description"] ) 
            {   
                //console.log( detected_cars[i]["description"] )
                description = detected_cars[i]["description"]
                updated_by_username = detected_cars[i]['updated_by_username']
            }

            increase_counter(objType);

            let found_index;
            for (let j = 0; j < added_drone_api_objs.length; j++) {
                if (objID === added_drone_api_objs[j]['track_id']) {
                    found = true;
                    found_index = j;
                    break;
                }
            }

            tooltip_html_value = detection_create_tooltip(objID,  objType, description, updated_by_username )
            
            if (found)
            {
                added_drone_models[found_index].setCoords(location)

                if( added_drone_api_objs[found_index]["description"] != description ) 
                { 
                    added_drone_models[found_index].removeTooltip()
                    added_drone_models[found_index].addTooltip(tooltip_html_value, true  , 15 );
                }
                added_drone_api_objs[found_index]["lat"] = detected_cars[i]["lat"]
                added_drone_api_objs[found_index]["lon"] = detected_cars[i]["lon"]
                added_drone_api_objs[found_index]["description"] = description
                added_drone_api_objs[found_index]["updated_by_username"] = updated_by_username

            } else
            {
                let dupModel = duplicate_model(objType);
                if (dupModel === NOT_FOUND_STRING) {
                    continue;
                }

                dupModel.visibility = get_object_visibility(objType);
                dupModel.setCoords(location);
                tb.add(dupModel)
                dupModel.addTooltip(tooltip_html_value, true, 15 );               
                added_drone_api_objs.push(detected_cars[i])

                added_drone_models.push(dupModel);
            }
        }
    }

    function detection_create_tooltip(objID,  objType, description, updated_by_username ){

        let tooltip = `
            <div>
            Object: ${objType} - ${objID}
            <button type="button" class="close" onclick='hideDetectioTooltip(${objID})'>x</button>
            <ul class="map_popup_list">
            <li>
                Object Description:
                <br>
                ${description}
            </li>
            <li>
                Updated By:
                <br>
                ${updated_by_username}
            </li>
            <li  onclick='displaySetDetectedObjectForm("${objID}","${description}")'> 
            Edit Object Description
            </li>
            </ul>
            </div> 
            `;

        return tooltip ; 
    }

    function displaySetDetectedObjectForm(track_id, description)
    {   
        //

        let titleIDEl     = document.getElementById('setDetectedObjectModalLabel');
        let descInputEl = document.getElementById('detected_object_descInput');
        let idInputEl = document.getElementById('detected_object_track_id');
        
        titleIDEl.innerHTML = 'Set Detected Object Description (Object ID: ' + track_id + ') :' ;
        descInputEl.value   = description
        idInputEl.value     = track_id ;
        
        $("#setDetectedObjectModal").show() ;
    }

    /*Returns the connection detection status for the specified drone.
     * E.g It will return CONNECTED if the current drone is currently running detection algorithms
     * */
    function get_drone_detection_state(droneID) {
        let returnObjectStatus;
        g_websocketMessage['drones'].forEach(function (drone) {
            if (drone['drone_name'] == droneID) {
                if(drone.hasOwnProperty("detection")) {
                objectDetected = drone['detection'];
                returnObjectStatus = objectDetected['detection_status'];
                }else{
                    returnObjectStatus=null;
                }
            }
        });
        return new Promise((resolve, reject) => resolve(returnObjectStatus));
        // return $.getJSON(url).then(function (detection_stats_obj)
        // {
        //     console.log(detection_stats_obj['detection_status'])
        //     return detection_stats_obj['detection_status']
        // });
    }

    function getDroneDetectionState(droneName) {
        let returnObjectStatus;
        g_websocketMessage['drones'].forEach(function (drone) {
            if (drone['drone_name'] == droneName) {
                objectDetected = drone['detection'];
                returnObjectStatus = objectDetected['detection_status'];
            }
        });
        return returnObjectStatus;
    }

    function get_object_visibility(currentDetObjType) {
        //based on toggles pressed
        // let objType = detected_api_objs[i].type
        let prototypeModels = get_prototype_models();
        let checkboxID = getCheckboxID(currentDetObjType, prototypeModels);

        let show_detections_toggle = $('#' + SHOW_DETS_TOGGLE_ID).is(':checked');
        let show_all_detections_checkbox = $('#' + SHOW_ALL_DETECTIONS_CHECKBOX_ID).is(':checked');

        let isCBchecked = $('#' + checkboxID).is(':checked');

        if (show_detections_toggle && show_all_detections_checkbox) {
            return true;
        }

        if (show_detections_toggle && isCBchecked) {
            return true;
        }
        return false;
    }

    function getCheckboxID(objType, protModels) {
        for (let i = 0; i < protModels.length; i++) {
            if (objType === protModels[i].type) {
                return protModels[i].checkboxID;
            }
        }

        return 'NO CHECKBOX ID FOUND';
    }

    function get_detected_objs() {
        return [added_drone_models, added_drone_api_objs];
    }

    function clear_detection_timeout() {
        clearTimeout(detection_timeout);
    }

    function update_drone_detection_state_on_api(_droneID, _detectionStatus, _detectionType = 'None') {
        const url = dutils.urls.resolve('detectionStartOrStop', {
            operation_name: CURRENT_OP,
            drone_name: _droneID,
        })
        const data = {
            droneName: _droneID,
            operationName: CURRENT_OP,
            detectionStatus: _detectionStatus,
            detectionType: _detectionType
          };
        const csrfToken = document.getElementById('csrf').querySelector('input').value
        fetch(url, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrfToken,
            },
            body: JSON.stringify(data)
        })
    }

    
    function create_det_video_elements(drone_ids) {
        for (let i = 0; i < drone_ids.length; i++) {
            let outer_div = document.createElement('div');
            $(outer_div).addClass('outer-img-container');
            $(outer_div).attr('id', 'detection-stream-wrapper-div-' + drone_ids[i]);
            $(outer_div).addClass('overlay-popup');
            $('#overlay').append($(outer_div).resizable());
            $(outer_div).draggable();

            let det_div = document.createElement('div');
            det_div.id = 'det-video-div-' + drone_ids[i];
            document.body.appendChild(det_div);
            $(det_div).addClass('img-container');
            outer_div.appendChild(det_div);
            let cover_img_elem = document.createElement('img');
            cover_img_elem.setAttribute('src', VIDEO_COVER_PHOTO_DRONE);
            cover_img_elem.id = 'detection-stream-img-' + drone_ids[i];
            let actionButtonID = 'initial-action-btn-' + drone_ids[i];
            let statusDivID = 'detection-stream-status-div-' + drone_ids[i];
            det_div.innerHTML =
                '<div class="top-left video-div-info">Detection: ' +
                drone_ids[i] +
                '<br/> Status: <div id="' +
                statusDivID +
                '" class="top-left-status">Disconnected</div></div>';
            det_div.prepend(cover_img_elem);
            makeItTouchDraggable(det_div.id);
        }
    }


    function remove_det_video_elements(deleted_drone_ids) {
        for (let i = 0; i < deleted_drone_ids.length; i++) {
            $('#det-video-div-' + deleted_drone_ids[i]).remove();
            $('#detection-stream-wrapper-div-' + deleted_drone_ids[i]).remove();
        }
    }
    

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



    function submitSetDetectedObjectData()
    {
        let data = formDataToJson($('#SetDetectedObjectDataForm').serializeArray())

        //console.log(data)
        
        let descriptiondata = JSON.stringify({
            "track_id": data.track_id,
            "description" : data.description
        });
        let track_id = data.track_id
        
        //console.log(descriptiondata) ; 
        
        $.ajax({

            type: "POST",
            url: dutils.urls.resolve('set_detected_object_description', { operation_name: CURRENT_OP , id: track_id }),       
            contentType: "application/json",
            data: descriptiondata,
            success: function (response)
            {

                //console.log(response) ; 

                let found_index = null
                
                for (let j = 0; j < added_drone_api_objs.length; j++)
                {   

                    if (track_id == added_drone_api_objs[j]["track_id"])
                    {   
                        found_index = j
                        break
                    }
                }
                //console.log(found_index)

                let updated_by_username = response.DetectedObjectDescription.updated_by_username
                
                //console.log('INDEX') ; 
                //console.log(found_index) ; 
                //console.log(data.description)
                if( found_index != null ) {

                    let objType = added_drone_api_objs[found_index]["label"]

                    tooltip_html_value = detection_create_tooltip( track_id,  objType, data.description, updated_by_username ) 
                  
                    added_drone_models[found_index].removeTooltip()
                    added_drone_models[found_index].addTooltip(tooltip_html_value, true,  15); 
                    
                    added_drone_api_objs[found_index]["description"] = data.description
                    added_drone_api_objs[found_index]["updated_by_username"] = updated_by_username
                }
                
                alert("Data Saved Succesfully!");

                $("#setDetectedObjectModal").hide() ;
            },
            error: function (error)
            {   
                alert("Data Not Saved! Error: ${error.responseText}" , {
                    position: 'center-center',
                    timeout: 2000,
                    showOnlyTheLastOne: true
                });
            },
            dataType: "json"
        });
    }

    function formDataToJson(serializedFormData)
    {
        let object = {};
        serializedFormData.forEach(function(el, i){
            let key = el.name;
            let val = el.value;
            object[key] = val;
        });
        return JSON.parse(JSON.stringify(object));
    }

    function hideDetectioTooltip(objID){

        let found_index;
        let found = false;
        for (let j = 0; j < added_drone_api_objs.length; j++) {
            if (objID === added_drone_api_objs[j]['track_id']) {
                found = true;
                found_index = j;
                break;
            }
        }

        if (found)
        {
            model = added_drone_models[found_index] 

            console.log(model)

            model.tooltip.visible = false
        }
    }

    function CancelSetDetectedObjectData()
    {
        $("#setDetectedObjectModal").hide() ;
    }
}
