{
    let detection_timeout
    let added_drone_api_objs = []
    let added_drone_models = []


    start_checking_for_detection_state()

    /*
    * Check periodically if detection is active and handle it accordingly
    * */
    function start_checking_for_detection_state()
    {

        setInterval(function ()
        {
            // //Get the detected objects from API. They are not related to drone id
            // //Every detected object from every drone is written on the api and we visualize it.


            let allDronesArr = get_all_drone_info_array()
            for (let i = 0; i < allDronesArr.length; i++)
            {
                webSocketMessage['drones'].forEach(function(drone) {
                    if(drone['drone_name']==allDronesArr[i].droneID){
                        objectDetected=drone['detected_frame']
                        if (objectDetected.length > 0){
                            visualize_detected_objects(objectDetected)
                        }
                    }
                });
                get_drone_detection_state(allDronesArr[i].droneID).then(function (detection_drone_state) //Check if the detection stream is connected. We can check that from the API
                {
                    if (detection_drone_state === DETECTION_CONNECTED && !allDronesArr[i].detVideoStarted )
                    {
                        let toggle = document.getElementById("detection-toggle-" + allDronesArr[i].droneID)
                        update_drone_detection_status_locally(i, DETECTION_CONNECTED)
                        $(toggle).bootstrapToggle('on');
                        start_detection_vid(allDronesArr[i].droneID)
                        hidePopup('#detection_loading_box')
                        hidePopup('#detection_fail_box')
                        let msg = "Detection successfully started for drone " + allDronesArr[i].droneID + "!"
                        showPopupForALittle('#detection_success_box', msg, 3000)
                        clear_detection_timeout()
                        allDronesArr[i].detVideoStarted = true
                    }
                    else if (detection_drone_state === DETECTION_DISCONNECTED && allDronesArr[i].detVideoStarted )
                    {
                        clear_detection_timeout()
                        update_drone_detection_status_locally(i, DETECTION_DISCONNECTED)
                        let toggleID = "detection-toggle-" + allDronesArr[i].droneID
                        let toggle = document.getElementById(toggleID)
                        $(toggle).bootstrapToggle('off');
                        hidePopup('#detection_loading_box')
                        let msg = "Live stream for drone " + allDronesArr[i].droneID + " just disconnected!"
                        showPopupForALittle('#detection_fail_box', msg, 4000)
                        allDronesArr[i].detVideoStarted = false
                    }
                })
            }
        }, 1000)
    }



    function toggleDetectionFunctionality(toggleID, droneID)
    {
        let allDrones = get_all_drone_info_array()
        let droneIndex = -1
        for (let i = 0; i < allDrones.length; i++)
        {
            if (droneID === allDrones[i].droneID)
            {
                droneIndex = i
                break
            }
        }
        if (droneIndex === -1)
        {
            return
        }
        var pressed = $("#" + toggleID).is(':checked');

        if (pressed)
        {
            if (droneIndex > -1)
            {
                if (allDrones[droneIndex].droneDetectionStatus === DETECTION_CONNECTED) //Already connected. Nothing to do here
                {
                    return
                }
                let message = `${DETECTION_TYPES.VEHICLE_DETECTOR.htmlDescr}` + "<br><br>" +
                                `${DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.htmlDescr}` + "<br><br>"  
                let title = "Choose Detector type"
                create_detection_confirmation_dialog(DETECTION_TYPES,message,title,).then(function (detectorType)
                {
                    proceed(detectorType)
                    postElementId(toggleID, pressed)
                })

                function proceed(detectorType)
                {
                    // update_drone_detection_status_locally(droneIndex, DETECTION_WANT_TO_CONNECT)
                    update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_CONNECT, detectorType)
                    let msg = "Starting Detection for drone " + droneID + "..." + '<button id="detection_loading_box_btn" type="button" class="close" data-dismiss="alert" aria-label="Close">\n' + '    <span aria-hidden="true">&times;</span>\n' + '  </button>'
                    showPopup('#detection_loading_box', msg)
                    let timeoutMs = 60000

                    detection_timeout = setTimeout(function () //If, even after X seconds, the loading box is still on, remove it, because drone's stream is probably disconnected
                    {
                        let allDrones = get_all_drone_info_array();
                        get_drone_detection_state(droneID).then(function (detection_drone_state)
                        {
                            if (detection_drone_state !== DETECTION_CONNECTED)
                            {
                                let toggle = document.getElementById(toggleID)
                                $(toggle).bootstrapToggle('off');
                                hidePopup('#detection_loading_box')
                                let msg = "Timeout error: Drone " + droneID + " failed to start detection after " + (timeoutMs / 1000) + " seconds"
                                showPopupForALittle('#detection_fail_box', msg, 4000)
                                update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_DISCONNECT)
                            }
                        });


                    }, timeoutMs);
                }

            } else
            {
                console.log("DRONE ID NOT FOUND")
            }


        }
        else
        {
            postElementId(toggleID, pressed)
            if (allDrones[droneIndex].droneDetectionStatus === DETECTION_DISCONNECTED)
            {
                return
            }
            console.log("ABOUT OT DEACTIVATE DETECTION...")
            update_drone_detection_state_on_api(allDrones[droneIndex].droneID, DETECTION_WANT_TO_DISCONNECT);
            let msg = "Cancelling Detection for drone " + droneID + "..." + '<button id="detection_loading_box_btn" type="button" class="close" data-dismiss="alert" aria-label="Close">\n' + '    <span aria-hidden="true">&times;</span>\n' + '  </button>'
            showPopup('#detection_loading_box', msg)

        }
    }


    /*
    * Visualizes on the map the specified detected objects found by drone.
    * */
    function visualize_detected_objects(detected_cars)
    {
        initialize_detected_objects_counter();

        for (let i = 0; i < detected_cars.length; i++)
        {
            let objID = detected_cars[i]["track_id"]
            let found = false
            let location = [detected_cars[i]["lat"], detected_cars[i]["lon"]]
            let objType = detected_cars[i]["label"]

            increase_counter(objType)


            let found_index
            for (let j = 0; j < added_drone_api_objs.length; j++)
            {
                if (objID === added_drone_api_objs[j]["track_id"])
                {
                    found = true
                    found_index = j
                    break
                }
            }

            if (found)
            {
                added_drone_models[found_index].setCoords(location)
            } else
            {
                let dupModel = duplicate_model(objType);
                if (dupModel === NOT_FOUND_STRING)
                {
                    continue
                }

                dupModel.visibility = get_object_visibility(objType)
                dupModel.setCoords(location);
                tb.add(dupModel)
                dupModel.addTooltip(objType + "_" + objID, true);
                added_drone_api_objs.push(detected_cars[i])

                added_drone_models.push(dupModel)
            }
        }
    }


    /*Returns the connection detection status for the specified drone.
    * E.g It will return CONNECTED if the current drone is currently running detection algorithms
    * */
    function get_drone_detection_state(droneID)
    {
        // console.log(webSocketMessage)
        let returnObjectStatus
        webSocketMessage['drones'].forEach(function(drone) {
            if(drone['drone_name']==droneID){
                objectDetected=drone['detection']
                returnObjectStatus = objectDetected['detection_status']
            }
        });
        return new Promise((resolve, reject) => resolve(returnObjectStatus));
        // return $.getJSON(url).then(function (detection_stats_obj)
        // {
        //     console.log(detection_stats_obj['detection_status'])
        //     return detection_stats_obj['detection_status']
        // });
    }


    function get_object_visibility(currentDetObjType) //based on toggles pressed
    {
        // let objType = detected_api_objs[i].type
        let prototypeModels = get_prototype_models()
        let checkboxID = getCheckboxID(currentDetObjType, prototypeModels)

        let show_detections_toggle = $("#" + SHOW_DETS_TOGGLE_ID).is(':checked');
        let show_all_detections_checkbox = $("#" + SHOW_ALL_DETECTIONS_CHECKBOX_ID).is(':checked');

        let isCBchecked = $("#" + checkboxID).is(':checked');

        if (show_detections_toggle && show_all_detections_checkbox)
        {
            return true
        }

        if (show_detections_toggle && isCBchecked)
        {
            return true
        }
        return false


    }

    function getCheckboxID(objType, protModels)
    {
        for (let i = 0; i < protModels.length; i++)
        {
            if (objType === protModels[i].type)
            {
                return protModels[i].checkboxID
            }
        }

        return "NO CHECKBOX ID FOUND"
    }

    function get_detected_objs()
    {
        return [added_drone_models, added_drone_api_objs]
    }



    function clear_detection_timeout()
    {
        clearTimeout(detection_timeout)
    }

    function update_drone_detection_state_on_api(droneID, detection_status, detectorType="None")
    {
        let type
        let data
        if (detectorType === "None")
        {
            data = {detection:{drone_name:droneID, operation_name:CURRENT_OP, detection_status: detection_status}}
        }
        else
        {
            data = {detection:{drone_name:droneID, operation_name:CURRENT_OP, detection_status: detection_status, detection_type:detectorType}}
        }
        socket.send(JSON.stringify(data))
        console.log("Updated Detection Status on API!")
    }

    function create_det_video_elements(drone_ids)
    {
        for (let i = 0; i < drone_ids.length; i++)
        {
            let outer_div = document.createElement('div');
            $(outer_div).addClass('outer-img-container')
            $(outer_div).attr('id', 'outer-det-img-div-' + drone_ids[i]);

            $("body").append(($(outer_div).resizable()));
            $(outer_div).draggable()

            let det_div = document.createElement('div');
            det_div.id = 'det-video-div-' + drone_ids[i]
            document.body.appendChild(det_div)
            $(det_div).addClass('img-container')
            outer_div.appendChild(det_div)
            let cover_img_elem = document.createElement("img");
            cover_img_elem.setAttribute("src", VIDEO_COVER_PHOTO_DRONE);
            let actionButtonID = 'initial-action-btn-' + drone_ids[i]
            let statusDivID = 'status-div-' + drone_ids[i]
            det_div.innerHTML = '<br/><button class="btn"  id="' + actionButtonID + '" > &#9658 </button><div class="top-left">Drone: ' + drone_ids[i] + '<br/> Status: <div id="' + statusDivID + '" class="top-left-status">Disconnected</div></div>'
            det_div.prepend(cover_img_elem)
            makeItTouchDraggable(det_div.id)
        }
    }


    function start_detection_vid(droneID)
    {
        let det_vid_element = document.getElementById("det-video-div-" + droneID)
        let actionButtonID = "actionbutton-" + droneID
        let statusDivID = 'status-div-' + droneID
        launch()
        var playing = true;
        var timer;
        

        console.log("STARTING DETECTION VID. ... ")
        function launch()
        {
            webSocketMessage['drones'].forEach(function(drone) {
                if(drone['drone_name']==droneID){
                    let latest_frame_path;

                    if (drone['detected_frame_url'] === NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE)
                    {
                        latest_frame_path = VIDEO_LOADING_PHOTO
                    } else
                    {
                        latest_frame_path = drone['detected_frame_url']
                    }
                    let imgID = 'det-img-' + droneID
                    det_vid_element.innerHTML = '<img id="' + imgID + '" src=' + latest_frame_path + '/><br/><button class="btn"  id="' + actionButtonID + '" > &#9658 </button><div class="top-left">Drone: ' + droneID + '<br/> Status: <div id="' + statusDivID + '" class="top-left-status"></div></div>'
                    document.getElementById(actionButtonID).addEventListener('click', play);
                    play();
                }
            });
        }

        function change()
        {
            webSocketMessage['drones'].forEach(function(drone) {
                if(drone['drone_name']==droneID){
                    let latest_frame_path = drone['detected_frame_url']

                    let currentDrone = get_drone_object(droneID);
                    let status_div_id = document.getElementById('status-div-' + droneID)

                    if (drone['detected_frame_url'] === NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE)
                    {
                        // status_div_id.style.color = 'orange'
                        // status_div_id.innerHTML = "Loading..."

                        status_div_id.style.color = 'red'
                        status_div_id.innerHTML = "Disconnected"
                    } else
                    {
                        if (currentDrone.droneDetectionStatus === DETECTION_CONNECTED)
                        {
                            status_div_id.style.color = 'lawngreen'
                            status_div_id.innerHTML = "Connected"
                        } else
                        {
                            status_div_id.style.color = 'red'
                            status_div_id.innerHTML = "Disconnected"
                        }
                    }

                    if (drone['detected_frame_url'] === NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE)
                    {
                        document.getElementById('det-img-' + droneID).src = VIDEO_COVER_PHOTO_DRONE;
                    } else
                    {
                        document.getElementById('det-img-' + droneID).src = latest_frame_path;
                    }
                    if (playing)
                    {
                        timer = setTimeout(change, 50);
                    }
                }
            })
        }

        function play()
        {
            let actionBtn = document.getElementById(actionButtonID)
            actionBtn.removeEventListener('click', play);
            actionBtn.addEventListener('click', pause);
            actionBtn.innerHTML = '&#9612;' + '&#9612'; //SHOW THE PAUSE BUTTON
            actionBtn.style.top = "93%"
            actionBtn.style.left = "6.5%"
            clearInterval(timer);
            playing = true;
            change();

        }

        function pause()
        {
            let actionBtn = document.getElementById(actionButtonID)
            actionBtn.removeEventListener('click', pause);
            actionBtn.addEventListener('click', play);
            actionBtn.innerHTML = '&#9658'; //PLAY
            actionBtn.style.top = "93%"
            actionBtn.style.left = "5%"
            playing = false;
        }
    }

    function touchHandler(event)
    {
        var touch = event.changedTouches[0];

        var simulatedEvent = document.createEvent("MouseEvent");
        console.log("MOUSE EVENT OCCURED!")
        simulatedEvent.initMouseEvent({
                touchstart: "mousedown",
                touchmove: "mousemove",
                touchend: "mouseup"
            }[event.type], true, true, window, 1,
            touch.screenX, touch.screenY,
            touch.clientX, touch.clientY, false,
            false, false, false, 0, null);

        touch.target.dispatchEvent(simulatedEvent);
        event.preventDefault();
    }

    function makeItTouchDraggable(element_id)
    {
        document.getElementById(element_id).addEventListener("touchstart", touchHandler, true);
        document.getElementById(element_id).addEventListener("touchmove", touchHandler, true);
        document.getElementById(element_id).addEventListener("touchend", touchHandler, true);
        document.getElementById(element_id).addEventListener("touchcancel", touchHandler, true);
    }


    function remove_det_video_elements(deleted_drone_ids)
    {
        for (let i = 0; i < deleted_drone_ids.length; i++)
        {
            $("#det-video-div-" + deleted_drone_ids[i]).remove();
            $("#outer-det-img-div-" + deleted_drone_ids[i]).remove();

        }
    }
}
