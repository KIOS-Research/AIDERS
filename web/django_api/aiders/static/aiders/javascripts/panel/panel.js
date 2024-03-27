
/**********************/
/******* DRONES *******/
/**********************/


// adds drones to the side-panel
function addNewDronesToSidePanel(_droneNames) {
    for (let i = 0; i < _droneNames.length; i++) {
        let droneName = _droneNames[i];
        console.log("Drone: " + droneName + " connected.");
        let ul = document.getElementById('drone-selection-list');

        let li = document.createElement('li');
        li.id = 'sidepanel-drone-' + droneName;
        ul.append(li);
        
        // create the row element for this drone
        let droneWrapperDiv = document.createElement('div');
        // $(droneWrapperDiv).addClass('row');
        droneWrapperDiv.style.fontSize = '13px';
        droneWrapperDiv.style.marginBottom = '15px';
        droneWrapperDiv.style.color = '#ffffff';
        droneWrapperDiv.style.lineHeight = '2';
        droneWrapperDiv.style.background = 'inherit';
        li.append(droneWrapperDiv);

        // create and add the drone's name
        let droneHeader = document.createElement('div');
        $(droneHeader).addClass('row client-header');
        // $(droneHeader).addClass('');

        // let droneNameHtml = document.createTextNode(droneName);
        let droneNameHeader = document.createElement('div');
        droneNameHeader.id = 'sidepanel-drone-name-' + droneName;
        $(droneNameHeader).addClass('col-md-6');

        let brandLogo = "";
        if (getDroneAttributeValue(droneName, "droneType") == "MAVLINK") {
            brandLogo = "<img src='/static/aiders/imgs/mavlink-logo.png' width='22' style='margin-right:6px;' title='MAVLink' />";
        }
        else {
            brandLogo = "<img src='/static/aiders/imgs/dji-logo.png' width='22' style='margin-right:6px;' title='DJI' />";
        }

        droneNameHeader.innerHTML = "<a href='#' style='padding: 0px; opacity: 1;' onclick='zoomToClient(\"drone\", \""+droneName+"\")'>" + brandLogo + droneName + "</a>";

        let droneAltitudeHeader = document.createElement('div');
        droneAltitudeHeader.id = 'sidepanel-drone-altitude-' + droneName;
        $(droneAltitudeHeader).addClass('col-md-2 client-header-data');
        droneAltitudeHeader.innerHTML = "...m";
        $(droneAltitudeHeader).attr("data-toggle", "tooltip");
        $(droneAltitudeHeader).attr("title", "Altitude");

        let droneBatteryHeader = document.createElement('div');
        droneBatteryHeader.id = 'sidepanel-drone-battery-' + droneName;
        $(droneBatteryHeader).addClass('col-md-2 client-header-data');
        droneBatteryHeader.innerHTML = "...%";
        $(droneBatteryHeader).attr("data-toggle", "tooltip");
        $(droneBatteryHeader).attr("title", "Battery level");        

        let droneStatusHeader = document.createElement('div');
        droneStatusHeader.id = 'sidepanel-drone-status-' + droneName;
        $(droneStatusHeader).addClass('col-md-2 client-header-data');
        droneStatusHeader.innerHTML = "";
        $(droneStatusHeader).attr("data-toggle", "tooltip");
        $(droneStatusHeader).attr("title", "Status");

        // icon = "bullseye";
        // droneHeader.innerHTML = "<i class='fa fa-" + icon + "'></i> ";
        droneHeader.appendChild(droneNameHeader);
        droneHeader.appendChild(droneAltitudeHeader);
        droneHeader.appendChild(droneBatteryHeader);
        droneHeader.appendChild(droneStatusHeader);
        
        droneWrapperDiv.appendChild(droneHeader);
        
        // create the divs that will contain the toggle buttons
        let togglesRow1 = document.createElement('div');
        togglesRow1.style.marginBottom = '4px';
        let togglesRow2 = document.createElement('div');
        togglesRow2.style.marginBottom = '4px';
        let togglesRow3 = document.createElement('div');
        // $(togglesRow1).addClass('col-md-12');

        // create and append the checkboxes
        let selectCheckbox = createCheckbox('drone-select-toggle-' + droneName);
        let gpsCheckbox = createCheckbox('drone-gps-toggle-' + droneName);
        let routeCheckbox = createCheckbox('drone-route-toggle-' + droneName);
        let videoCheckbox = createCheckbox('drone-video-toggle-' + droneName);
        let cvCheckbox = createCheckbox('drone-cv-toggle-' + droneName);
        let cvVideoCheckbox = createCheckbox('drone-cv-video-toggle-' + droneName);
        // let weatherCheckbox = createCheckbox('drone-weather-toggle-' + droneName);

        togglesRow1.appendChild(selectCheckbox);
        togglesRow1.appendChild(gpsCheckbox);
        togglesRow1.appendChild(routeCheckbox);
        
        togglesRow2.appendChild(videoCheckbox);
        togglesRow2.appendChild(cvCheckbox);
        togglesRow2.appendChild(cvVideoCheckbox);

        // togglesRow2.appendChild(weatherCheckbox);

        // append the div holding the checkboxes to the wrapper div
        // this must be done before converting checkboxes to toggles
        droneWrapperDiv.appendChild(togglesRow1);
        droneWrapperDiv.appendChild(togglesRow2);
        droneWrapperDiv.appendChild(togglesRow3);

        convertCheckboxToToggleButton(selectCheckbox, droneName, "check", "SELECT", toggleSelectDrone, "off");
        convertCheckboxToToggleButton(gpsCheckbox, droneName, "location-dot", "GPS", droneClickInfoBox, "off");
        convertCheckboxToToggleButton(routeCheckbox, droneName, "route", "PATH", toggleLayerVisibility, "on");
        convertCheckboxToToggleButton(videoCheckbox, droneName, "video", "LIVE", toggleVideoVisibility, "off");

        convertCheckboxToToggleButton(cvVideoCheckbox, droneName, "arrows-to-eye", "DET. LIVE", toggleDetVideoVisibility, "off");
        // convertCheckboxToToggleButton(weatherCheckbox, droneName, "cloud-sun", "WEATHER", droneClickWeatherBox, "off"); // TODO: make this appear conditionally
        // createWeatherBoxForDrone(droneName);

        // special treatment for detection toggle button
        var detectionToggleState = "off";
        var droneDetectionState = getDroneDetectionState(droneName);
        // console.log(droneDetectionState);

        if (droneDetectionState === DETECTION_CONNECTED) {
            // start_detection_vid(droneName);
            let allDrones = get_all_drone_info_array();
            for (let i = 0; i < allDrones.length; i++) {
                if (droneName === allDrones[i].droneID) {
                    // update_drone_detection_status_locally(i, DETECTION_CONNECTED);
                    setDroneAttributeValueByIndex(i, "droneDetectionStatus", DETECTION_CONNECTED);
                    break;
                }
            }
            detectionToggleState = "on";
        }
        
        convertCheckboxToToggleButton(cvCheckbox, droneName, "eye", "DETECT", toggleDetectionFunctionality, detectionToggleState);


        // MAVLINK Drones

        if(getDroneAttributeValue(droneName, "droneType") == "MAVLINK") {
            let mavlinkActions1 = document.createElement('div');
            $(mavlinkActions1).addClass('mavlink-wrapper');

            let mavlinkActions2 = document.createElement('div');
            $(mavlinkActions2).addClass('mavlink-wrapper');

            let takeoffBtn = createButton('primary', 'mavlink-takeoff-land-' + droneName, droneName, "plane-departure", "Takeoff", handleMavlinkTakeoffLand);
            let returnHomeBtn = createButton('primary', 'mavlink-return-' + droneName, droneName, "house", "Return", handleMavlinkReturn);
            let transitionBtn = createButton('primary', 'mavlink-transition-' + droneName, droneName, "shuffle", "Transition", handleMavlinkTransition);            
            let armBtn = createButton('primary', 'mavlink-set-speed-' + droneName, droneName, "gauge-high", "Set Speed", handleMavlinkSetSpeed);
            // let disarmBtn = createButton('warning', 'mavlink-disarm-' + droneName, droneName, "plug-circle-xmark", "Disarm", handleMavlinkReturn);
            let killBtn = createButton('danger', 'mavlink-kill-' + droneName, droneName, "skull", "Kill", handleMavlinkKill);

            mavlinkActions1.appendChild(takeoffBtn);
            mavlinkActions1.appendChild(returnHomeBtn);
            mavlinkActions1.appendChild(transitionBtn);
            mavlinkActions2.appendChild(armBtn);
            // mavlinkActions2.appendChild(disarmBtn);
            mavlinkActions2.appendChild(killBtn);            
            
            droneWrapperDiv.appendChild(mavlinkActions1);
            droneWrapperDiv.appendChild(mavlinkActions2);
        }
    }
}










function handleMavlinkTakeoffLand(_droneName) {
    droneState = getDroneAttributeValue(_droneName, "droneState");
    if (droneState == "Landed") {
        handleMavlinkTakeoff(_droneName);
    }
    else {
        handleMavlinkLand(_droneName);
    }
}


function handleMavlinkTakeoff(_droneName) {
    let title = _droneName + ' Take-Off';
    let msg = '';
    let confirmButton = 'Take-Off';
    let label = 'Take-Off Altitude:';
    let subtext = 'Enter a value between 10 and 120 meters.'

    create_confirmation_dialog_with_input('number', confirmButton, 'Cancel', msg, title, label, subtext, "50").then(function (inputValue) {
        if (inputValue !== null) {
            if (parseInt(inputValue) >= 10 && parseInt(inputValue) <= 120) { // validate takeoff altitude
                console.log("Taking off to " + inputValue + "m.");
                showPopupForALittle('#successBox', "Sending takeoff command to "+_droneName, 5000);
                url = "mavlinkTakeoff"
                data = {
                    droneName: _droneName,
                    altitude: inputValue,
                }
                postMavlinkRequest(url, data);
            }
            else {
                console.log("Invalid takeoff altitude!");
                showPopupForALittle('#failureBox', "Take-Off aborted: invalid takeoff altitude!", 5000);
            }
        }
        else {
            console.log("Take-Off cancelled!");
            showPopupForALittle('#failureBox', "Take-Off cancelled.", 2000);

        }
    });
}



function handleMavlinkLand(_droneName) {
    let msg = 'Are you sure you want land <b>"' + _droneName + '"</b>? <br />';

    create_confirmation_dialog('Land', 'Cancel', msg, 'Landing Confirmation').then(function (canProceed) {
        if (canProceed) {
            console.log("Landing...");
            showPopupForALittle('#successBox', "Sending land command to " + _droneName, 5000);
            url = "mavlinkLand"
            data = {
                droneName: _droneName,
            }
            postMavlinkRequest(url, data);
        }
        else {
            console.log("Landing cancelled!");
            showPopupForALittle('#failureBox', "Landing cancelled.", 2000);

        }
    });
}




function handleMavlinkReturn(_droneName) {
    let msg = 'The UAV will ascend to its pre-defined altitude (if needed) and navigate to the location from where it took off. ' +
        'Once reaching its takeoff location it will begin its descent and proceed to land. <br /><br />' +
        'Please consult the UAV\'s user manual for more information on the return altitude and transitioning between fixed-wing and multicopter (if applicable). <br /><br />' +
        'Are you sure you want <b>"' + _droneName + '"</b> to return home? <br />';
    create_confirmation_dialog('Return to Home', 'Cancel', msg, 'Return to Home Confirmation').then(function (canProceed) {
        if (canProceed) {
            console.log("Return to home " + _droneName);
            showPopupForALittle('#successBox', "Sending 'Return to Home' command to " + _droneName, 5000);
            url = "mavlinkReturnHome"
            data = {
                droneName: _droneName,
            }
            postMavlinkRequest(url, data);            
        }
        else {
            console.log("Return to home cancelled!");
            showPopupForALittle('#failureBox', "Cancelled.", 2000);
        }
    });
}


function handleMavlinkTransition(_droneName) {
    let title = _droneName + ' Transition';
    let msg = '';
    let transitionTo = '';

    getDroneAttributeValue(_droneName, "vtolState") == "MC" ? msg = 'Transition to Fixed-wing?' : msg = 'Transition to Multicopter?';
    getDroneAttributeValue(_droneName, "vtolState") == "MC" ? transitionTo = 'FW' : transitionTo = 'MC';

    create_confirmation_dialog('Transition', 'Cancel', msg, title).then(function (canProceed) {
        if (canProceed) {
            console.log("Transition " + _droneName + " to " + transitionTo);
            showPopupForALittle('#successBox', "Sending transition command to " + _droneName, 5000);
            url = "mavlinkTransition"
            data = {
                droneName: _droneName,
                mode: transitionTo,
            }
            postMavlinkRequest(url, data);
        }
        else {
            console.log("Transition cancelled!");
            showPopupForALittle('#failureBox', "Transition cancelled.", 2000);
        }
    });
}




function handleMavlinkSetSpeed(_droneName) {
    let title = _droneName + ' Set Speed';
    let msg = '';
    let label = 'Speed:';
    let subtext = 'Enter speed in meters per second (m/s).'

    create_confirmation_dialog_with_input('number', 'Set Speed', 'Cancel', msg, title, label, subtext, "15").then(function (inputValue) {
        if (inputValue !== null) {
            if (parseInt(inputValue) >= 2) { // validate speed
                console.log("Setting speed to " + inputValue + "m/s.");
                showPopupForALittle('#successBox', "Sending speed command to " + _droneName, 5000);
                url = "mavlinkSetSpeed"
                data = {
                    droneName: _droneName,
                    speed: inputValue,
                }
                postMavlinkRequest(url, data);
            }
            else {
                console.log("Invalid speed value!");
                showPopupForALittle('#failureBox', "Set speed aborted: invalid speed value!", 5000);
            }
        }
        else {
            console.log("Set speed cancelled!");
            showPopupForALittle('#failureBox', "Set speed cancelled.", 2000);

        }
    });
}



function handleMavlinkKill(_droneName) {
    let title = _droneName + ' Kill';
    let msg = '<b>DANGER:</b> This will disarm a drone irrespective of whether it is landed or flying. Note that the drone will fall out of the sky if this command is used while flying.';
    let label = 'Type "KILL" to confirm:';
    let subtext = ''

    create_confirmation_dialog_with_input('text', 'Kill', 'Cancel', msg, title, label, subtext, "").then(function (inputValue) {
        if (inputValue !== null) {
            if (inputValue == "KILL") {
                console.log("Kill " + _droneName);
                showPopupForALittle('#successBox', "Sending KILL command to " + _droneName, 5000);
                url = "mavlinkKill"
                data = {
                    droneName: _droneName,
                }
                postMavlinkRequest(url, data);
            }
            else {
                console.log("Invalid transition mode!");
                showPopupForALittle('#failureBox', "KILL aborted!", 5000);
            }
        }
        else {
            console.log("Kill cancelled!");
            showPopupForALittle('#failureBox', "KILL cancelled.", 2000);

        }
    });
}



async function postMavlinkRequest(_url, _data) {
    let settings = {
        url: _url,
        method: "POST",
        headers: {
            "Content-Type": "application/json",
            "X-CSRFToken": document.getElementById("csrf").querySelector("input").value,
        },
        data: JSON.stringify(_data),
    };
    let sessionId;
    await $.ajax(settings).done(function (response) {
        sessionId = response.data;
    });
    return sessionId;
}




// removes drones from the side-panel
function removeDronesFromSidePanel(_deletedDroneNames) {
    for (let i = 0; i < _deletedDroneNames.length; i++) {
        console.log("Drone: " + _deletedDroneNames[i] + " disconnected.");
        $('#sidepanel-drone-' + _deletedDroneNames[i]).remove();
    }
}




/**********************/
/****** DEVICES *******/
/**********************/



// add devices to the side-panel
function addNewDevicesToSidePanel(_deviceNames) {
    for (let i = 0; i < _deviceNames.length; i++) {
        console.log("Device: " + _deviceNames[i] + " connected.");
        let deviceName = _deviceNames[i];
        let ul = document.getElementById('device-selection-list');

        let li = document.createElement('li');
        li.id = 'mobile-device-' + deviceName;
        ul.append(li);
        
        // create the row element for this device
        let deviceWrapperDiv = document.createElement('div');
        // $(deviceWrapperDiv).addClass('row');
        deviceWrapperDiv.style.fontSize = '13px';
        deviceWrapperDiv.style.marginBottom = '15px';
        deviceWrapperDiv.style.color = '#ffffff';
        deviceWrapperDiv.style.lineHeight = '2';
        deviceWrapperDiv.style.background = 'inherit';
        li.append(deviceWrapperDiv);

        // create and add the device's name
        let deviceHeader = document.createElement('div');
        $(deviceHeader).addClass('row client-header');
        let deviceNameHeader = document.createElement('div');
        deviceNameHeader.id = 'mobile-device-name-' + deviceName;
        $(deviceNameHeader).addClass('col-md-6');
        deviceNameHeader.innerHTML = "<a href='#' style='padding: 0px; opacity: 1;' onclick='zoomToClient(\"device\", \"" + deviceName + "\")'>" + deviceName + "</a>";            

        let deviceBatteryHeader = document.createElement('div');
        deviceBatteryHeader.id = 'mobile-device-battery-' + deviceName;
        $(deviceBatteryHeader).addClass('col-md-2 client-header-data');
        deviceBatteryHeader.innerHTML = "...%";
        $(deviceBatteryHeader).attr("data-toggle", "tooltip");
        $(deviceBatteryHeader).attr("title", "Battery level");   

        deviceHeader.appendChild(deviceNameHeader);
        deviceHeader.appendChild(deviceBatteryHeader);
        deviceWrapperDiv.appendChild(deviceHeader);
        
        // create the divs that will contain the toggle buttons
        let togglesRow = document.createElement('div');
        togglesRow.style.marginBottom = '4px';

        // create and append the checkboxes
        let selectCheckbox = createCheckbox('device-select-toggle-' + deviceName);
        let gpsCheckbox = createCheckbox('device-gps-toggle-' + deviceName);
        let routeCheckbox = createCheckbox('device-route-toggle-' + deviceName);
        togglesRow.appendChild(selectCheckbox);
        togglesRow.appendChild(gpsCheckbox);
        togglesRow.appendChild(routeCheckbox);

        // append the div holding the checkboxes to the wrapper div
        // this must be done before converting checkboxes to toggles
        deviceWrapperDiv.appendChild(togglesRow);

        convertCheckboxToToggleButton(selectCheckbox, deviceName, "check", "SELECT", toggleSelectDevice, "off");
        convertCheckboxToToggleButton(gpsCheckbox, deviceName, "location-dot", "GPS", deviceClickInfoBox, "off");
        convertCheckboxToToggleButton(routeCheckbox, deviceName, "route", "PATH", toggleLayerVisibility, "on");

    }
}


// removes devices from the side-panel
function removeDevicesFromSidePanel(_deletedDevicesNames) {
    for (let i = 0; i < _deletedDevicesNames.length; i++) {
        console.log("Device: " + _deletedDevicesNames[i] + " disconnected.");
        $('#mobile-device-' + _deletedDevicesNames[i]).remove();
    }
}




/**********************/
/****** BALORA *******/
/**********************/



// add balora to the side-panel
function addNewBalorasToSidePanel(_baloraNames) {
    for (let i = 0; i < _baloraNames.length; i++) {
        console.log("Balora: " + _baloraNames[i] + " connected.");
        let baloraName = _baloraNames[i];
        let ul = document.getElementById('balora-selection-list');

        let li = document.createElement('li');
        li.id = 'lora-tracker-' + baloraName;
        ul.append(li);
        
        // create the row element for this balora
        let baloraWrapperDiv = document.createElement('div');
        baloraWrapperDiv.style.fontSize = '13px';
        baloraWrapperDiv.style.marginBottom = '15px';
        baloraWrapperDiv.style.color = '#ffffff';
        baloraWrapperDiv.style.lineHeight = '2';
        baloraWrapperDiv.style.background = 'inherit';
        li.append(baloraWrapperDiv);

        // create and add the balora's name
        let baloraHeader = document.createElement('div');
        $(baloraHeader).addClass('row client-header');
        let baloraNameHeader = document.createElement('div');
        baloraNameHeader.id = 'lora-tracker-name-' + baloraName;
        $(baloraNameHeader).addClass('col-md-6');
        baloraNameHeader.innerHTML = "<a href='#' style='padding: 0px; opacity: 1;' onclick='zoomToClient(\"balora\", \"" + baloraName + "\")'>" + baloraName + "</a>";

        let baloraBatteryHeader = document.createElement('div');
        baloraBatteryHeader.id = 'lora-tracker-battery-' + baloraName;
        $(baloraBatteryHeader).addClass('col-md-2 client-header-data');
        baloraBatteryHeader.innerHTML = "...%";
        $(baloraBatteryHeader).attr("data-toggle", "tooltip");
        $(baloraBatteryHeader).attr("title", "Battery level");   

        baloraHeader.appendChild(baloraNameHeader);
        baloraHeader.appendChild(baloraBatteryHeader);
        baloraWrapperDiv.appendChild(baloraHeader);
        
        // create the divs that will contain the toggle buttons
        let togglesRow = document.createElement('div');
        togglesRow.style.marginBottom = '4px';

        // create and append the checkboxes
        let selectCheckbox = createCheckbox('balora-select-toggle-' + baloraName);
        let gpsCheckbox = createCheckbox('balora-gps-toggle-' + baloraName);
        let routeCheckbox = createCheckbox('balora-route-toggle-' + baloraName);
        togglesRow.appendChild(selectCheckbox);
        togglesRow.appendChild(gpsCheckbox);
        togglesRow.appendChild(routeCheckbox);

        // append the div holding the checkboxes to the wrapper div
        // this must be done before converting checkboxes to toggles
        baloraWrapperDiv.appendChild(togglesRow);

        convertCheckboxToToggleButton(selectCheckbox, baloraName, "check", "SELECT", toggleSelectBalora, "off");
        convertCheckboxToToggleButton(gpsCheckbox, baloraName, "location-dot", "GPS", baloraClickInfoBox, "off");
        convertCheckboxToToggleButton(routeCheckbox, baloraName, "route", "PATH", toggleLayerVisibility, "on");

    }
}


// removes balora from the side-panel
function removeBalorasFromSidePanel(_deletedBalorasNames) {
    for (let i = 0; i < _deletedBalorasNames.length; i++) {
        console.log("Balora: " + _deletedBalorasNames[i] + " disconnected.");
        $('#lora-tracker-' + _deletedBalorasNames[i]).remove();
    }
}



/**********************/
/******* UTILS ********/
/**********************/


function createCheckbox(_id) {
    let cb = document.createElement('input');
    cb.id = _id;
    cb.type = 'checkbox';
    return cb;
}

function convertCheckboxToToggleButton(_cb, _clientName, _icon, _text, _callbackFunction, _initialState) {
    // var iconWidth = 21;
    // var charWidth = 11;
    // var strLen = _text.length;
    // var width = iconWidth + (strLen * charWidth);
    // _cb.setAttribute('data-width', width)
    _cb.setAttribute('data-width', '90')
    _cb.setAttribute('data-height', '26')
    $(_cb).bootstrapToggle({
        on: "<i class='fa fa-" + _icon + "'></i> " + _text,
        off: "<i class='fa fa-" + _icon + "'></i> " + _text
    });
    $(_cb).bootstrapToggle(_initialState)
    $(_cb).on('change', function (event) {
        _callbackFunction(this.id, _clientName);
    });
}


function createButton(_cssClass, _id, _clientName, _icon, _text, _callbackFunction) {
    let btn = document.createElement('button');
    btn.id = _id;
    btn.innerHTML = "<i class='fa fa-" + _icon + "'></i> " + _text;
    btn.classList.add("btn", "btn-sm", "btn-" + _cssClass, "col", "mavlink-btn"); // , "col-md-4"
    // btn.setAttribute('data-width', '90')
    $(btn).on('click', function (event) {
        _callbackFunction(_clientName);
    });
    return btn;
}





function toggleSelectDrone(toggleID, droneid) {
    var pressed = $('#' + toggleID).is(':checked');
    postElementId('Select ' + droneid, pressed);
    let allDrones = get_all_drone_info_array();
    for (let i = 0; i < allDrones.length; i++) {
        if (droneid === allDrones[i].droneID) {
            if (pressed) {
                //If toggle is checked but drone not selected, select it
                select_toggled_drone(i);
                return;
            } else if (!pressed) {
                // if toggle is unchecked and drone selected, UNSELECT IT
                unselect_toggled_drone(i);
                return;
            }
        }
    }
}

function toggleSelectDevice(toggleID, deviceid) {
    let pressed = $('#' + toggleID).is(':checked');
    postElementId('Select ' + deviceid, pressed);
    console.log('post request');
    let alldevices = get_all_device_info_array();
    for (let i = 0; i < alldevices.length; i++) {
        if (deviceid === alldevices[i].deviceID) {
            if (pressed) {
                //If toggle is checked but device not selected, select it
                select_toggled_device(i);
                return;
            } else if (!pressed) {
                // if toggle is unchecked and device selected, UNSELECT IT
                unselect_toggled_device(i);
                return;
            }
        }
    }
}

function toggleSelectBalora(toggleID, baloraid) {
    let pressed = $('#' + toggleID).is(':checked');
    postElementId('Select ' + baloraid, pressed);
    let allbaloras = get_all_balora_info_array();
    for (let i = 0; i < allbaloras.length; i++) {
        if (baloraid === allbaloras[i].baloraID) {
            if (pressed) {
                //If toggle is checked but balora not selected, select it
                select_toggled_balora(i);
                return;
            } else if (!pressed) {
                // if toggle is unchecked and balora selected, UNSELECT IT
                unselect_toggled_balora(i);
                return;
            }
        }
    }
}






// function add_list_items_to_selected_device(device_ids) {
//     for (let i = 0; i < device_ids.length; i++) {
//         let deviceID = device_ids[i];
//         let ul = document.getElementById('device-selection-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode('Device ' + deviceID);

//         ul.append(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'device-toggle' + deviceID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.id = 'list-item-select-' + deviceID;
//         li.style.width = '270px';

//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             toggleSelectDevice(this.id, deviceID);
//         });
//     }
// }




// function add_list_items_to_uav_missions(drone_ids) {
//     for (let i = 0; i < drone_ids.length; i++) {
//         let droneID = drone_ids[i];
//         let ul = document.getElementById('drone-selection-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(droneID);

//         ul.append(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'drone-toggle' + droneID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.id = 'list-item-build-map-' + droneID;

//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             toggleSelectDrone(this.id, droneID);
//         });
//     }
// }





















// function add_list_items_to_trajectories_device(device_ids) {
//     for (let i = 0; i < device_ids.length; i++) {
//         let deviceID = device_ids[i];
//         let ul = document.getElementById('trajectory-list-device');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(deviceID);

//         // ul.style.background="#3d3e3f"
//         ul.appendChild(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);

//         toggle.id = 'trajectories-toggle-' + deviceID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.style.paddingTop = '4px';
//         li.style.paddingBottom = '4px';
//         li.style.width = '250px';
//         li.id = 'list-item-trajectories-' + deviceID;
//         // li.style.background="inherit"

//         $(toggle).bootstrapToggle('on');
//         $(toggle).on('change', function (event) {
//             toggleLayerVisibility(this.id, deviceID);
//         });
//     }
// }


function add_list_items_to_selected_balora(balora_ids) {
    for (let i = 0; i < balora_ids.length; i++) {
        let baloraID = balora_ids[i];
        let ul = document.getElementById('balora-selection-list');
        let li = document.createElement('li');
        let div = document.createElement('div');
        let toggle = document.createElement('input');
        let linkText = document.createTextNode('Balora ' + baloraID);

        ul.append(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'balora-toggle' + baloraID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-select-' + baloraID;
        li.style.width = '270px';

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            toggleSelectBalora(this.id, baloraID);
        });
    }
}

function add_list_items_to_trajectories_balora(balora_ids) {
    for (let i = 0; i < balora_ids.length; i++) {
        let baloraID = balora_ids[i];
        let ul = document.getElementById('trajectory-list-balora');
        let li = document.createElement('li');
        let div = document.createElement('div');
        let toggle = document.createElement('input');
        let linkText = document.createTextNode(baloraID);

        // ul.style.background="#3d3e3f"
        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);

        toggle.id = 'trajectories-toggle-' + baloraID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.style.paddingTop = '4px';
        li.style.paddingBottom = '4px';
        li.style.width = '250px';
        li.id = 'list-item-trajectories-' + baloraID;
        // li.style.background="inherit"

        $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            toggleLayerVisibility(this.id, baloraID);
        });
    }
}




function add_list_items_to_map_tools_weather(drone_ids) {
    if (document.getElementById('drone-weather-station-list-default')) {
        document.getElementById('drone-weather-station-list-default').remove();
    }    
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        let ul = document.getElementById('drone-weather-station-list');
        let li = document.createElement('li');
        let div = document.createElement('m1');
        let toggle = document.createElement('input');

        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(document.createTextNode(droneID));

        toggle.id = 'drone-weather-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-drone-weather-' + droneID;

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            droneClickWeatherBox(this.id, droneID);
        });
        createWeatherBoxForDrone(droneID);
    }
}



function add_list_items_to_uav_missions_lidar(drone_ids) {
    if (document.getElementById('lidar-data-list-default')) {
        document.getElementById('lidar-data-list-default').remove();
    }
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        let li = document.createElement('li');
        let m1 = document.createElement('m1');
        let m2 = document.createElement('m2');
        li.id = 'drone-lidar-list-' + drone_ids;
        li.append(m1);
        li.append(m2);
        // m1.style.paddingLeft = '0px';
        m1.innerHTML = droneID;
        m2.innerHTML =
            `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="startLidarPointColection" onclick="startLidarPoints('` +
            droneID +
            `')" type="button">START</button>
        <button aria-pressed="false" class="btn btn-outline-danger panel-btn bldmp" id="StopLidarPointColection" onclick="stopLidarPoints('` +
            droneID +
            `')" type="button">STOP</button>`;
        document.getElementById('drone-lidar-list').appendChild(li);
    }
}

function add_list_items_to_multispectral_build_map() {
    let available_indices = MULTISPECTRAL_INDEX_NAMES;
    for (let i = 0; i < available_indices.length; i++) {
        let indice = available_indices[i];
        let ul = document.getElementById('uav-multispectral-build-map-list');
        let li = document.createElement('li');
        let div = document.createElement('div');
        let toggle = document.createElement('input');

        ul.append(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'build-map-index-toggle-' + indice;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-build-map-' + indice;

        $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            console.log('CHANGE DETECTED');
            toggleMultispectralPhotoLayer(this.id, indice);
        });
    }
}
function add_list_items_to_uav_missions_water_sampler(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        if (document.getElementById('water-sampler-data-list-default')) {
            document.getElementById('water-sampler-data-list-default').remove();
        }
        let droneID = drone_ids[i];

        // let ul = document.getElementById("drone-selection-list");
        let li = document.createElement('li');
        let m1 = document.createElement('m1');
        let m2 = document.createElement('m2');
        li.append(m1);
        li.append(m2);
        m1.innerHTML = droneID;
        m2.innerHTML =
            `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="water_sampler" onclick="activate_water_sampler('` +
            droneID +
            `')" type="button">START</button>`;
        document.getElementById('water-collector-list').appendChild(li);
    }
}

// function add_list_items_to_video_feeds(drone_ids) {
//     for (let i = 0; i < drone_ids.length; i++) {
//         let droneID = drone_ids[i];
//         let ul = document.getElementById('video-feeds-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(droneID);

//         ul.appendChild(li);

//         $(div).addClass('d-flex justify-content-between ');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'video-toggle-' + droneID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.style.width = '270px';
//         li.append(div);
//         li.id = 'list-item-video-' + droneID;

//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             toggleVideoVisibility(this.id, droneID);
//         });
//     }
// }

// function add_list_items_to_det_video_feeds(drone_ids) {
//     for (let i = 0; i < drone_ids.length; i++) {
//         let droneID = drone_ids[i];
//         let ul = document.getElementById('det-video-feeds-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(droneID);

//         ul.appendChild(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'det-video-toggle-' + droneID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.style.width = '270px';
//         li.id = 'list-item-det-video-' + droneID;

//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             toggleDetVideoVisibility(this.id, droneID);
//         });
//     }
// }

// function add_list_items_to_map_tools_drones(listOfDroneNames) {
//     listOfDroneNames.forEach((droneName) => {
//         let ul = document.getElementById('drone-tools-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(droneName + ' GPS');

//         ul.appendChild(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);

//         toggle.id = 'drone-info-' + droneName;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.style.paddingTop = '4px';
//         li.style.paddingBottom = '4px';
//         li.style.width = '250px';
//         li.id = 'list-item-drone-info-' + droneName;
//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             droneClickInfoBox(this.id, droneName);
//         });
//     });
// }
// function add_list_items_to_map_tools_devices(listOfDeviceNames) {
//     listOfDeviceNames.forEach((deviceName) => {
//         if ((get_all_device_info_array().find((device) => device.deviceID === deviceName) || null) !== null) {
//             let ul = document.getElementById('device-tools-list');
//             let li = document.createElement('li');
//             let div = document.createElement('div');
//             let toggle = document.createElement('input');
//             ul.appendChild(li);
//             div.appendChild(document.createTextNode(deviceName + ' GPS'));
//             toggle.id = 'device-info-' + deviceName;
//             toggle.type = 'checkbox';
//             $(div).addClass('d-flex justify-content-between');
//             div.appendChild(toggle);
//             li.append(div);
//             li.style.paddingTop = '4px';
//             li.style.paddingBottom = '4px';
//             li.style.width = '250px';
//             li.id = 'list-item-device-info-' + deviceName;
//             $(toggle).bootstrapToggle('off');
//             $(toggle).on('change', function (event) {
//                 deviceClickInfoBox(deviceName);
//             });
//         }
//     });
// }

function add_list_items_to_map_tools_baloras(listOfDeviceNames) {
    listOfDeviceNames.forEach((baloraName) => {
        if ((get_all_balora_info_array().find((balora) => balora.baloraID === baloraName) || null) !== null) {
            let ul = document.getElementById('balora-tools-list');
            let li = document.createElement('li');
            let div = document.createElement('div');
            let toggle = document.createElement('input');
            ul.appendChild(li);
            div.appendChild(document.createTextNode(baloraName + ' GPS'));
            toggle.id = 'balora-info-' + baloraName;
            toggle.type = 'checkbox';
            $(div).addClass('d-flex justify-content-between');
            div.appendChild(toggle);
            li.append(div);
            li.style.paddingTop = '4px';
            li.style.paddingBottom = '4px';
            li.style.width = '250px';
            li.id = 'list-item-balora-info-' + baloraName;
            $(toggle).bootstrapToggle('off');
            $(toggle).on('change', function (event) {
                baloraClickInfoBox(baloraName);
            });
        }
    });
}


function add_no_drones_text_on_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        if (!$('#no-drones-list-' + element_id).length) {
            let ul = document.getElementById(element_id);
            let li = document.createElement('li');
            li.id = 'no-drones-list-' + element_id;
            li.append('No drones currently connected.');
            li.style.color = WHITE_COLOR;
            li.style.paddingBottom = '5px';
            ul.append(li);
        }
    });
}

function remove_no_drones_text_from_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        $('#no-drones-list-' + element_id).remove();
    });
}

function add_no_devices_text_on_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        if (!$('#no-device-list-' + element_id).length) {
            let ul = document.getElementById(element_id);
            let li = document.createElement('li');
            li.id = 'no-device-list-' + element_id;
            li.append('No phones currently connected.');
            li.style.color = WHITE_COLOR;
            li.style.paddingBottom = '5px';
            ul.append(li);
        }
    });
}

function remove_no_devices_text_from_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        $('#no-device-list-' + element_id).remove();
    });
}

function add_no_baloras_text_on_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        if (!$('#no-balora-list-' + element_id).length) {
            let ul = document.getElementById(element_id);
            let li = document.createElement('li');
            li.id = 'no-balora-list-' + element_id;
            li.append('No trackers currently connected.');
            li.style.color = WHITE_COLOR;
            li.style.paddingBottom = '5px';
            ul.append(li);
        }
    });
}

function remove_no_baloras_text_from_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        $('#no-balora-list-' + element_id).remove();
        if (element_id === 'balora-sensors-list') {
            let ul = document.getElementById('balora-sensors-list');
            let li = document.createElement('li');
            let div = document.createElement('div');
            let toggle = document.createElement('input');
            let linkText = document.createTextNode('PM2.5');

            ul.append(li);

            $(div).addClass('d-flex justify-content-between');
            div.appendChild(linkText);
            div.style.fontSize = '13px';
            div.style.color = '#ffffff';
            div.style.lineHeight = '2';
            div.style.background = 'inherit';

            toggle.id = 'balora-pm25-toggle';
            toggle.type = 'checkbox';
            div.appendChild(toggle);

            li.append(div);
            li.id = 'list-sensor-select-pm25';
            li.style.width = '270px';

            $(toggle).bootstrapToggle('off');
            $(toggle).on('change', function (event) {
                getBaloraPM25SensorData(this);
            });
        }
    });
}

function remove_list_items_to_map_tools(deleted_ids, element_id) {
    for (let i = 0; i < deleted_ids.length; i++) {
        $('#list-item-' + element_id + deleted_ids[i]).remove();
    }
}
function remove_list_items_to_map_lidar(deleted_ids) {
    for (let i = 0; i < deleted_ids.length; i++) {
        $('#drone-lidar-list-' + deleted_ids[i]).remove();
    }
}

function remove_list_items_from_trajectories(deleted_ids) {
    for (let i = 0; i < deleted_ids.length; i++) {
        $('#list-item-trajectories-' + deleted_ids[i]).remove();
    }
}

function remove_list_items_from_uav_missions(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#drone-selection-list' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_video_feeds(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-video-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_det_video_feeds(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-det-video-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_detection_types() {
    let prototypeModels = get_prototype_models();

    for (let i = 0; i < prototypeModels.length; i++) {
        $('#list-item-detection-types-' + prototypeModels[i].type).remove();
    }

    $('#list-item-detection-types-All').remove();
}

function remove_list_items_from_select_drones(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-build-map-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_selected_device(deleted_devices_ids) {
    for (let i = 0; i < deleted_devices_ids.length; i++) {
        $('#list-item-select-' + deleted_devices_ids[i]).remove();
    }
}

function remove_list_items_from_selected_balora(deleted_baloras_ids) {
    for (let i = 0; i < deleted_baloras_ids.length; i++) {
        $('#list-item-select-' + deleted_baloras_ids[i]).remove();
    }
}

/*
 * Fired when any radio button is selected. The map's style is then changed according to the chosen button
 * */
function changeMapStyle(layerid, layer_type) {
    console.log("changeMapStyle");
    postElementId('Change map style ' + layerid.id, 'Click');
    let urlVector = 'https://api.maptiler.com/maps/<>/style.json?key=blpQOMdNw0JJIq07I9Ln';
    let currentStyle;
    let lastDroneLocation;
    if (layer_type === MAPLIBRE_STYLE) {
        // clear_timer(droneTimer);
        let allDrones = get_all_drone_info_array();

        if (allDrones.length === 0) {
            lastDroneLocation = DEFAULT_MAP_CENTER;
        } else {
            lastDroneLocation = allDrones[0].droneInfo.currentCoordinate;
        }
        currentStyle = urlVector.replace('<>', layerid);
        map.setStyle(currentStyle);
    }
    // else if (layer_type === OWN_LAYER)
    // {
    //     currentStyle = offlineStyle
    //     map.setStyle(offlineStyle)
    // }

    sessionStorage.setItem('currentStyle', currentStyle);
    sessionStorage.setItem('currentStyleRadioBtn', layerid);
    sessionStorage.setItem('lastDroneLocation', JSON.stringify(lastDroneLocation));
    window.location.reload();
}


// load a list of available drone sessions in a popup dialog
function showAvailableDroneSessions(type) {
    let settings = {
        url: "/api/operations/" + CURRENT_OP + "/getAvailableDroneSessions/" + type,
        method: "GET",
    };
    $.ajax(settings).done(function (_response) {
        createPopupDialogForLoadClearProcess(
            JSON.parse(_response),
            "Select Session to view"
        ).then(function (selectedSessions) {
            let sessionId = selectedSessions[selectedSessions.length - 1].sessionId;
            window.open('/drone_session_replay/' + type + '/' + sessionId, '_blank');
        });
    });
}


// load a list of available device sessions in a popup dialog
function showAvailableDeviceSessions() {
    let settings = {
        url: "/api/operations/" + CURRENT_OP + "/getAvailableDeviceSessions",
        method: "GET",
    };
    $.ajax(settings).done(function (_response) {
        createPopupDialogForLoadClearProcess(
            JSON.parse(_response),
            "Select Session to view",
            false
        ).then(function (selectedSessions) {
            let sessionId = selectedSessions[selectedSessions.length - 1].sessionId;
            window.open('/device_session_replay/' + sessionId, '_blank');
        });
    });
}


/*Fired when the toggle button about the video is pressed.
 * It toggles the video's frame visibility
 * */
function toggleVideoVisibility(toggleID, droneID) {
    let pressed = $('#' + toggleID).is(':checked');
    let x = document.getElementById('live-stream-wrapper-div-' + droneID);

    if (pressed) {
        x.style.position = 'absolute';
        let offset = getNumberOfOverlayPanels() * 20;
        x.style.top = offset + 'px';
        x.style.left = offset + 'px';        
        x.style.display = 'block';
        // start_live_stream(droneID);
    } else {
        // let actionButtonID = 'live-feed-action-btn-' + droneID;
        // let actionBtn = document.getElementById(actionButtonID);

        // //If live stream is on when user decides to toggle off the
        // //live video, then before making the video disappear, pause it.
        // console.log('ACTION BTN INNER HTML: ', actionBtn.textContent.charCodeAt(0));
        // let pauseBtnCode = '9612';
        // let currentBtnCode = actionBtn.textContent.charCodeAt(0);
        // if (currentBtnCode == pauseBtnCode) {
        //     //The pause button is displayed currently. Means that video is currently playing
        //     console.log('will now PAUSE video1');
        //     actionBtn.click();
        // }
        x.style.display = 'none';
    }
}

function toggleDetVideoVisibility(toggleID, droneID) {
    let pressed = $('#' + toggleID).is(':checked');
    postElementId('Show Detection Video', pressed);
    let x = document.getElementById('detection-stream-wrapper-div-' + droneID);
    if (pressed) {
        x.style.position = 'absolute';
        let offset = getNumberOfOverlayPanels() * 20;
        x.style.top = offset + 'px';
        x.style.left = offset + 'px';
        x.style.display = 'block';
    } else {
        x.style.display = 'none';
    }
}

function toggleSearchVisibility(searchBoxID) {
    let pressed = $('#' + searchBoxID).is(':checked');
    postElementId('Show Search Box', pressed);
    let x = document.getElementById('geocoder');

    if (pressed) {
        x.style.display = 'block';
    } else {
        x.style.display = 'none';
    }
}

/*Triggered when toggle buttons are checked.
 * It toggles the layers' visibility
 * */
function toggleLayerVisibility(toggleID, ID) {
    // if (!getIfConstantsAreDeclaredFromAPI() || WEB_SERVER_URL === undefined)
    // {
    //     create_popup_for_a_little(WARNING_ALERT,"Layer needs few more seconds to be initialized!",2000)
    //     return
    // }
    let allDrones = get_all_drone_info_array();
    let allDevices = get_all_device_info_array();
    let allBaloras = get_all_balora_info_array();
    let layerID;
    let toggleElement = $('#' + toggleID);
    let pressed = $(toggleElement).is(':checked');
    let found = false;
    let layerLoaded;
    let selectedLayer;

    for (let i = 0; i < allDrones.length; i++) {
        if (allDrones[i].droneID === ID) {
            postElementId('Line ' + allDrones[i].droneID, pressed);
            layerID = allDrones[i].droneLineLayer.id;
            found = true;
            break;
        }
    }
    for (let i = 0; i < allDevices.length; i++) {
        if (allDevices[i].deviceID === ID) {
            postElementId('Line ' + allDevices[i].deviceID, pressed);
            layerID = allDevices[i].deviceLineLayer.id;
            found = true;
            break;
        }
    }
    for (let i = 0; i < allBaloras.length; i++) {
        if (allBaloras[i].baloraID === ID) {
            postElementId('Line ' + allBaloras[i].baloraID, pressed);
            layerID = allBaloras[i].baloraLineLayer.id;
            found = true;
            break;
        }
    }
    if (!found) {
        switch (toggleID) {
            case 'threedBuildingsToggle':
                selectedLayer = threeDbuildingLayer;
                break;

            case 'roadLayerToggle':
                selectedLayer = layerRoads;
                if (pressed) {
                    postElementId('Layer Roads', pressed);
                    add_layer_on_map(
                        layerRoads.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_roadnetwork_original.geojson',
                        }),
                        layerRoads,
                        'geojson'
                    );
                }
                break;

            case 'buildingsLayerToggle':
                selectedLayer = layerBuildings;
                if (pressed) {
                    postElementId('Layer Buildings', pressed);
                    add_layer_on_map(
                        layerBuildings.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_buildings.geojson',
                        }),
                        layerBuildings,
                        'geojson',
                        'None',
                        'None',
                        0.1,
                        true,
                        3.5
                    );
                }

                break;

            case 'damsLayerToggle':
                selectedLayer = layerDams;
                if (pressed) {
                    postElementId('Layer Dams', pressed);
                    add_layer_on_map(
                        layerDams.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_dams.geojson',
                        }),
                        layerDams,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'platform_geojson_files_dam_icon.png',
                        }),
                        '#000000',
                        0.5,
                        true
                    );
                }

                break;

            case 'hospitalsLayerToggle':
                selectedLayer = layerHospitals;
                if (pressed) {
                    postElementId('Layer Hospitals', pressed);
                    add_layer_on_map(
                        layerHospitals.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_hospitals.geojson',
                        }),
                        layerHospitals,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'platform_geojson_files_hospital_icon.png',
                        }),
                        '#ff0000',
                        0.5,
                        true
                    );
                }
                break;

            case 'contoutEleLayerToggle':
                selectedLayer = layerTerrainLines;
                if (pressed) {
                    postElementId('Layer Terrain', pressed);
                    add_terrain_lines_layer();
                }
                break;

            case 'polesLayerToggle':
                selectedLayer = layerPoles;
                if (pressed) {
                    postElementId('Layer Poles', pressed);
                    add_layer_on_map(
                        layerPoles.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_aikpilwnes.geojson',
                        }),
                        layerPoles,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'platform_geojson_files_mv_pole.png',
                        }),
                        '#000000',
                        0.5,
                        true
                    );
                }
                break;

            case 'polesLinesToggle':
                selectedLayer = layerPoleLines;
                if (pressed) {
                    postElementId('Layer Poles Lines', pressed);
                    add_layer_on_map(
                        layerPoleLines.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_aikpilwnes_lines.geojson',
                        }),
                        layerPoleLines,
                        'geojson'
                    );
                }
                break;

            case 'restrictedAllowedAreasForDroneToggle':
                selectedLayer = layerRestrictedAllowedAreasForDrone;
                if (pressed) {
                    postElementId('Layer Rest/Allow Area', pressed);
                    add_layer_on_map(
                        layerRestrictedAllowedAreasForDrone.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'cyprus_restricted_allowed_areas_for_Drones.geojson',
                        }),
                        layerRestrictedAllowedAreasForDrone,
                        'geojson'
                    );
                }
                break;

            case 'cyprusFirToggle':
                selectedLayer = layercyprusFir;
                if (pressed) {
                    postElementId('Layer Cyprus FIR', pressed);
                    add_layer_on_map(
                        layercyprusFir.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'cyprus_fir_nicosia.geojson',
                        }),
                        layercyprusFir,
                        'geojson'
                    );
                }
                break;

            case 'operationAreasToggle':
                selectedLayer = layerOperationAreas;
                if (pressed) {
                    postElementId('Layer Operation Areas', pressed);
                    add_layer_on_map(
                        layerOperationAreas.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'cyprus_operation_areas.geojson',
                        }),
                        layerOperationAreas,
                        'geojson'
                    );
                }
                break;
                // case '3DTerrainToggle':
                //     selectedLayer = layerTerrainLines;
                //     if (pressed) {

                //         // map.addLayer( {
                //         //     id: 'hills',
                //         //     type: 'hillshade',
                //         //     source: 'hillshadeSource',
                //         //     layout: { visibility: 'visible' },
                //         //     paint: { 'hillshade-shadow-color': '#473B24' }
                //         // },
                //         // 'osm' );
                
                //         map.setTerrain( { source: 'terrainSource' ,  exaggeration: 3 } )
                //     }
                //     break;
        }
        if (pressed) {
            let msg = 'Layer is loading. Please wait...';
            let successMsg = 'Layer successfully loaded!';
            create_popup(WARNING_ALERT, msg, 'tempPopup');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
                create_popup_for_a_little(SUCCESS_ALERT, successMsg, 1000);
            });
        } else if (map.getLayer(selectedLayer.id)) {
            map.removeLayer(selectedLayer.id);
        }
    }
    if (layerID !== undefined) {
        if (pressed) {
            let successMsg = 'Layer successfully visible!';
            map.setLayoutProperty(layerID, 'visibility', 'visible');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
                create_popup_for_a_little(SUCCESS_ALERT, successMsg, 1000);
            });
        } else {
            map.setLayoutProperty(layerID, 'visibility', 'none');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
            });
        }
    }
}

function layer_loading() {}

var canVisualizeObjects = true;

function handle_layer_loading_popup(layer_id, pressed) {
    let okButton = 'OK';

    let dialogTitle = 'Warning';
    // if(layerExists(layer_id) && pressed)
    // {
    //     let message = "Please wait a few seconds until the layer is done loading!"
    //     showPopupForALittle('#box_layerLoading',message,2000)
    // }
    if (!layerExists(layer_id) && pressed) {
        //Layer did not load yet.
        let message = 'Layer needs a few seconds to load. Please try again in a few seconds';
        create_dialog_with_one_button(okButton, message, dialogTitle, 750, 'auto');
        return false;
    }
    return true;
}
function checkPrototypeCheckbox(toggleID) {
    let detection_arrays = get_detected_objs();
    let detected_models = detection_arrays[0];
    let detected_api_objs = detection_arrays[1];
    var pressed = $('#' + toggleID).is(':checked');

    var all_dets_toggle_pressed = $('#' + ALL_DETS_TOGGLE_ID).is(':checked');
    var person_toggle_pressed = $('#' + PERSON_DETS_TOGGLE_ID).is(':checked');
    var car_toggle_pressed = $('#' + CAR_DETS_TOGGLE_ID).is(':checked');
    var motor_toggle_pressed = $('#' + MOTOR_DETS_TOGGLE_ID).is(':checked');
    console.log('TOGGLE ID: ' + toggleID);
    console.log('IS IT PRESSED? ' + pressed);

    if (pressed) {
        if (toggleID === SHOW_DETS_TOGGLE_ID) {
            document.getElementById(ALL_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(CAR_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(MOTOR_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(PERSON_DETS_TOGGLE_ID).disabled = false;
        }
        console.log('all dets pressed: ' + all_dets_toggle_pressed);
        console.log('person_dets_pressed: ' + person_toggle_pressed);
        console.log('car_dets_pressed: ' + car_toggle_pressed);
        console.log('tree_dets_pressed: ' + motor_toggle_pressed);

        // canVisualizeObjects = true
        // for (let i=0; i<detected_objs.length; i++)
        // {
        //     detected_objs[i].visibility = true
        // }

        for (let i = 0; i < detected_models.length; i++) {
            if (car_toggle_pressed && detected_api_objs[i].type === CAR) {
                console.log('OVER_IN_CARS');
                detected_models[i].visibility = true;
            } else if (person_toggle_pressed && detected_api_objs[i].type === PERSON) {
                detected_models[i].visibility = true;
            } else if (motor_toggle_pressed && detected_api_objs[i].type === MOTORBIKE) {
                detected_models[i].visibility = true;
            } else if (all_dets_toggle_pressed) {
                detected_models[i].visibility = true;
            }
        }
    } else {
        if (toggleID === SHOW_DETS_TOGGLE_ID) {
            document.getElementById(ALL_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(CAR_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(MOTOR_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(PERSON_DETS_TOGGLE_ID).disabled = true;
        }

        // console.log("OVER_OVER")
        // canVisualizeObjects = false
        // for (let i=0; i<detected_objs.length; i++)
        // {
        //     detected_objs[i].visibility = false
        // }

        for (let i = 0; i < detected_models.length; i++) {
            if (toggleID === CAR_DETS_TOGGLE_ID && detected_api_objs[i].type === CAR) {
                detected_models[i].visibility = false;
            } else if (toggleID === PERSON_DETS_TOGGLE_ID && detected_api_objs[i].type === PERSON) {
                detected_models[i].visibility = false;
            } else if (toggleID === MOTOR_DETS_TOGGLE_ID && detected_api_objs[i].type === MOTORBIKE) {
                detected_models[i].visibility = false;
            } else if (toggleID === ALL_DETS_TOGGLE_ID) {
                //User wants to remove all detected objects from map
                detected_models[i].visibility = false;

                // console.log("car_dets_pressed: " + car_toggle_pressed)
                // console.log("object type: " + detected_api_objs[i].type)
                // let none_from_above = true

                // if (person_toggle_pressed && detected_api_objs[i].type === PERSON) //Don't remove people if people toggle is ON
                // {
                //     detected_models[i].visibility = true
                // }
                // else if (car_toggle_pressed && detected_api_objs[i].type === CAR)
                // {
                //     detected_models[i].visibility = true
                // }
                // else if (motor_toggle_pressed && detected_api_objs[i].type === MOTORBIKE)
                // {
                //     detected_models[i].visibility = true
                // }
                // else
                // {
                //     detected_models[i].visibility = false
                // }
            } else if (toggleID === SHOW_DETS_TOGGLE_ID) {
                detected_models[i].visibility = false;
            }
        }
    }
}

function toggleCrowdLocalization(_checked) {
	showOrHideCrowdLocalizationResultsLayer(_checked);
	manageWebsocketForCrowdLocalization(_checked);
}
function toggleDissasterClassification(_checked) {
	showOrHideDisasterClassificationResultsLayer(_checked);
	manageWebsocketForDisasterClassification(_checked);
}
function toggleVechicleAndPersonTracker(_checked) {
	// vehicle-and-person-tracker-visual-option class allow selected
	var elements = document.getElementsByClassName("vehicle-and-person-tracker-visual-option");
	// Loop through each element
	for (var i = 0; i < elements.length; i++) {
		// If the element is a checkbox
		if (elements[i].type === "checkbox") {
			elements[i].disabled = !_checked;
		}
	}
	if (!_checked) {
		let options = Array.from(document.getElementsByClassName("vehicle-and-person-tracker-visual-option"));
		for (let i = 0; i < options.length; i++) {
			let option = options[i];
			option.checked = false;
			vechicleAndPersonTrackerOptionClick();
		}
	}
	manageWebsocketForVehicleAndPersonTracker(_checked);
}

function vechicleAndPersonTrackerOptionClick() {
	let detection_arrays = get_detected_objs()[0];
	let detected_api_objs = get_detected_objs()[1];
	let prototypeModels = get_prototype_models();

	if (detection_arrays) {
		let options = Array.from(document.getElementsByClassName("vehicle-and-person-tracker-visual-option"));
		let isAnyOptionChecked = false;

		for (let i = 0; i < options.length; i++) {
			let option = options[i];

			if (option.checked) {
				isAnyOptionChecked = true;

				if (option.value == "all") {
					detection_arrays.forEach((detectedObject) => {
						detectedObject.visibility = true;
					});
					break;
				} else {
					for (let j = 0; j < detected_api_objs.length; j++) {
						let objType = detected_api_objs[j].label;
						let checkboxID = getCheckboxID(objType, prototypeModels);
						let isCBchecked = $("#" + checkboxID).is(":checked");
						detection_arrays[j].visibility = isCBchecked;
					}
				}
			}
		}

		if (!isAnyOptionChecked) {
			detection_arrays.forEach((detectedObject) => {
				detectedObject.visibility = false;
			});
		}
	} else {
		console.error("No detected objects found.");
	}
}

function getCheckboxID(objType, protModels) {
	for (let i = 0; i < protModels.length; i++) {
		if (objType === protModels[i].type) {
			return protModels[i].checkboxID;
		}
	}
	return "NO CHECKBOX ID FOUND";
}
