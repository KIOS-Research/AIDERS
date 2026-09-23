/**********************/
/******* DRONES *******/
/**********************/


// adds drones to the side-panel
function addNewDronesToSidePanel(droneName) {
    console.log("Drone: " + droneName + " connected.");
    const droneType = getDroneAttributeValue(droneName, "droneType");
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
    if (droneType == "MAVLINK") {
        brandLogo = "<img src='/static/aiders/imgs/mavlink-logo.png' width='22' style='margin-right:6px;' title='MAVLink' />";
    }
    else {
        brandLogo = "<img src='/static/aiders/imgs/dji-logo.png' width='22' style='margin-right:6px;' title='DJI' />";
    }

    droneNameHeader.innerHTML = "<a href='#' style='padding: 0px; opacity: 1;' onclick='zoomToClient(\"drone\", \"" + droneName + "\")'>" + brandLogo + droneName + "</a>";

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
    let pilotNotificationCheckbox = createCheckbox('pilot-notification-toggle-' + droneName);

    
    
    togglesRow1.appendChild(selectCheckbox);
    togglesRow1.appendChild(gpsCheckbox);
    togglesRow1.appendChild(routeCheckbox);

    togglesRow2.appendChild(videoCheckbox);
    if (USER_EXECUTE_COMMAND_PERMISSION == "True" || USER_SUPERUSER_PERMISSION == "True") {
        togglesRow2.appendChild(cvCheckbox)
        if (droneType == "DJI") {
            togglesRow3.appendChild(pilotNotificationCheckbox);
        }        
    }
    togglesRow2.appendChild(cvVideoCheckbox);

    // togglesRow2.appendChild(weatherCheckbox);


    // append the div holding the checkboxes to the wrapper div
    // this must be done before converting checkboxes to toggles
    droneWrapperDiv.appendChild(togglesRow1);
    droneWrapperDiv.appendChild(togglesRow2);
    droneWrapperDiv.appendChild(togglesRow3);

    convertCheckboxToToggleButton(selectCheckbox, droneName, "check", "SELECT", toggleSelectDrone, "off");
    convertCheckboxToToggleButton(gpsCheckbox, droneName, "location-dot", "GPS", changeDroneTelemetryElementVisibility, "off");
    convertCheckboxToToggleButton(routeCheckbox, droneName, "route", "PATH", toggleLayerVisibility, "on");
    convertCheckboxToToggleButton(videoCheckbox, droneName, "video", "LIVE", toggleVideoVisibility, "off");

    convertCheckboxToToggleButton(cvVideoCheckbox, droneName, "arrows-to-eye", "DET. LIVE", toggleDetVideoVisibility, "off");
    // convertCheckboxToToggleButton(weatherCheckbox, droneName, "cloud-sun", "WEATHER", droneClickWeatherBox, "off"); // TODO: make this appear conditionally
    // createWeatherBoxForDrone(droneName);

    convertCheckboxToToggleButton(pilotNotificationCheckbox, droneName, "exclamation-triangle", "NOTIFY", sendMessageToPilot, "off");    


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

    if (getDroneAttributeValue(droneName, "droneType") == "MAVLINK") {
        let mavlinkActions1 = document.createElement('div');
        $(mavlinkActions1).addClass('mavlink-wrapper');

        let mavlinkActions2 = document.createElement('div');
        $(mavlinkActions2).addClass('mavlink-wrapper');

        let takeoffBtn = createButton('primary', 'mavlink-takeoff-land-' + droneName, droneName, "plane-departure", "Takeoff", handleMavlinkTakeoffLand);
        let returnHomeBtn = createButton('primary', 'mavlink-return-' + droneName, droneName, "house", "Return", handleMavlinkReturn);
        // let transitionBtn = createButton('primary', 'mavlink-transition-' + droneName, droneName, "shuffle", "Transition", handleMavlinkTransition);
        let transitionBtnMC = createButton('primary', 'mavlink-transition-mc-' + droneName, droneName, "helicopter", "Trans. MC", handleMavlinkTransition, "MC");
        let transitionBtnFW = createButton('primary', 'mavlink-transition-fw-' + droneName, droneName, "plane", "Trans. FW", handleMavlinkTransition, "FW");
        let setSpeedBtn = createButton('primary', 'mavlink-set-speed-' + droneName, droneName, "gauge-high", "Set Speed", handleMavlinkSetSpeed);
        // let disarmBtn = createButton('warning', 'mavlink-disarm-' + droneName, droneName, "plug-circle-xmark", "Disarm", handleMavlinkReturn);
        let killBtn = createButton('danger', 'mavlink-kill-' + droneName, droneName, "skull", "Kill", handleMavlinkKill);

        mavlinkActions1.appendChild(takeoffBtn);
        mavlinkActions1.appendChild(returnHomeBtn);
        mavlinkActions1.appendChild(setSpeedBtn);
        if (getDroneAttributeValue(droneName, "droneConfiguration") == "VTOL") {
            // mavlinkActions2.appendChild(transitionBtn);
            mavlinkActions2.appendChild(transitionBtnMC);
            mavlinkActions2.appendChild(transitionBtnFW);
        }
        // mavlinkActions2.appendChild(disarmBtn);
        mavlinkActions2.appendChild(killBtn);

        droneWrapperDiv.appendChild(mavlinkActions1);
        droneWrapperDiv.appendChild(mavlinkActions2);
    }

    // console.log(getDroneAttributeValue(droneName, "droneConfiguration"));
    
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

    create_confirmation_dialog_with_input('number', confirmButton, 'Cancel', msg, title, label, subtext, "10").then(function (inputValue) {
        if (inputValue !== null) {
            if (parseInt(inputValue) >= 10 && parseInt(inputValue) <= 120) { // validate takeoff altitude
                console.log("Taking off to " + inputValue + "m.");
                showPopupForALittle('#successBox', "Sending takeoff command to " + _droneName, 5000);
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


function handleMavlinkTransition(_droneName, _tempTransitionMode) {
    let title = _droneName + ' Transition';
    let msg = '';
    let transitionTo = '';

    // TODO: TEMP: until we have a better way to determine the current state of the drone
    _tempTransitionMode == "FW" ? msg = 'Transition to Fixed-wing?' : msg = 'Transition to Multicopter?';
    _tempTransitionMode == "FW" ? transitionTo = 'FW' : transitionTo = 'MC';

    // getDroneAttributeValue(_droneName, "vtolState") == "MC" ? msg = 'Transition to Fixed-wing?' : msg = 'Transition to Multicopter?';
    // getDroneAttributeValue(_droneName, "vtolState") == "MC" ? transitionTo = 'FW' : transitionTo = 'MC';

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
function removeDronesFromSidePanel(_deleteDroneName) {
    console.log("Drone: " + _deleteDroneName + " disconnected.");
    $('#sidepanel-drone-' + _deleteDroneName).remove();
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
        let togglesRow2 = document.createElement('div');
        togglesRow2.style.marginBottom = '4px';

        // create and append the checkboxes
        let selectCheckbox = createCheckbox('device-select-toggle-' + deviceName);
        let gpsCheckbox = createCheckbox('device-gps-toggle-' + deviceName);
        let routeCheckbox = createCheckbox('device-route-toggle-' + deviceName);
        let deviceNotificationCheckbox = createCheckbox('device-notification-toggle-' + deviceName);

        togglesRow.appendChild(selectCheckbox);
        togglesRow.appendChild(gpsCheckbox);
        togglesRow.appendChild(routeCheckbox);
        togglesRow2.appendChild(deviceNotificationCheckbox);

        // append the div holding the checkboxes to the wrapper div
        // this must be done before converting checkboxes to toggles
        deviceWrapperDiv.appendChild(togglesRow);
        deviceWrapperDiv.appendChild(togglesRow2);

        convertCheckboxToToggleButton(selectCheckbox, deviceName, "check", "SELECT", toggleSelectDevice, "off");
        convertCheckboxToToggleButton(gpsCheckbox, deviceName, "location-dot", "GPS", deviceClickInfoBox, "off");
        convertCheckboxToToggleButton(routeCheckbox, deviceName, "route", "PATH", toggleLayerVisibility, "on");

        convertCheckboxToToggleButton(deviceNotificationCheckbox, deviceName, "exclamation-triangle", "NOTIFY", sendMessageToDevice, "off");    


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


function createButton(_cssClass, _id, _clientName, _icon, _text, _callbackFunction, _extraParam = null) {
    let btn = document.createElement('button');
    btn.id = _id;
    btn.innerHTML = "<i class='fa fa-" + _icon + "'></i> " + _text;
    btn.classList.add("btn", "btn-sm", "btn-" + _cssClass, "col", "mavlink-btn"); // , "col-md-4"
    // btn.setAttribute('data-width', '90')
    $(btn).on('click', function (event) {
        if (_extraParam) {
            _callbackFunction(_clientName, _extraParam);
        }
        else {
            _callbackFunction(_clientName);
        }
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





// NOT USED
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



// NOT USED
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

// NOT USED
// function add_list_items_to_selected_balora(balora_ids) {
//     for (let i = 0; i < balora_ids.length; i++) {
//         let baloraID = balora_ids[i];
//         let ul = document.getElementById('balora-selection-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode('Balora ' + baloraID);

//         ul.append(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'balora-toggle' + baloraID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.id = 'list-item-select-' + baloraID;
//         li.style.width = '270px';

//         $(toggle).bootstrapToggle('off');
//         $(toggle).on('change', function (event) {
//             toggleSelectBalora(this.id, baloraID);
//         });
//     }
// }

// NOT USED
// function add_list_items_to_trajectories_balora(balora_ids) {
//     for (let i = 0; i < balora_ids.length; i++) {
//         let baloraID = balora_ids[i];
//         let ul = document.getElementById('trajectory-list-balora');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');
//         let linkText = document.createTextNode(baloraID);

//         // ul.style.background="#3d3e3f"
//         ul.appendChild(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);

//         toggle.id = 'trajectories-toggle-' + baloraID;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.style.paddingTop = '4px';
//         li.style.paddingBottom = '4px';
//         li.style.width = '250px';
//         li.id = 'list-item-trajectories-' + baloraID;
//         // li.style.background="inherit"

//         $(toggle).bootstrapToggle('on');
//         $(toggle).on('change', function (event) {
//             toggleLayerVisibility(this.id, baloraID);
//         });
//     }
// }




function addDroneToPanelListWeatherStation(_droneID) {
    document.getElementById('drone-weather-station-list-default').style.display = 'none';
    let ul = document.getElementById('drone-weather-station-list');
    let li = document.createElement('li');
    let div = document.createElement('m1');
    let toggle = document.createElement('input');

    ul.appendChild(li);

    $(div).addClass('d-flex justify-content-between');
    div.appendChild(document.createTextNode(_droneID));

    toggle.id = 'drone-weather-' + _droneID;
    toggle.type = 'checkbox';
    div.appendChild(toggle);

    li.append(div);
    li.id = 'list-item-drone-weather-' + _droneID;

    $(toggle).bootstrapToggle('off');
    $(toggle).on('change', function (event) {
        droneClickWeatherBox(this.id, _droneID);
    });
    createWeatherBoxForDrone(_droneID);
}

function removeDroneToPanelListWeatherStation(_droneID) {
    if (document.getElementById('list-item-drone-weather-' + _droneID)) {
        document.getElementById('list-item-drone-weather-' + _droneID).remove();
    }
    if (document.querySelectorAll('#drone-weather-station-list li').length == 1) {
        document.getElementById('drone-weather-station-list-default').style.display = 'block';
    }
}


function addDroneToPanelListLidar(_droneName) {
    document.getElementById('lidar-data-list-default').style.display = 'none';
    let li = document.createElement('li');
    let m1 = document.createElement('m1');
    let m2 = document.createElement('m2');
    li.id = 'drone-lidar-list-' + _droneName;
    li.append(m1);
    li.append(m2);
    // m1.style.paddingLeft = '0px';
    m1.innerHTML = _droneName;
    m2.innerHTML =
        `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="startLidarPointColection`+ _droneName +`" onclick="startLidarPoints('` +
        _droneName +
        `')" type="button">START</button>
    <button aria-pressed="false" class="btn btn-outline-danger panel-btn bldmp" id="StopLidarPointColection`+ _droneName +`" onclick="stopLidarPoints('` +
        _droneName +
        `')" type="button">STOP</button>`;
    document.getElementById('drone-lidar-list').appendChild(li);
}

function removeDroneToPanelListLidar(_droneName) {
    if (document.getElementById('drone-lidar-list-' + _droneName)) {
        document.getElementById('drone-lidar-list-' + _droneName).remove();
    }
    if (document.querySelectorAll('#drone-lidar-list li').length == 1) {
        document.getElementById('lidar-data-list-default').style.display = 'block';
    }
}

// NOT USED
// function add_list_items_to_multispectral_build_map() {
//     let available_indices = MULTISPECTRAL_INDEX_NAMES;
//     for (let i = 0; i < available_indices.length; i++) {
//         let indice = available_indices[i];
//         let ul = document.getElementById('uav-multispectral-build-map-list');
//         let li = document.createElement('li');
//         let div = document.createElement('div');
//         let toggle = document.createElement('input');

//         ul.append(li);

//         $(div).addClass('d-flex justify-content-between');
//         div.appendChild(linkText);
//         div.style.fontSize = '13px';
//         div.style.color = '#ffffff';
//         div.style.padding = '.15rem 0.55rem';
//         div.style.lineHeight = '2';
//         div.style.background = 'inherit';

//         toggle.id = 'build-map-index-toggle-' + indice;
//         toggle.type = 'checkbox';
//         div.appendChild(toggle);

//         li.append(div);
//         li.id = 'list-item-build-map-' + indice;

//         $(toggle).bootstrapToggle('on');
//         $(toggle).on('change', function (event) {
//             console.log('CHANGE DETECTED');
//             toggleMultispectralPhotoLayer(this.id, indice);
//         });
//     }
// }
function addDroneToPanelListWaterSamper(_droneName) {
    document.getElementById('water-sampler-data-list-default').style.display = 'none';

    // let ul = document.getElementById("drone-selection-list");
    let li = document.createElement('li');
    let m1 = document.createElement('m1');
    let m2 = document.createElement('m2');
    li.id = 'water-collector-list-' + _droneName;
    li.append(m1);
    li.append(m2);
    m1.innerHTML = _droneName;
    m2.innerHTML =
        `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="water_sampler" onclick="activate_water_sampler('` +
        _droneName +
        `')" type="button">START</button>`;
    document.getElementById('water-collector-list').appendChild(li);
}

function removeDroneToPanelListWaterSamper(_droneName) {
    if (document.getElementById('water-collector-list-' + _droneName)) {
        document.getElementById('water-collector-list-' + _droneName).remove();
    }
    if (document.querySelectorAll('#water-collector-list li').length == 1) {
        document.getElementById('water-sampler-data-list-default').style.display = 'block';
    }
}
// NOT USED
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
// NOT USED
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
// NOT USED
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
// NOT USED
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

            // PM2.5 Sensor
            let liPM25 = document.createElement('li');
            let divPM25 = document.createElement('div');
            let radioPM25 = document.createElement('input');
            let linkTextPM25 = document.createTextNode('PM2.5');

            ul.append(liPM25);

            $(divPM25).addClass('d-flex justify-content-between');
            divPM25.appendChild(linkTextPM25);
            divPM25.style.fontSize = '13px';
            divPM25.style.color = '#ffffff';
            divPM25.style.lineHeight = '2';
            divPM25.style.background = 'inherit';

            radioPM25.id = 'balora-pm25-radio';
            radioPM25.type = 'radio';
            radioPM25.name = 'balora-sensors'; // Shared name for radio group
            divPM25.appendChild(radioPM25);

            liPM25.append(divPM25);
            liPM25.id = 'list-sensor-select-pm25';
            liPM25.style.width = '270px';

            $(radioPM25).on('change', function () {
                if (activePopup) {
                    activePopup.remove();
                    activePopup = null;
                }
                getBaloraPM25SensorData(this);
                $('.mapboxgl-popup').remove();
            });

            // PM1 Sensor
            let liPM1 = document.createElement('li');
            let divPM1 = document.createElement('div');
            let radioPM1 = document.createElement('input');
            let linkTextPM1 = document.createTextNode('PM1');

            ul.append(liPM1);

            $(divPM1).addClass('d-flex justify-content-between');
            divPM1.appendChild(linkTextPM1);
            divPM1.style.fontSize = '13px';
            divPM1.style.color = '#ffffff';
            divPM1.style.lineHeight = '2';
            divPM1.style.background = 'inherit';

            radioPM1.id = 'balora-pm1-radio';
            radioPM1.type = 'radio';
            radioPM1.name = 'balora-sensors'; // Shared name for radio group
            divPM1.appendChild(radioPM1);

            liPM1.append(divPM1);
            liPM1.id = 'list-sensor-select-pm1';
            liPM1.style.width = '270px';

            $(radioPM1).on('change', function () {
                if (activePopup) {
                    activePopup.remove();
                    activePopup = null;
                }
                getBaloraPM1SensorData(this);
                $('.mapboxgl-popup').remove();
            });

            // NOx Sensor
            let liNOX = document.createElement('li');
            let divNOX = document.createElement('div');
            let radioNOX = document.createElement('input');
            let linkTextNOX = document.createTextNode('NOx');

            ul.append(liNOX);

            $(divNOX).addClass('d-flex justify-content-between');
            divNOX.appendChild(linkTextNOX);
            divNOX.style.fontSize = '13px';
            divNOX.style.color = '#ffffff';
            divNOX.style.lineHeight = '2';
            divNOX.style.background = 'inherit';

            radioNOX.id = 'balora-nox-radio';
            radioNOX.type = 'radio';
            radioNOX.name = 'balora-sensors'; // Shared name for radio group
            divNOX.appendChild(radioNOX);

            liNOX.append(divNOX);
            liNOX.id = 'list-sensor-select-nox';
            liNOX.style.width = '270px';

            $(radioNOX).on('change', function () {
                if (activePopup) {
                    activePopup.remove();
                    activePopup = null;
                }
                getBaloraNOxSensorData(this);
                $('.mapboxgl-popup').remove();
            });

            // VOC Sensor
            let liVOC = document.createElement('li');
            let divVOC = document.createElement('div');
            let radioVOC = document.createElement('input');
            let linkTextVOC = document.createTextNode('VOC');

            ul.append(liVOC);

            $(divVOC).addClass('d-flex justify-content-between');
            divVOC.appendChild(linkTextVOC);
            divVOC.style.fontSize = '13px';
            divVOC.style.color = '#ffffff';
            divVOC.style.lineHeight = '2';
            divVOC.style.background = 'inherit';

            radioVOC.id = 'balora-voc-radio';
            radioVOC.type = 'radio';
            radioVOC.name = 'balora-sensors'; // Shared name for radio group
            divVOC.appendChild(radioVOC);

            liVOC.append(divVOC);
            liVOC.id = 'list-sensor-select-voc';
            liVOC.style.width = '270px';

            $(radioVOC).on('change', function () {
                if (activePopup) {
                    console.log("ABCD");
                    activePopup.remove();
                    activePopup = null;
                }
                getBaloraVOCSensorData(this);
                $('.mapboxgl-popup').remove();
            });
        }
    });
}


// function remove_no_baloras_text_from_panel_sections(section_element_ids) {
//     section_element_ids.forEach((element_id) => {
//         $('#no-balora-list-' + element_id).remove();
//         if (element_id === 'balora-sensors-list') {
//             let ul = document.getElementById('balora-sensors-list');
//             let li = document.createElement('li');
//             let div = document.createElement('div');
//             let toggle = document.createElement('input');
//             let linkText = document.createTextNode('PM2.5');

//             ul.append(li);

//             $(div).addClass('d-flex justify-content-between');
//             div.appendChild(linkText);
//             div.style.fontSize = '13px';
//             div.style.color = '#ffffff';
//             div.style.lineHeight = '2';
//             div.style.background = 'inherit';

//             toggle.id = 'balora-pm25-toggle';
//             toggle.type = 'checkbox';
//             div.appendChild(toggle);

//             li.append(div);
//             li.id = 'list-sensor-select-pm25';
//             li.style.width = '270px';

//             $(toggle).bootstrapToggle('off');
//             $(toggle).on('change', function (event) {
//                 getBaloraPM25SensorData(this);
//             });

//             // PM1 Sensor

//             let liPM1 = document.createElement('li');
//             let divPM1 = document.createElement('div');
//             let togglePM1 = document.createElement('input');
//             let linkTextPM1 = document.createTextNode('PM1');

//             ul.append(liPM1);

//             $(divPM1).addClass('d-flex justify-content-between');
//             divPM1.appendChild(linkTextPM1);
//             divPM1.style.fontSize = '13px';
//             divPM1.style.color = '#ffffff';
//             divPM1.style.lineHeight = '2';
//             divPM1.style.background = 'inherit';

//             togglePM1.id = 'balora-pm1-toggle';
//             togglePM1.type = 'checkbox';
//             divPM1.appendChild(togglePM1);

//             liPM1.append(divPM1);
//             liPM1.id = 'list-sensor-select-pm1';
//             liPM1.style.width = '270px';

//             $(togglePM1).bootstrapToggle('off');
//             $(togglePM1).on('change', function (event) {
//                 getBaloraPM1SensorData(this);
//             });


//             // NOX Sensor
//             let liNOX = document.createElement('li');
//             let divNOX = document.createElement('div');
//             let toggleNOX = document.createElement('input');
//             let linkTextNOX = document.createTextNode('NOx');

//             ul.append(liNOX);

//             $(divNOX).addClass('d-flex justify-content-between');
//             divNOX.appendChild(linkTextNOX);
//             divNOX.style.fontSize = '13px';
//             divNOX.style.color = '#ffffff';
//             divNOX.style.lineHeight = '2';
//             divNOX.style.background = 'inherit';

//             toggleNOX.id = 'balora-nox-toggle';
//             toggleNOX.type = 'checkbox';
//             divNOX.appendChild(toggleNOX);

//             liNOX.append(divNOX);
//             liNOX.id = 'list-sensor-select-nox';
//             liNOX.style.width = '270px';

//             $(toggleNOX).bootstrapToggle('off');
//             $(toggleNOX).on('change', function (event) {
//                 getBaloraNOxSensorData(this);
//             });

//             // VOC Sensor
//             let liVOC = document.createElement('li');
//             let divVOC = document.createElement('div');
//             let toggleVOC = document.createElement('input');
//             let linkTextVOC = document.createTextNode('VOC');

//             ul.append(liVOC);

//             $(divVOC).addClass('d-flex justify-content-between');
//             divVOC.appendChild(linkTextVOC);
//             divVOC.style.fontSize = '13px';
//             divVOC.style.color = '#ffffff';
//             divVOC.style.lineHeight = '2';
//             divVOC.style.background = 'inherit';

//             toggleVOC.id = 'balora-voc-toggle';
//             toggleVOC.type = 'checkbox';
//             divVOC.appendChild(toggleVOC);

//             liVOC.append(divVOC);
//             liVOC.id = 'list-sensor-select-voc';
//             liVOC.style.width = '270px';

//             $(toggleVOC).bootstrapToggle('off');
//             $(toggleVOC).on('change', function (event) {
//                 getBaloraVOCSensorData(this);
//             });

//             // Temperature Sensor
//             // let liTemp = document.createElement('li');
//             // let divTemp = document.createElement('div');
//             // let toggleTemp = document.createElement('input');
//             // let linkTextTemp = document.createTextNode('Temperature');

//             // ul.append(liTemp);

//             // $(divTemp).addClass('d-flex justify-content-between');
//             // divTemp.appendChild(linkTextTemp);
//             // divTemp.style.fontSize = '13px';
//             // divTemp.style.color = '#ffffff';
//             // divTemp.style.lineHeight = '2';
//             // divTemp.style.background = 'inherit';

//             // toggleTemp.id = 'balora-temperature-toggle';
//             // toggleTemp.type = 'checkbox';
//             // divTemp.appendChild(toggleTemp);

//             // liTemp.append(divTemp);
//             // liTemp.id = 'list-sensor-select-temperature';
//             // liTemp.style.width = '270px';

//             // $(toggleTemp).bootstrapToggle('off');
//             // $(toggleTemp).on('change', function (event) {
//             //     getBaloraTemperatureSensorData(this);
//             // });

//             // Humidity Sensor
//             // let liHumidity = document.createElement('li');
//             // let divHumidity = document.createElement('div');
//             // let toggleHumidity = document.createElement('input');
//             // let linkTextHumidity = document.createTextNode('Humidity');

//             // ul.append(liHumidity);

//             // $(divHumidity).addClass('d-flex justify-content-between');
//             // divHumidity.appendChild(linkTextHumidity);
//             // divHumidity.style.fontSize = '13px';
//             // divHumidity.style.color = '#ffffff';
//             // divHumidity.style.lineHeight = '2';
//             // divHumidity.style.background = 'inherit';

//             // toggleHumidity.id = 'balora-humidity-toggle';
//             // toggleHumidity.type = 'checkbox';
//             // divHumidity.appendChild(toggleHumidity);

//             // liHumidity.append(divHumidity);
//             // liHumidity.id = 'list-sensor-select-humidity';
//             // liHumidity.style.width = '270px';

//             // $(toggleHumidity).bootstrapToggle('off');
//             // $(toggleHumidity).on('change', function (event) {
//             //     getBaloraHumiditySensorData(this);
//             // });


//         }
//     });
// }
// NOT USED
// function remove_list_items_to_map_tools(deleted_ids, element_id) {
//     for (let i = 0; i < deleted_ids.length; i++) {
//         $('#list-item-' + element_id + deleted_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_to_map_lidar(deleted_ids) {
//     for (let i = 0; i < deleted_ids.length; i++) {
//         $('#drone-lidar-list-' + deleted_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_trajectories(deleted_ids) {
//     for (let i = 0; i < deleted_ids.length; i++) {
//         $('#list-item-trajectories-' + deleted_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_uav_missions(deleted_drones_ids) {
//     for (let i = 0; i < deleted_drones_ids.length; i++) {
//         $('#drone-selection-list' + deleted_drones_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_video_feeds(deleted_drones_ids) {
//     for (let i = 0; i < deleted_drones_ids.length; i++) {
//         $('#list-item-video-' + deleted_drones_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_det_video_feeds(deleted_drones_ids) {
//     for (let i = 0; i < deleted_drones_ids.length; i++) {
//         $('#list-item-det-video-' + deleted_drones_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_detection_types() {
//     let prototypeModels = get_prototype_models();

//     for (let i = 0; i < prototypeModels.length; i++) {
//         $('#list-item-detection-types-' + prototypeModels[i].type).remove();
//     }

//     $('#list-item-detection-types-All').remove();
// }

// function remove_list_items_from_select_drones(deleted_drones_ids) {
//     for (let i = 0; i < deleted_drones_ids.length; i++) {
//         $('#list-item-build-map-' + deleted_drones_ids[i]).remove();
//     }
// }
// NOT USED
// function remove_list_items_from_selected_device(deleted_devices_ids) {
//     for (let i = 0; i < deleted_devices_ids.length; i++) {
//         $('#list-item-select-' + deleted_devices_ids[i]).remove();
//     }
// }

// NOT USED
// function remove_list_items_from_selected_balora(deleted_baloras_ids) {
//     for (let i = 0; i < deleted_baloras_ids.length; i++) {
//         $('#list-item-select-' + deleted_baloras_ids[i]).remove();
//     }
// }

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
                        0.4,
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
                        0.4,
                        true
                    );
                }
                break;

            case 'fireStationsLayerToggle':
                selectedLayer = layerFireStations;
                if (pressed) {
                    postElementId('Layer Fire Stations', pressed);
                    add_layer_on_map(
                        layerFireStations.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_fire_stations.geojson',
                        }),
                        layerFireStations,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'platform_geojson_files_cyprus_fire_stations.png',
                        }),
                        '#d62828',
                        0.5,
                        true
                    );
                }
                break;

            case 'sheltersLayerToggle':
                selectedLayer = layerShelters;
                if (pressed) {
                    postElementId('Layer Shelters', pressed);
                    add_shelters_on_map(
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_shelters.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'shelter.png',
                        })
                    );
                } else {
                    /*The shelters and their clusters are three layers of the same toggle*/
                    remove_shelters_from_map();
                }
                break;

            case 'policeStationsLayerToggle':
                selectedLayer = layerPoliceStations;
                if (pressed) {
                    postElementId('Layer Police Stations', pressed);
                    add_police_stations_on_map(
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_police_stations_new.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'police-station.png',
                        })
                    );
                } else {
                    /*The stations and their boundaries are three layers of the same toggle*/
                    remove_police_stations_from_map();
                }
                break;

            case 'firebreaksLayerToggle':
                selectedLayer = layerFirebreaks;
                if (pressed) {
                    postElementId('Layer Firebreaks', pressed);
                    add_layer_on_map(
                        layerFirebreaks.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_firebreaks.geojson',
                        }),
                        layerFirebreaks,
                        'geojson',
                        'None',
                        'None',
                        0.1,
                        true
                    );
                }
                break;

            case 'forestStationsLayerToggle':
                selectedLayer = layerForestStations;
                if (pressed) {
                    postElementId('Layer Forest Stations', pressed);
                    add_clustered_layer_on_map(
                        layerForestStations,
                        forestStationClusterLayers,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_forest_stations.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'forest-station.png',
                        })
                    );
                } else {
                    /*The stations and their clusters are three layers of the same toggle*/
                    remove_clustered_layer_from_map(layerForestStations, forestStationClusterLayers);
                }
                break;

            case 'fireLookoutsLayerToggle':
                selectedLayer = layerFireLookouts;
                if (pressed) {
                    postElementId('Layer Fire Lookouts', pressed);
                    add_clustered_layer_on_map(
                        layerFireLookouts,
                        fireLookoutClusterLayers,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_fire_lookouts.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'fire-lookout.png',
                        })
                    );
                } else {
                    /*The lookouts and their clusters are three layers of the same toggle*/
                    remove_clustered_layer_from_map(layerFireLookouts, fireLookoutClusterLayers);
                }
                break;

            case 'heliportsLayerToggle':
                selectedLayer = layerHeliports;
                if (pressed) {
                    postElementId('Layer Heliports', pressed);
                    add_clustered_layer_on_map(
                        layerHeliports,
                        heliportClusterLayers,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_heliports.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'heliport.png',
                        })
                    );
                } else {
                    /*The heliports and their clusters are three layers of the same toggle*/
                    remove_clustered_layer_from_map(layerHeliports, heliportClusterLayers);
                }
                break;

            case 'fireHydrantsLayerToggle':
                selectedLayer = layerFireHydrants;
                if (pressed) {
                    postElementId('Layer Fire Hydrants', pressed);
                    add_clustered_layer_on_map(
                        layerFireHydrants,
                        fireHydrantClusterLayers,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_fire_hydrants.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'fire-hydrant.png',
                        })
                    );
                } else {
                    /*The hydrants and their clusters are three layers of the same toggle*/
                    remove_clustered_layer_from_map(layerFireHydrants, fireHydrantClusterLayers);
                }
                break;

            case 'explosivesStoresLayerToggle':
                selectedLayer = layerExplosivesStores;
                if (pressed) {
                    postElementId('Layer Explosives Stores', pressed);
                    add_clustered_layer_on_map(
                        layerExplosivesStores,
                        explosivesStoreClusterLayers,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'platform_geojson_files_cyprus_explosives_stores.geojson',
                        }),
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'explosives-store.png',
                        })
                    );
                } else {
                    /*The stores and their clusters are three layers of the same toggle*/
                    remove_clustered_layer_from_map(layerExplosivesStores, explosivesStoreClusterLayers);
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
                            geolocation_path: 'platform_geojson_files_aikpilwnes_new.geojson',
                        }),
                        layerPoles,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'platform_geojson_files_mv_pole.png',
                        }),
                        '#000000',
                        0.3,
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

            case 'geoZonesToggle':
                selectedLayer = layerGeoZones;
                if (pressed) {
                    postElementId('Layer Geo Zones', pressed);
                    add_layer_on_map(
                        layerGeoZones.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'cyprus_geo_zones.geojson',
                        }),
                        layerGeoZones,
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

            case 'populationLayerToggle':
                selectedLayer = layerPopulation;
                if (pressed) {
                    postElementId('Layer Population', pressed);
                    add_layer_on_map(
                        layerPopulation.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'postcodes_with_population.geojson',
                        }),
                        layerPopulation,
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
        } else if (selectedLayer !== undefined && map.getLayer(selectedLayer.id)) {
            /*Toggles that belong to no layer of the switch above (e.g. a drone's PATH toggle
             * whose drone was not found) leave selectedLayer undefined*/
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


function toggleCrowdLocalization(_checked) {
    showOrHideCrowdLocalizationResultsLayer(_checked);
    manageWebsocketForCrowdLocalizationVisualization(_checked);
}

function toggleDissasterClassification(_checked) {
    showOrHideDisasterClassificationResultsLayer(_checked);
    manageWebsocketForDisasterClassificationVisualization(_checked);
}

function toggleVehicleAndPersonTrackerVisualization(isChecked) {
    const checkboxes = Array.from(document.getElementsByClassName('vehicle-and-person-tracker-visual-option'));
    checkboxes.forEach((checkbox) => {
        checkbox.disabled = !isChecked;
        if (!isChecked) {
            checkbox.checked = false;
            vechicleAndPersonTrackerOptionClick(checkbox);
        }
    });
    manageWebsocketForVehicleAndPersonTrackerVisualization(isChecked);
}


function vechicleAndPersonTrackerOptionClick(_element) {
    if (_element.checked) {
        toggleModelVisibility(_element.value, true)
    } else {
        toggleModelVisibility(_element.value, false)
    }
}

function detectionObjectsAllButtonClick(_element) {
    document.querySelectorAll('.vehicle-and-person-tracker-visual-option').forEach(checkBox => {
        if (checkBox.value != 'all') {
            checkBox.checked = _element.checked
            checkBox.disabled = _element.checked
            vechicleAndPersonTrackerOptionClick(checkBox)
        }
    });
}
function getCheckboxID(objType, protModels) {
    for (let i = 0; i < protModels.length; i++) {
        if (objType === protModels[i].type) {
            return protModels[i].checkboxID;
        }
    }
    return "NO CHECKBOX ID FOUND";
}

