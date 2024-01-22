{
    let loadingBuildMapBoxTimer = {};
    let loadingToCancelBuildMapTimer;
    function popup_BuildMapLoading(action, drone_id, retryNumber) {
        if (action === SHOW_POPUP) {
            showPopup('#loadingBuildMapBox', 'Build Map loading. Please wait...');
            loadingBuildMapBoxTimer[drone_id] = setInterval(function () {
                retryNumber = retryNumber + 1;
                console.log('retryNumber: ' + retryNumber);
                hidePopup('#loadingBuildMapBox');
                if (retryNumber >= 4) {
                    hidePopup('#loadingBuildMapBox');
                    updateBuildMapStatusOnAPI(drone_id, false);
                    showPopupForALittle('#buildMapFailedToStart', '', 2000);
                    clearBuildMapLoadingPopupTimer(drone_id);
                } else {
                    let msg = 'Something went wrong! Build Map failed to start. Trying to start Build Map again...(' + retryNumber + ')';
                    showPopup('#loadingBuildMapBox', msg);
                    updateBuildMapStatusOnAPI(drone_id, true);
                }
            }, 10000);
        } else if (action === REMOVE_POPUP) {
            hidePopup('#loadingBuildMapBox');
        }
    }

    function clearBuildMapLoadingPopupTimer(drone_id) {
        if (loadingBuildMapBoxTimer[drone_id] !== undefined) {
            hidePopup('#loadingBuildMapBox');
            clearTimeout(loadingBuildMapBoxTimer[drone_id]);
        }
    }

    function clearLoadingToCancelBuildMapPopupTimer() {
        if (loadingToCancelBuildMapTimer !== undefined) {
            clearTimeout(loadingToCancelBuildMapTimer);
        }
    }

    function popup_BuildMapLoadingToStop(action, reasonToStop) {
        if (action === SHOW_POPUP) {
            if (reasonToStop === USER_WANTS_TO_STOP_IT) {
                showPopup('#buildMapLoadingToStop', '');
            } else if (reasonToStop === TOO_LOW_ALTITUDE_FOR_BUILD_MAP) {
                showPopup('#buildMapLoadingToStopDueToLowAltitude', '');
            }
            loadingToCancelBuildMapTimer = setTimeout(function () {
                hidePopup('#buildMapLoadingToStop');
                hidePopup('#buildMapLoadingToStopDueToLowAltitude');
                showPopupForALittle('#buildMapFailedToStop', '', 3000);
            }, 7000);
        } else if (action === REMOVE_POPUP) {
            hidePopup('#buildMapLoadingToStop');
        }
    }

    function highlightElement(element, action) {
        if (action === ACTIVATE_HIGHLIGHT) {
            $(element).addClass('active');
        } else if (action === DEACTIVATE_HIGHLIGHT) {
            $(element).removeClass('active');
        }
    }

    function showPopupForALittle(popup_element, message, duration_ms) {
        if (message !== '') {
            $(popup_element).html(message);
        }
        $(popup_element).show();

        setTimeout(function () {
            // $(popup_element).hide()
            $(popup_element).fadeOut();
        }, duration_ms);
    }

    function hidePopup(popup_element) {
        $(popup_element).hide();
    }

    function removePopupByElement(element) {
        element.remove();
    }

    function removePopupByID(el_id) {
        $(el_id).remove();
    }
    function showPopup(popup_element, message) {
        if (message !== '') {
            $(popup_element).html(message);
        }
        $(popup_element).show();
    }

    function create_form_with_checkboxes_for_load_build_map(buildMapPeriods) {
        let iDiv = document.createElement('div');
        iDiv.id = 'myCheckboxDiv';
        //
        document.body.appendChild(iDiv);
        for (var i = 0; i < buildMapPeriods.length; i++) {
            let drone_id = buildMapPeriods[i]['drone_id'];
            let start_time = convertUTCDateToLocalDate(buildMapPeriods[i]['start_time']);
            let end_time = convertUTCDateToLocalDate(buildMapPeriods[i]['end_time']);

            var checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.name = 'myCustomCheckBox' + i;
            checkbox.value = drone_id + start_time;
            checkbox.id = 'myCustomCheckBox' + i;
            document.body.appendChild(checkbox);
            iDiv.appendChild(checkbox);
            var newLabel = document.createElement('Label');
            newLabel.setAttribute('for', 'myCustomCheckBox' + i);
            newLabel.innerHTML =
                'Drone: ' +
                '<i>' +
                drone_id +
                ' </i>' +
                '&nbsp;&nbsp;Started: ' +
                '<i>' +
                start_time +
                ' </i>' +
                '&nbsp;&nbsp;Ended: ' +
                '<i>' +
                end_time +
                ' </i>';
            iDiv.appendChild(newLabel);
            var br = document.createElement('br');
            iDiv.appendChild(br);
        }

        $('input[id^="myCustomCheckBox"]').checkboxradio();

        return iDiv;
    }

    function createPopupDialogForLoadClearProcess(_sessionDataList, _title) {
        return new Promise((resolve, reject) => {
            let sessionTable = $("<table class='popupSessionTableSelection'>").append(
                "<tr><th>Select</th><th>ID</th><th>Count</th><th>Drone Name</th><th>Start Time</th><th>End Time</th></tr>"
            );
            $.each(_sessionDataList, function (_i, _rowData) {
                sessionTable.append(
                    `<tr><td><input type='checkbox' class='record-select'></td><td>${_rowData.sessionId}</td><td>${_rowData.count}</td><td>${_rowData.droneName}</td><td>${_rowData.startTime}</td><td>${_rowData.endTime}</td></tr>`
                );
            });
            $("<div>")
                .append(sessionTable)
                .dialog({
                    modal: true,
                    title: _title,
                    width: "auto",
                    height: 600,
                    buttons: {
                        Select: function () {
                            let selectedSessionsOnDialog = [];
                            $(".record-select:checked").each(function () {
                                let row = $(this).closest("tr");
                                let record = {
                                    sessionId: row.find("td:eq(1)").text(),
                                    count: row.find("td:eq(2)").text(),
                                    droneName: row.find("td:eq(3)").text(),
                                    startTime: row.find("td:eq(4)").text(),
                                    endTime: row.find("td:eq(5)").text(),
                                };
                                selectedSessionsOnDialog.push(record);
                            });
    
                            $(this).dialog("destroy").remove();
                            resolve(selectedSessionsOnDialog);
                        },
                        Cancel: function () {
                            $(this).dialog("destroy").remove();
                            reject("Cancelled");
                        },
                    },
                });
        });
    }
    
    
    function create_confirmation_dialog(okButton, cancelButton, message, dialogTitle) {
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 400,
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            defer.resolve(true);
                            $(this).remove();
                        },
                    },
                    {
                        text: cancelButton,
                        click: function () {
                            defer.resolve(false);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        return defer.promise();
    }


    function create_confirmation_dialog_with_textfield(okButton, cancelButton, message, dialogTitle, label, subtext) {
        let defer = $.Deferred();
        let inputField = $('<input type="text" size="4" class="form-control">');
        let labelElement = $('<label>' + label + '</label>');
        let helpElement = $('<small class="form-text text-muted">'+subtext+'</small > ');

        

        let dialog = $('<div></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .append(labelElement)
            .append(inputField) // add an input field to the dialog
            .append(helpElement)
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 400,
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            // Resolve the deferred object with the input field value and remove the dialog
                            defer.resolve(inputField.val());
                            $(this).remove();
                        },
                    },
                    {
                        text: cancelButton,
                        click: function () {
                            // Resolve the deferred object with null and remove the dialog
                            defer.resolve(null);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    // Remove the dialog when it is closed
                    $(this).remove();
                },
            });

        dialog.dialog('open');

        return defer.promise();
    }
    
    
    // Safeml Remove button defer.resolve second value
    function create_detection_confirmation_dialog(detTypes, message, title) {
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 600,
                modal: true,
                title: title,
                buttons: [
                    {
                        id: DETECTION_TYPES.VEHICLE_DETECTOR.refName,
                        text: DETECTION_TYPES.VEHICLE_DETECTOR.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.VEHICLE_DETECTOR.name, null, null);
                            $(this).remove();
                        },
                        width: '130',
                    },

                    {
                        id: DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.refName,
                        text: DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.name, document.getElementById('safemlCheckbox').checked, document.getElementById('deepKnowledgeCheckbox').checked);
                            $(this).remove();
                        },
                        width: '130',
                    },
                    {
                        id: DETECTION_TYPES.DISASTER_CLASSIFICATION.refName,
                        text: DETECTION_TYPES.DISASTER_CLASSIFICATION.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.DISASTER_CLASSIFICATION.name, null, null);
                            $(this).remove();
                        },
                        width: '130',
                    },
                    {
                        id: DETECTION_TYPES.CROWD_LOCALIZATION.refName,
                        text: DETECTION_TYPES.CROWD_LOCALIZATION.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.CROWD_LOCALIZATION.name);
                            $(this).remove();
                        },
                        width: '130',
                    },                                
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        return defer.promise();
    }
    function create_confirmation_dialog_with_checkbox(okButton, cancelButton, message, dialogTitle, checkBoxID, checkBoxLabel) {
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 400,
                create: function (e, ui) {
                    var pane = $(this).dialog('widget').find('.ui-dialog-buttonpane');
                    $(`<label ><input id=${checkBoxID}  type='checkbox' style='margin-right: 0px'/> ${checkBoxLabel}</label>`).prependTo(pane);
                },
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            defer.resolve([true, document.querySelector('#' + checkBoxID).checked]);
                            $(this).remove();
                        },
                    },
                    {
                        text: cancelButton,
                        click: function () {
                            defer.resolve([false, document.querySelector('#' + checkBoxID).checked]);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        return defer.promise();
    }













    
    function create_mission_dialog_types(selected_drones, missionPath) {
        let droneNameList = '';
        let droneNameListHtml = '';
        selected_drones.forEach(function (drone) {
            droneNameList = droneNameList + drone.droneID + ',';
            droneNameListHtml = droneNameListHtml + '<div class="selected-drone-for-mission"><i class="fa fa-check"></i> ' + drone.droneID + '</div>';
        });
        let dronePathList = '';
        missionPath.forEach(function (path) {
            dronePathList = dronePathList + path + '/n';
        });

        let normalRadioBtn = 'normalMissionRB';
        let searchAndRescueRadioBtn = 'searchRB';
        let gridRadioBtn = 'mapRB';

        let allCurrentAltitudeRadioBtn = 'currentAltitudeRB';
        let allSameAltitudeRadioBtn = 'sameAltitudeRB';
        let allDifferentAltitudeRadioBtn = 'differentAltitudeRB';
        let customAltitudeRadioBtn = 'customAltitudeRB';

        let allSameAltitudeLabel = 'allSameAltitudeLabel';
        let allSameAltitudeInput = 'allSameAltitudeInput';
        let allDifferentAltitudeInput = 'allDifferentAltitudeInput';
        let customAltitudeInput = 'customAltitudeInput';

        let allSameSpeedLabel = 'allSameSpeedLabel';
        let allSameSpeedInput = 'allSameSpeedInput';
        let allDifferentSpeedInput = 'allDifferentSpeedInput';
        let customSpeedInput = 'customSpeedInput';

        let allSameSpeedRadioBtn = 'allSameSpeedRadioBtn';
        let allDifferentSpeedRadioBtn = 'allDifferentSpeedRadioBtn';
        let customSpeedRadioBtn = 'customSpeedRadioBtn';

        let gimbalManualRadioBtn = 'gimbalManualRadioBtn';
        let gimbalCustomRadioBtn = 'gimbalCustomRadioBtn';
        let customGimbalInput = 'customGimbalInput';

        let repeatMission = 'repeatMissionInput';
        let customRepeatInput = 'customRepeatInput';
        let captureImagesCBid = 'captureCB';

        let dialogTitle = 'Create Mission';
        let nextButton = 'Next';
        let backButton = 'Previous';
        let finishButton = 'Finish';
        let cancelButton = 'Cancel';

        let stepMin = 1;
        let stepMax = 5;

        customHtml = $(`
        <input id="form-step" value="1" type="hidden"></input>
        <input id="form-drone" value=${droneNameList} type="hidden"></input>
        <input id="form-path" value=${dronePathList} type="hidden"></input>
        <div class="selected-mission-drones">
            <div class="mission-popup-title">Participating Drones:</div>
            ${droneNameListHtml}
        </div>
        <form id="missionOptionsForm1" name="missionOptionsForm1" style="display: none;">
            <div>
                <label style="margin-bottom: 0px">
                    <div class="mission-popup-title">Mission Type:</div>
                    <label style="margin-bottom: 0px" ><input type="radio" id="${normalRadioBtn}" name="mission" checked="checked" onclick="radioTypeForm(this)" value="${NORMAL_MISSION}" />
                        Point to Point</label>
                    <label style="margin-bottom: 0px"><input type="radio" id="${searchAndRescueRadioBtn}" name="mission" onclick="radioTypeForm(this)" value="${SEARCH_AND_RESCUE_MISSION}" />
                        Search and Rescue Grid</label>
                </label>
            </div>

            <div>
                <label style="margin-bottom: 0px">
                    <div class="mission-popup-title">Altitude (${DRONE_ALTITUDE_MIN} to ${DRONE_ALTITUDE_MAX} Meters):</div>
                    <label style="margin-bottom: 0px"><input type="radio" id="${allCurrentAltitudeRadioBtn}" name="altitude" checked="checked" value="${ALL_CURRENT_ALTITUDE}" onclick="radioAltitudeForm(this)" />
                        Stay at current altitudes</label>
                    <label style="margin-bottom: 0px; display: none;"><input type="radio" id="${allSameAltitudeRadioBtn}" name="altitude" value="${ALL_SAME_ALTITUDE}" onclick="radioAltitudeForm(this)" />
                        Same altitude for all drones</label>
                    <label style="margin-bottom: 0px"><input type="radio" id="${allDifferentAltitudeRadioBtn}" name="altitude" value="${ALL_DIFFERENT_ALTITUDE}" onclick="radioAltitudeForm(this)" />
                        Custom altitude per drone</label>
                    <label style="margin-bottom: 0px; display: none;"><input type="radio" id="${customAltitudeRadioBtn}" name="altitude" value="${CUSTOM_ALTITUDE}" onclick="radioAltitudeForm(this)" />
                        Custom altitude per drone per waypoint
                    </label>
                    <label style="margin-bottom: 0px" id="${allSameAltitudeLabel}" hidden >
                        <input id="${allSameAltitudeInput}" type="number" placeholder="Altitude" />
                    </label>
                    <label style="margin-bottom: 0px" id="${allDifferentAltitudeInput}" hidden></label>
                    <label style="margin-bottom: 0px" id="${customAltitudeInput}" hidden></label>
                    </label>
                    </div>
                </label>
            </div>

            <div>
                <label style="margin-bottom: 0px">
                    <div class="mission-popup-title">Speed (${DRONE_SPEED_MIN} to ${DRONE_SPEED_MAX} m/s):</div>
                    <label style="margin-bottom: 0px"><input type="radio" id="${allSameSpeedRadioBtn}" name="speed" checked="checked" value="${ALL_SAME_SPEED}" onclick="radioSpeedForm(this)" />
                    Same speed for all drones
                    </label>
                    <label style="margin-bottom: 0px"><input type="radio" id="${allDifferentSpeedRadioBtn}" name="speed" value="${ALL_DIFFERENT_SPEED}" onclick="radioSpeedForm(this)" />
                        Custom speed per drone
                    </label>
                    <label style="margin-bottom: 0px; display: none;"><input type="radio" id="${customSpeedRadioBtn}" name="speed" value="${CUSTOM_SPEED}" onclick="radioSpeedForm(this)" />
                        Custom speed per drone per waypoint
                    </label>
                    <label style="margin-bottom: 0px" id="${allSameSpeedLabel}" >
                        <input id="${allSameSpeedInput}" type="number" placeholder="Speed" value='5'/>
                    </label>
                    <label style="margin-bottom: 0px" id="${allDifferentSpeedInput}" hidden ></label>
                    <label style="margin-bottom: 0px" id="${customSpeedInput}" hidden ></label>
                </label>
            </div>

            <div>
                <label style="margin-bottom: 0px">
                    <div class="mission-popup-title">Gimbal Angle (-90 to 0 Degrees):</div>
                    <label style="margin-bottom: 0px"><input type="radio" id="${gimbalManualRadioBtn}" name="gimbal" checked="checked" value="${MANUAL_GIMBAL}" onclick="radioGimbalForm(this)" />
                       Manual control
                    </label>
                    <label style="margin-bottom: 0px"><input type="radio" id="${gimbalCustomRadioBtn}" name="gimbal" value="${CUSTOM_GIMBAL}" onclick="radioGimbalForm(this)" />
                        Set angle
                    </label>
                    <label style="margin-bottom: 0px" id="${customGimbalInput}" hidden ></label>
                </label>
            </div>

        <div>
            <label style="margin-bottom: 0px">
                <div class="mission-popup-title">Repeat Mission:</div>
                <label style="margin-bottom: 0px" id="${customRepeatInput}" >
                <input id="${repeatMission}" type="number" value="1" min="1" />
                </label>
            </label>

            <div class="mission-popup-title">Extra Features:</div>
            <label style="margin-bottom: 0px"><input id="${captureImagesCBid}" type="checkbox" value="false" /> 
            Capture and store images on drone's memory
            </label>
                
        </div>
    </form>
    
    <p class="errorMessageMissionForm"  style="color:red; margin:0; margin-top: 10px; border: 2px solid red; padding: 3px; display: none;"> </p>
        <script>
            document.getElementById("missionOptionsForm"+document.getElementById("form-step").value).style.display = "block";
            typeSelected="None";
            // Function to handle the mission type form
            function radioTypeForm(element) {
                if(element.id === ${normalRadioBtn}.id && element.id !== typeSelected.id){
                    // ${customAltitudeRadioBtn}.parentNode.style.display = "";
                    // ${customSpeedRadioBtn}.parentNode.style.display = "";
                    ${gimbalCustomRadioBtn}.parentNode.style.display = "";
                } else if(element.id === ${searchAndRescueRadioBtn}.id && element.id !== typeSelected.id){
                    ${customAltitudeRadioBtn}.parentNode.style.display = "none";
                    ${customSpeedRadioBtn}.parentNode.style.display = "none";
                    ${gimbalCustomRadioBtn}.parentNode.style.display = "none";
                    radioAltitudeForm(${allCurrentAltitudeRadioBtn})
                    ${allCurrentAltitudeRadioBtn}.checked=true;
                    radioSpeedForm(${allSameSpeedRadioBtn})
                    ${allSameSpeedRadioBtn}.checked=true;
                    radioGimbalForm(${gimbalManualRadioBtn})
                    ${gimbalManualRadioBtn}.checked=true;
                } else if(element.id === ${gridRadioBtn}.id && element.id !== typeSelected.id){
                    ${customAltitudeRadioBtn}.parentNode.style.display = "none";
                    ${customSpeedRadioBtn}.parentNode.style.display = "none";
                    ${gimbalCustomRadioBtn}.parentNode.style.display = "none";
                    radioAltitudeForm(${allCurrentAltitudeRadioBtn})
                    ${allCurrentAltitudeRadioBtn}.checked=true;
                    radioSpeedForm(${allSameSpeedRadioBtn})
                    ${allSameSpeedRadioBtn}.checked=true;
                    radioGimbalForm(${gimbalManualRadioBtn})
                    ${gimbalManualRadioBtn}.checked=true;
                }
                typeSelected=element
            }
            altitudeSelected="None";
            // Function to handle the altitude form
            function radioAltitudeForm(element) {
                if(element.id === ${allCurrentAltitudeRadioBtn}.id && element.id !== altitudeSelected.id){
                    ${allSameAltitudeLabel}.hidden = true;
                    ${allDifferentAltitudeInput}.hidden = true;
                    ${customAltitudeInput}.hidden = true;
                }else if(element.id === ${allSameAltitudeRadioBtn}.id && element.id !== altitudeSelected.id){
                    ${allSameAltitudeLabel}.hidden = false;
                    ${allDifferentAltitudeInput}.hidden = true;
                    ${customAltitudeInput}.hidden = true;
                }else if(element.id === ${allDifferentAltitudeRadioBtn}.id && element.id !== altitudeSelected.id){
                    ${allSameAltitudeLabel}.hidden = true;
                    ${allDifferentAltitudeInput}.hidden = false;
                    ${customAltitudeInput}.hidden = true;
                }else if(element.id === ${customAltitudeRadioBtn}.id && element.id !== altitudeSelected.id){
                    ${allSameAltitudeLabel}.hidden = true;
                    ${allDifferentAltitudeInput}.hidden = true;
                    ${customAltitudeInput}.hidden = false;
                }
                altitudeSelected=element
            }
            // Function to handle the speed form
            speedSelected="None";
            function radioSpeedForm(element) {
                if(element.id === ${allSameSpeedRadioBtn}.id && element.id !== speedSelected.id){
                    ${allSameSpeedLabel}.hidden = false;
                    ${allDifferentSpeedInput}.hidden = true;
                    ${customSpeedInput}.hidden = true;
                }else if(element.id === ${allDifferentSpeedRadioBtn}.id && element.id !== speedSelected.id){
                    ${allSameSpeedLabel}.hidden = true;
                    ${allDifferentSpeedInput}.hidden = false;
                    ${customSpeedInput}.hidden = true;
                }else if(element.id === ${customSpeedRadioBtn}.id && element.id !== speedSelected.id){
                    ${allSameSpeedLabel}.hidden = true;
                    ${allDifferentSpeedInput}.hidden = true;
                    ${customSpeedInput}.hidden = false;
                }
            }

            // Function to handle the gimbal form
            gimbalSelected="None";
            function radioGimbalForm(element) {
                if(element.id === ${gimbalManualRadioBtn}.id && element.id !== gimbalSelected.id){
                    ${customGimbalInput}.hidden = true;
                }else if(element.id === ${gimbalCustomRadioBtn}.id && element.id !== gimbalSelected.id){
                    ${customGimbalInput}.hidden = false;
                }
            }

            listOfDrones=document.getElementById('form-drone').value.split(',').filter(entry => entry.trim() != '')
            listOfPaths=document.getElementById('form-path').value.split('/n').filter(entry => entry.trim() != '')

            // function for adding the different drone names
            listOfDrones.forEach(function (drone) {
                inputAlt = document.createElement('input');
                inputAlt.id =${allDifferentAltitudeInput}.id+drone
                inputAlt.type = 'number';
                inputAlt.placeholder ="Altitude";
                inputSpeed = document.createElement('input');
                inputSpeed.id = ${allDifferentSpeedInput}.id+drone
                inputSpeed.type = 'number';
                inputSpeed.placeholder ="Speed";
                label = document.createElement('label');
                label.style.margin = "0px 0px 0px 15px";
                label.style.width = '100px';
                label.style.fontWeight = 'bold';
                label.style.display = 'inline';
                label.innerHTML = drone+': ';

                ${allDifferentAltitudeInput}.appendChild(label);
                ${allDifferentAltitudeInput}.appendChild(inputAlt);
                ${allDifferentAltitudeInput}.innerHTML = ${allDifferentAltitudeInput}.innerHTML+'<br>';

                ${allDifferentSpeedInput}.appendChild(label);
                ${allDifferentSpeedInput}.appendChild(inputSpeed);
                ${allDifferentSpeedInput}.innerHTML = ${allDifferentSpeedInput}.innerHTML+'<br>';
            })
            
            // Custom mission inputs
            function tableCustom(element, listOfDrones, listOfPaths, placeholders , min, max){
                table=document.createElement('table');
                table.style.textAlign = 'center';
                table.border='1'
                table.style.borderSpacing= '5px';
                table.classList.add("table");
                table.classList.add("table-striped");
                tr=document.createElement('tr')
                th=document.createElement('th')
                th.innerHTML="Waypoints"
                tr.appendChild(th)
                listOfDrones.forEach(function (drone) {
                    th=document.createElement('th')
                    th.innerHTML=drone
                    tr.appendChild(th)
                })
                table.appendChild(tr)
                listOfPaths.forEach(function (path, index) {
                    tr=document.createElement('tr')
                    td=document.createElement('td')
                    td.style.paddingLeft='5px'
                    td.style.paddingRight='5px'
                    td.innerHTML=index+1
                    tr.appendChild(td)
                    listOfDrones.forEach(function (drone) {
                        input=document.createElement('input')
                        input.id=placeholders+index+drone
                        input.type="number"
                        input.placeholder=placeholders
                        input.style.width="100%"
                        td=document.createElement('td')
                        td.appendChild(input)
                        tr.appendChild(td)
                    })
                    table.appendChild(tr)
                })
                element.appendChild(table)
            }

            tableCustom(${customAltitudeInput}, listOfDrones, listOfPaths, 'Altitude', DRONE_ALTITUDE_MIN, DRONE_ALTITUDE_MAX)
            tableCustom(${customSpeedInput}, listOfDrones, listOfPaths, 'Speed', DRONE_SPEED_MIN, DRONE_SPEED_MAX)
            tableCustom(${customGimbalInput}, listOfDrones, listOfPaths, 'Angle', DRONE_GIMBAL_ANGLE_MIN, DRONE_GIMBAL_ANGLE_MAX)

            // Extra details
            // ${allDifferentAltitudeInput}.innerHTML = ${allDifferentAltitudeInput}.innerHTML+"<label style='margin-bottom:0px'>The drone's altitude ranges from ${DRONE_ALTITUDE_MIN} and ${DRONE_ALTITUDE_MAX}.</label>";
            // ${customAltitudeInput}.innerHTML = ${customAltitudeInput}.innerHTML+"<label style='margin-bottom:0px'>Altitude ranges from ${DRONE_ALTITUDE_MIN} and ${DRONE_ALTITUDE_MAX}.</label>";
            // ${allDifferentSpeedInput}.innerHTML = ${allDifferentSpeedInput}.innerHTML+"<label style='margin-bottom:0px'>Speed ranges from ${DRONE_SPEED_MIN} and ${DRONE_SPEED_MAX}.</label>";
            // ${customSpeedInput}.innerHTML = ${customSpeedInput}.innerHTML+"<label style='margin-bottom:0px'>The drone's speed ranges from ${DRONE_SPEED_MIN} and ${DRONE_SPEED_MAX}.</label>";
            // ${customGimbalInput}.innerHTML = ${customGimbalInput}.innerHTML+"<label style='margin-bottom:0px'>The gimbal's angle ranges from ${DRONE_GIMBAL_ANGLE_MIN} and ${DRONE_GIMBAL_ANGLE_MAX}.</label>";
            

            ${captureImagesCBid}.addEventListener('change', function() {
                if ($(this).is(':checked')) {
                  $(this).attr('value', 'true');
                } else {
                  $(this).attr('value', 'false');
                }
            })

            </script>
        `);
        // List of the response
        response_list = [];

        function changeDialogButtons(currentStep) {
            currentStep = parseInt(currentStep);
            for (let index = 0; index < document.getElementsByClassName('ui-button ui-corner-all ui-widget').length; index++) {
                const buttonElement = document.getElementsByClassName('ui-button ui-corner-all ui-widget')[index];
                if (buttonElement.innerHTML == nextButton) {
                    //Next button
                    buttonElement.style.display = 'none';
                    if (stepMin <= currentStep && currentStep < stepMax) {
                        buttonElement.style.display = 'unset';
                    }
                } else if (buttonElement.innerHTML == backButton) {
                    //Previous button
                    buttonElement.style.display = 'none';
                    if (stepMin < currentStep && currentStep <= stepMax) {
                        buttonElement.style.display = 'unset';
                    }
                } else if (buttonElement.innerHTML == finishButton) {
                    //Finnish button
                    buttonElement.style.display = 'none';
                    if (currentStep == 1) {
                        buttonElement.style.display = 'unset';
                    }
                }
            }
        }

        /* VALIDATION */

        function validateMissionOptionsForm5() {
            response_list[1] = { [document.querySelector('input[name="mission"]:checked').value]: '' };

            if (document.querySelector('input[name="altitude"]:checked').value == ALL_CURRENT_ALTITUDE) {
                response_list[2] = { [ALL_CURRENT_ALTITUDE]: '' };
            } else if (document.querySelector('input[name="altitude"]:checked').value == ALL_SAME_ALTITUDE) {
                if (
                    document.forms['missionOptionsForm1'][allSameAltitudeInput].value === '' ||
                    document.forms['missionOptionsForm1'][allSameAltitudeInput].value === null ||
                    document.forms['missionOptionsForm1'][allSameAltitudeInput].value < DRONE_ALTITUDE_MIN ||
                    document.forms['missionOptionsForm1'][allSameAltitudeInput].value > DRONE_ALTITUDE_MAX
                ) {
                    document.forms['missionOptionsForm1'][allSameAltitudeInput].style.border = '2px solid red';
                    document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                    document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                        'Invalid altitude value.<br />Please enter a value between ' + DRONE_ALTITUDE_MIN + ' and ' + DRONE_ALTITUDE_MAX + '.';
                    return false;
                }

                document.forms['missionOptionsForm1'][allSameAltitudeInput].style.border = '';
                response_list[2] = { [ALL_SAME_ALTITUDE]: document.forms['missionOptionsForm1'][allSameAltitudeInput].value };
            } else if (document.querySelector('input[name="altitude"]:checked').value == ALL_DIFFERENT_ALTITUDE) {
                droneAlts = [];
                rawAltitudes = [];
                for (let index = 0; index < droneNameList.split(',').filter((entry) => entry.trim() != '').length; index++) {
                    const drone = droneNameList.split(',').filter((entry) => entry.trim() != '')[index];
                    if (
                        document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value == '' ||
                        document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value == null ||
                        document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value < DRONE_ALTITUDE_MIN ||
                        document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value > DRONE_ALTITUDE_MAX
                    ) {
                        document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].style.border = '2px solid red';
                        document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                        document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                            'Invalid altitude value.<br />Please enter a value between ' + DRONE_ALTITUDE_MIN + ' and ' + DRONE_ALTITUDE_MAX + '.';
                        return false;
                    }

                    document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].style.border = '';
                    droneAlts.push({ [drone]: document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value });
                    rawAltitudes.push(document.forms['missionOptionsForm1'][allDifferentAltitudeInput + drone].value)
                }
                
                // check vertical separation
                if(hasDifferenceWithinRange(rawAltitudes, altitudeSafetyDistance-1)) {
                    document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                    document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                        'Drone altitudes are too close.<br />Please keep a vertical separation of at least ' + altitudeSafetyDistance + 'm.';
                    return false;
                }


                response_list[2] = { [ALL_DIFFERENT_ALTITUDE]: droneAlts };
            } else if (document.querySelector('input[name="altitude"]:checked').value == CUSTOM_ALTITUDE) {
                droneAlts = [];
                for (let index = 0; index < droneNameList.split(',').filter((entry) => entry.trim() != '').length; index++) {
                    const drone = droneNameList.split(',').filter((entry) => entry.trim() != '')[index];
                    dronesCustomAltsList = [];
                    droneAltsCurrent = '';
                    for (let altIndex = 0; altIndex < dronePathList.split(',').filter((entry) => entry.trim() != '').length - 1; altIndex++) {
                        if (
                            document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value === '' ||
                            document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value === null
                        ) {
                            dronesCustomAltsList.push(droneAltsCurrent);
                        } else {
                            if (
                                document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value < DRONE_ALTITUDE_MIN ||
                                document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value > DRONE_ALTITUDE_MAX
                            ) {
                                document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].style.border = '2px solid red';
                                document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                                document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                                    'Invalid altitude value.<br />Please enter a value between ' + DRONE_ALTITUDE_MIN + ' and ' + DRONE_ALTITUDE_MAX + '.';
                                return false;
                            }
                            dronesCustomAltsList.push(document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value);
                            droneAltsCurrent = document.forms['missionOptionsForm1']['Altitude' + altIndex + drone].value;
                        }
                    }
                    droneAlts.push(dronesCustomAltsList);
                }
                response_list[2] = { [CUSTOM_ALTITUDE]: droneAlts };
            }
            // document.getElementsByClassName('errorMessageMissionForm')[0].style.visibility = 'hidden';

            if (document.querySelector('input[name="speed"]:checked').value == ALL_SAME_SPEED) {
                if (
                    document.forms['missionOptionsForm1'][allSameSpeedInput].value === '' ||
                    document.forms['missionOptionsForm1'][allSameSpeedInput].value === null ||
                    document.forms['missionOptionsForm1'][allSameSpeedInput].value < DRONE_SPEED_MIN ||
                    document.forms['missionOptionsForm1'][allSameSpeedInput].value > DRONE_SPEED_MAX
                ) {
                    document.forms['missionOptionsForm1'][allSameSpeedInput].style.border = '2px solid red';
                    document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                    document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                        'Invalid speed value.<br />Please enter a value between ' + DRONE_SPEED_MIN + ' and ' + DRONE_SPEED_MAX + '.';
                    return false;
                }
                document.forms['missionOptionsForm1'][allSameSpeedInput].style.border = '';
                response_list[3] = { [ALL_SAME_SPEED]: document.forms['missionOptionsForm1'][allSameSpeedInput].value };
            } else if (document.querySelector('input[name="speed"]:checked').value == ALL_DIFFERENT_SPEED) {
                droneSpeeds = [];
                for (let index = 0; index < droneNameList.split(',').filter((entry) => entry.trim() != '').length; index++) {
                    const drone = droneNameList.split(',').filter((entry) => entry.trim() != '')[index];
                    if (
                        document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].value == '' ||
                        document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].value == null ||
                        document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].value < DRONE_SPEED_MIN ||
                        document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].value > DRONE_SPEED_MAX
                    ) {
                        document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].style.border = '2px solid red';
                        document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                        document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                            'Invalid speed value.<br />Please enter a value between ' + DRONE_SPEED_MIN + ' and ' + DRONE_SPEED_MAX + '.';
                        return false;
                    }
                    document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].style.border = '';
                    droneSpeeds.push(document.forms['missionOptionsForm1'][allDifferentSpeedInput + drone].value);
                }
                response_list[3] = { [ALL_DIFFERENT_SPEED]: droneSpeeds };
            } else if (document.querySelector('input[name="speed"]:checked').value == CUSTOM_SPEED) {
                droneSpeeds = [];
                for (let index = 0; index < droneNameList.split(',').filter((entry) => entry.trim() != '').length; index++) {
                    const drone = droneNameList.split(',').filter((entry) => entry.trim() != '')[index];
                    dronesCustomSpeedsList = [];
                    droneSpeedsCurrent = '';
                    if (
                        document.forms['missionOptionsForm1']['Speed0' + drone].value == '' ||
                        document.forms['missionOptionsForm1']['Speed0' + drone].value == null ||
                        document.forms['missionOptionsForm1']['Speed0' + drone].value < DRONE_SPEED_MIN ||
                        document.forms['missionOptionsForm1']['Speed0' + drone].value > DRONE_SPEED_MAX
                    ) {
                        document.forms['missionOptionsForm1']['Speed0' + drone].style.border = '2px solid red';
                        document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                        document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                            'Invalid speed value.<br />Please enter a value between ' + DRONE_SPEED_MIN + ' and ' + DRONE_SPEED_MAX + '.';
                        return false;
                    }
                    document.forms['missionOptionsForm1']['Speed0' + drone].style.border = '';
                    for (let speedIndex = 0; speedIndex < dronePathList.split(',').filter((entry) => entry.trim() != '').length - 1; speedIndex++) {
                        if (
                            document.forms['missionOptionsForm1']['Speed' + speedIndex + drone].value === '' ||
                            document.forms['missionOptionsForm1']['Speed' + speedIndex + drone].value === null
                        ) {
                            dronesCustomSpeedsList.push(droneSpeedsCurrent);
                        } else {
                            dronesCustomSpeedsList.push(document.forms['missionOptionsForm1']['Speed' + speedIndex + drone].value);
                            droneSpeedsCurrent = document.forms['missionOptionsForm1']['Speed' + speedIndex + drone].value;
                        }
                    }
                    droneSpeeds.push(dronesCustomSpeedsList);
                }
                response_list[3] = { [CUSTOM_SPEED]: droneSpeeds };
                // document.getElementsByClassName('errorMessageMissionForm')[0].style.visibility = 'hidden';
            }
            // document.getElementsByClassName('errorMessageMissionForm')[0].style.visibility = 'hidden';

            if (document.querySelector('input[name="gimbal"]:checked').value == MANUAL_GIMBAL) {
                response_list[4] = { [MANUAL_GIMBAL]: '' };
            } else if (document.querySelector('input[name="gimbal"]:checked').value == CUSTOM_GIMBAL) {
                droneGimbal = [];
                for (let index = 0; index < droneNameList.split(',').filter((entry) => entry.trim() != '').length; index++) {
                    const drone = droneNameList.split(',').filter((entry) => entry.trim() != '')[index];
                    dronesCustomGimbalList = [];
                    for (let speedIndex = 0; speedIndex < dronePathList.split(',').filter((entry) => entry.trim() != '').length - 1; speedIndex++) {
                        if (
                            document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].value < DRONE_GIMBAL_ANGLE_MIN ||
                            document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].value > DRONE_GIMBAL_ANGLE_MAX
                        ) {
                            document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].style.border = '2px solid red';
                            document.getElementsByClassName('errorMessageMissionForm')[0].style.display = "block";
                            document.getElementsByClassName('errorMessageMissionForm')[0].innerHTML =
                                'Invalid gimbal angle value.<br />Please enter a value between ' + DRONE_GIMBAL_ANGLE_MIN + ' and ' + DRONE_GIMBAL_ANGLE_MAX + '.';
                            return false;
                        }
                        if (
                            document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].value === '' ||
                            document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].value === null
                        ) {
                            dronesCustomGimbalList.push('');
                        } else {
                            dronesCustomGimbalList.push(document.forms['missionOptionsForm1']['Angle' + speedIndex + drone].value);
                        }
                    }
                    droneGimbal.push(dronesCustomGimbalList);
                }
                response_list[4] = { [CUSTOM_GIMBAL]: droneGimbal };
            }

            response_list[5] = { repeatMission: document.forms['missionOptionsForm1'][repeatMission].value };
            response_list[6] = { images: document.forms['missionOptionsForm1'][captureImagesCBid].value };
            return true;
        }






        let defer = $.Deferred();
        let dialog = $('<div class="mission-popup-inner"></div>')
            .appendTo('body')
            .html(customHtml)
            .dialog({
                autoOpen: false,
                height: 'auto',
                maxHeight: 800,
                width: 'fit-content',
                position: { my: "left top", at: "left+310 top+10", of: window },
                dialogClass: 'mission-popup',
                create: function (e, ui) {
                    // var pane = $(this).dialog("widget").find(".ui-dialog-buttonpane")
                    // customHtml.prependTo(pane)
                },
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: backButton,
                        click: function () {
                            if (document.getElementById('form-step').value > stepMin) {
                                document.getElementById('missionOptionsForm' + document.getElementById('form-step').value).style.display = 'none';
                                document.getElementById('form-step').value = (parseInt(document.getElementById('form-step').value) - 1).toString();
                                document.getElementById('missionOptionsForm' + document.getElementById('form-step').value).style.display = 'block';
                                changeDialogButtons(document.getElementById('form-step').value);
                            }
                        },
                    },
                    // {
                    //     text: nextButton,
                    //     click: function () {
                    //         validationChecker = true;
                    //         if (parseInt(document.getElementById('form-step').value) == 1) {
                    //             validationChecker = validateMissionOptionsForm1();
                    //         } else if (parseInt(document.getElementById('form-step').value) == 2) {
                    //             validationChecker = validateMissionOptionsForm2();
                    //         } else if (parseInt(document.getElementById('form-step').value) == 3) {
                    //             validationChecker = validateMissionOptionsForm3();
                    //         } else if (parseInt(document.getElementById('form-step').value) == 4) {
                    //             validationChecker = validateMissionOptionsForm4();
                    //         }
                    //         if (document.getElementById('form-step').value < stepMax && validationChecker) {
                    //             document.getElementById('missionOptionsForm' + document.getElementById('form-step').value).style.display = 'none';
                    //             document.getElementById('form-step').value = (parseInt(document.getElementById('form-step').value) + 1).toString();
                    //             document.getElementById('missionOptionsForm' + document.getElementById('form-step').value).style.display = 'block';
                    //             changeDialogButtons(document.getElementById('form-step').value);
                    //         }
                    //     },
                    // },
                    {
                        text: finishButton,
                        click: function () {
                            validationChecker = validateMissionOptionsForm5();
                            if (validationChecker) {
                                response_list[0] = true;
                                defer.resolve(response_list);
                                // console.log("------------------------------------------------------------------");
                                // console.log(response_list);
                                // console.log("------------------------------------------------------------------");
                                $(this).remove();
                            }
                        },
                    },
                    {
                        text: cancelButton,
                        click: function () {
                            defer.resolve([false]);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        changeDialogButtons(stepMin);
        dialog.dialog('open');
        return defer.promise();
    }














    function create_dialog_with_one_button(okButton, message, dialogTitle, width, height, div_id = 'temp_warning_dialog') {
        let defer = $.Deferred();

        let dialog = $('<div id=' + div_id + '></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: height,
                width: width,
                modal: true,
                resizable: false,
                title: dialogTitle,

                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            defer.resolve(true);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        return defer.promise();
    }

    function create_dialog_with_one_button_and_checkbox(okButton, message, dialogTitle, width, height, checkboxText, div_id = 'temp_warning_dialog') {
        let defer = $.Deferred();

        let dialog = $('<div id=' + div_id + '></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: height,
                width: width,
                modal: true,
                resizable: false,
                title: dialogTitle,
                create: function (e, ui) {
                    var pane = $(this).dialog('widget').find('.ui-dialog-buttonpane');
                    $(`<label class='shut-up' ><input  type='checkbox'/> ${checkboxText} </label>`).prependTo(pane);
                },
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            let checkboxValue = document.querySelector('.shut-up input').checked;
                            defer.resolve([true, checkboxValue]);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        // $(document).on("change", ".shut-up input", function () {
        //     alert("shut up! " + this.checked)
        // })

        return defer.promise();
    }

    function fnOpenNormalDialog() {
        // Define the Dialog and its properties.
        $('#dialog-confirm').dialog({
            resizable: false,
            modal: true,
            title: 'Modal',
            height: 250,
            width: 400,
            create: function (e, ui) {
                var pane = $(this).dialog('widget').find('.ui-dialog-buttonpane');
                $("<label class='shut-up' ><input  type='checkbox'/> Stop asking!</label>").prependTo(pane);
            },
            buttons: {
                Yes: function () {
                    $(this).dialog('close');
                    callback(true);
                },
                No: function () {
                    $(this).dialog('close');
                    callback(false);
                },
            },
        });
    }

    function create_operation_join_dialog(okButton, cancelButton, message, dialogTitle) {
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html(
                `
            ${message}
             <form>
              <label for="operationName">Operation Name:</label>
              <input type="text" id="opName" name="operationName"><br>
            </form> 

            `
            )
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 400,
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            let operationName = document.getElementById('opName').value;
                            defer.resolve([true, operationName]);
                            $(this).remove();
                        },
                    },
                    {
                        text: cancelButton,
                        click: function () {
                            defer.resolve([false]);
                            $(this).remove();
                        },
                    },
                ],
                close: function () {
                    $(this).remove();
                },
            });
        dialog.dialog('open');
        return defer.promise();
    }
    function create_popup_for_a_little(type, msg, duration_ms) {
        let div_id = 'temp_popup';
        try {
            document.getElementById(div_id).remove(); // remove already existing temp popups
        } catch (error) {
            
        }

        let myDiv = jQuery('<div>', {
            id: div_id,
            class: `center-element alert ${type} fade in alert-dismissible show`,
            css: {
                width: 'fit-content',
                height: 'fit-content',
            },
            text: msg,
        }).appendTo('body');
        let btn = $(
            '<button type="button" class="close" data-dismiss="alert" aria-label="Close">\n' +
                '    <span aria-hidden="true">&times;</span>\n' +
                '  </button>'
        );
        myDiv.append(btn);
        setTimeout(function () {
            // $(popup_element).hide()
            $(myDiv).fadeOut();
            $(myDiv).remove();
        }, duration_ms);
        return div_id;
    }

    function create_popup(type, msg, popup_id) {
        let myDiv = jQuery('<div>', {
            id: popup_id,
            class: `center-element alert ${type} fade in alert-dismissible show`,
            css: {
                width: 'fit-content',
                height: 'fit-content',
            },
            text: msg,
        }).appendTo('body');
        return myDiv;
    }

    function hideParentEl(el) {
        $(el).parent().hide();
        map.off('click', placeMarkerOnClick);
    }

    function removeEl(id) {
        $(id).remove();
    }
}

function create_operation_dialog(okButton, cancelButton, selected, message, dialogTitle) {
    html_string = `<div>Operation: <select  name="operation" id="operation">`;
    html_string = html_string + '<option value= "None">' + 'None' + '</option>';
    message.forEach(function (operation) {
        if (selected == operation) {
            select = ' selected';
        } else {
            select = ' ';
        }
        html_string = html_string + '<option value="' + operation + '"' + select + '>' + operation + '</option>';
    });
    html_string = html_string + `</select></div>`;

    let defer = $.Deferred();
    let dialog = $('<div></div>')
        .appendTo('body')
        .html(html_string)
        .dialog({
            autoOpen: false,
            height: 'auto',
            width: 300,
            modal: true,
            title: dialogTitle,
            buttons: [
                {
                    text: okButton,
                    click: function () {
                        var selected_operation = document.getElementById('operation');
                        defer.resolve(selected_operation.value);
                        $(this).remove();
                    },
                },
                {
                    text: cancelButton,
                    click: function () {
                        defer.resolve(false);
                        $(this).remove();
                    },
                },
            ],
            close: function () {
                $(this).remove();
            },
        });
    dialog.dialog('open');
    return defer.promise();
}

function create_form_with_checkboxes_for_3d_mesh(mesh_periods) {
    let iDiv = document.createElement('div');
    iDiv.id = 'myCheckboxDiv';
    //
    document.body.appendChild(iDiv);
    for (var i = 0; i < mesh_periods.length; i++) {
        let drone_id = mesh_periods[i]['id'];
        let start_time = convertUTCDateToLocalDate(mesh_periods[i]['start_time']);

        let end_time = convertUTCDateToLocalDate(mesh_periods[i]['end_time']);
        var checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.name = 'myCustomCheckBox' + i;
        checkbox.value = drone_id + start_time;
        checkbox.id = 'myCustomCheckBox' + i;
        document.body.appendChild(checkbox);
        iDiv.appendChild(checkbox);
        var newLabel = document.createElement('Label');
        newLabel.setAttribute('for', 'myCustomCheckBox' + i);
        newLabel.innerHTML =
            'ID: ' +
            '<i>' +
            drone_id +
            ' </i>' +
            '&nbsp;&nbsp;Started: ' +
            '<i>' +
            start_time +
            ' </i>' +
            '&nbsp;&nbsp;Ended: ' +
            '<i>' +
            end_time +
            ' </i>';
        iDiv.appendChild(newLabel);
        var br = document.createElement('br');
        iDiv.appendChild(br);
    }

    $('input[id^="myCustomCheckBox"]').checkboxradio();

    return iDiv;
}

function create_3d_mesh_dialog_with_checkboxes_form(checkboxes_form_element, okButton, cancelButton, dialogTitle) {
    let defer = $.Deferred();
    let dialog = $(checkboxes_form_element).dialog({
        autoOpen: false,
        height: 400,
        width: 660,
        modal: true,
        title: dialogTitle,
        buttons: [
            {
                text: okButton,
                click: function () {
                    // defer.resolve(true);
                    defer.resolve([true, dialog]);
                    // $(this).dialog("close");
                },
            },
            {
                text: cancelButton,
                click: function () {
                    // defer.resolve(false);
                    defer.resolve([false, dialog]);
                    $(this).remove();
                },
            },
        ],
        close: function () {
            defer.resolve([false, dialog]);
            $(this).remove();
        },
    });
    dialog.dialog('open');
    return defer.promise();
}
