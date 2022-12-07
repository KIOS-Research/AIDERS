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
                    let msg =
                        'Something went wrong! Build Map failed to start. Trying to start Build Map again...(' +
                        retryNumber +
                        ')';
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

    function hightlightElement(element, action) {
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

    function create_form_with_checkboxes_for_load_build_map(buildmap_periods) {
        let iDiv = document.createElement('div');
        iDiv.id = 'myCheckboxDiv';
        //
        document.body.appendChild(iDiv);
        for (var i = 0; i < buildmap_periods.length; i++) {
            let drone_id = buildmap_periods[i]['drone_id'];
            let start_time = convertUTCDateToLocalDate(buildmap_periods[i]['start_time']);
            console.log(buildmap_periods[i]['end_time']);
            let end_time = convertUTCDateToLocalDate(buildmap_periods[i]['end_time']);

            var checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.name = 'myCustomCheckBox' + i;
            checkbox.value = drone_id + start_time;
            checkbox.id = 'myCustomCheckBox' + i;
            document.body.appendChild(checkbox);
            iDiv.appendChild(checkbox);
            var newlabel = document.createElement('Label');
            newlabel.setAttribute('for', 'myCustomCheckBox' + i);
            newlabel.innerHTML =
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
            iDiv.appendChild(newlabel);
            var br = document.createElement('br');
            iDiv.appendChild(br);
        }

        $('input[id^="myCustomCheckBox"]').checkboxradio();

        return iDiv;
    }

    /*
     * Creates a custom pop up dialog.
     * It returns true once it is created, and also returns the dialog itself
     * */
    function create_build_map_dialog_with_checkboxes_form(
        checkboxes_form_element,
        okButton,
        cancelButton,
        dialogTitle
    ) {
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

    function create_detection_confirmation_dialog(detTypes, message, title) {
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html('<div><h6>' + message + '</h6></div>')
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 550,
                modal: true,
                title: title,
                buttons: [
                    {
                        id: DETECTION_TYPES.VEHICLE_DETECTOR.refName,
                        text: DETECTION_TYPES.VEHICLE_DETECTOR.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.VEHICLE_DETECTOR.name);
                            $(this).remove();
                        },
                        width: '100',
                    },

                    {
                        id: DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.refName,
                        text: DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.refName,
                        click: function () {
                            defer.resolve(DETECTION_TYPES.VEHICLE_PERSON_DETECTOR.name);
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
    function create_confirmation_dialog_with_checkbox(
        okButton,
        cancelButton,
        message,
        dialogTitle,
        checkBoxID,
        checkBoxLabel
    ) {
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
                    $(
                        `<label ><input id=${checkBoxID}  type='checkbox' style='margin-right: 0px'/> ${checkBoxLabel}</label>`
                    ).prependTo(pane);
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

    function create_mission_confirmation_dialog() {
        let message = `You are about to start a mission`;
        let radioBtnSearchID = 'searchRB';
        let searchLabel = 'Search and Rescue Grid Mission';

        let radioBtnMappingID = 'mapRB';
        let mappingLabel = 'Map Grid Mission';

        let captureImagesCBid = 'captureCB';
        let altitude = 'altitude';
        let captureImagesLabel = "Capture images and store them on drone's memory";
        let dialogTitle = 'Choose Mission Type';
        let okButton = 'GO';
        let cancelButton = 'Cancel';

        let normalRadioBtn = 'normalMissionRB';
        let normalRadioLabel = 'Normal Mission';

        let customHtml = $(`<form>
            <div>
                Type:
                <br>
                <label ><input type="radio" id=${normalRadioBtn} name="mission" value=${NORMAL_MISSION} style='margin-right: 0'> ${normalRadioLabel}</label> 
                <br>
                <label> <input type="radio" id=${radioBtnSearchID} name="mission" value=${SEARCH_AND_RESCUE_MISSION}  style='margin-right: 0'>${searchLabel}</label>
                <br>
                <label><input type="radio" id=${radioBtnMappingID} name="mission" value=${MAP_MISSION}  style='margin-right: 0'>${mappingLabel}</label>
                <br>
                <br>
                Drone altitude:<label><input type="number" id="${altitude}" name="altitude" min="0" max="120" step=".1"></label>
                <br>
                (Min 3 meters and Max 120 meters)
                <br>
                <br>
                <label ><input id=${captureImagesCBid}  type='checkbox' style='margin-right: 0'/> ${captureImagesLabel}</label>

                <script>document.getElementById('altitude').addEventListener('change', function (e) {
                    if(this.value < 3){
                      this.value = 3;
                    }else if(this.value>120){
                        this.value=120;
                    } else {
                      this.value = this.value;
                    }
                });
                </script>
<!--    <br>-->     
            </form>`);
        let defer = $.Deferred();
        let dialog = $('<div></div>')
            .appendTo('body')
            .html(customHtml)
            .dialog({
                autoOpen: false,
                height: 'auto',
                width: 600,
                create: function (e, ui) {
                    // var pane = $(this).dialog("widget").find(".ui-dialog-buttonpane")
                    // customHtml.prependTo(pane)
                },
                modal: true,
                title: dialogTitle,
                buttons: [
                    {
                        text: okButton,
                        click: function () {
                            if (document.querySelector('#' + altitude).value !== '') {
                                defer.resolve([
                                    true,
                                    document.querySelector('input[name="mission"]:checked').value,
                                    document.querySelector('#' + altitude).value,
                                    document.querySelector('#' + captureImagesCBid).checked,
                                ]);
                                $(this).remove();
                            } else {
                                defer.resolve([
                                    true,
                                    document.querySelector('input[name="mission"]:checked').value,
                                    0,
                                    document.querySelector('#' + captureImagesCBid).checked,
                                ]);
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
        dialog.dialog('open');
        return defer.promise();
    }

    function create_dialog_with_one_button(
        okButton,
        message,
        dialogTitle,
        width,
        height,
        div_id = 'temp_warning_dialog'
    ) {
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

    function create_dialog_with_one_button_and_checkbox(
        okButton,
        message,
        dialogTitle,
        width,
        height,
        checkboxText,
        div_id = 'temp_warning_dialog'
    ) {
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
        let mydiv = jQuery('<div>', {
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
        mydiv.append(btn);
        setTimeout(function () {
            // $(popup_element).hide()
            $(mydiv).fadeOut();
            $(mydiv).remove();
        }, duration_ms);
        return div_id;
    }

    function create_popup(type, msg, popup_id) {
        let mydiv = jQuery('<div>', {
            id: popup_id,
            class: `center-element alert ${type} fade in alert-dismissible show`,
            css: {
                width: 'fit-content',
                height: 'fit-content',
            },
            text: msg,
        }).appendTo('body');
        return mydiv;
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
        var newlabel = document.createElement('Label');
        newlabel.setAttribute('for', 'myCustomCheckBox' + i);
        newlabel.innerHTML =
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
        iDiv.appendChild(newlabel);
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
