/*
 * Responsible for handling the button actions for the Build Map functionality.
 * E.g When pressing the "Load Build Map" button
 * */
let global_active_build_map_periods = [];
{
    $('.bldmp').click(function (e) {
        let selected_drones = get_selected_drones();
        let selectedDrone_id = get_selected_drone_id();
        let selectedDrone_ids = get_selected_drone_ids();
        let clickedBtnID = e.target.id;

        let loadBuildMapElement = document.getElementById('loadBuildMapBtn');
        let clearBuildMapElement = document.getElementById('clearBuildMapBtn');
        let startBuildMapElement = document.getElementById('startBuildMapBtn');
        let stopBuildMapElement = document.getElementById('stopBuildMapBtn');
        let LOADURL = dutils.urls.resolve('load_build_map', {
            operation_name: CURRENT_OP,
        });

        if (clickedBtnID === loadBuildMapElement.id) {
            //What happens if user clicks on Load Build Map element
            /*Display a pop up that displays all th ebuild map period the user has performed
             * User can choose any period they want to load on the map
             * */
            $.ajax({
                url: LOADURL,
                cache: false,
                success: function (
                    response //Get the build map periods from API
                ) {
                    response = JSON.parse(response);

                    let not_active_ms_build_map_periods = subtract_two_object_arrays(
                        response,
                        global_active_build_map_periods
                    );
                    if (not_active_ms_build_map_periods.length == 0) {
                        popup_element = '#failureBox';
                        message = 'There not Build Maps available to load.';
                        duration_ms = 3000;
                        showPopupForALittle(popup_element, message, duration_ms);
                    } else {
                        let form_element = create_form_with_checkboxes_for_load_build_map(
                            not_active_ms_build_map_periods
                        );
                        let okButton = 'Load Build Map';
                        let cancelButton = 'Cancel';
                        let dialogTitle = 'Choose Build Map sessions to load';
                        create_build_map_dialog_with_checkboxes_form(
                            form_element,
                            okButton,
                            cancelButton,
                            dialogTitle
                        ).then(function (canProceed_and_dialog) {
                            let canProceed = canProceed_and_dialog[0];
                            let dialog = canProceed_and_dialog[1];

                            if (canProceed) {
                                let checkboxes = $('input[id^="myCustomCheckBox"]');
                                let selected_periods = get_selected_buildmap_period_checkboxes(checkboxes, response);
                                global_active_build_map_periods.push(...selected_periods);
                                selected_periods.forEach(function (selected_option) {
                                    let buildmap_id = selected_option['id'];
                                    postBuildMapLoad(
                                        buildmap_id,
                                        LOADURL,
                                        document.getElementById('csrf').querySelector('input').value
                                    );
                                    dialog.remove();
                                    form_element.remove();
                                });
                            } else {
                                // alert("Something wrong with the build image data.")
                            }
                        });
                    }
                },
                error: function () {
                    showPopupForALittle('#noBuildMapAvailable', '', 3000);
                },
            });
        } else if (clickedBtnID === clearBuildMapElement.id) {
            //What happens if user clicks on Clear button
            let form_element = create_form_with_checkboxes_for_load_build_map(global_active_build_map_periods);
            let okButton = 'Clear';
            let cancelButton = 'Cancel';
            let dialogTitle = 'Choose Build Map sessions to clear';
            if (global_active_build_map_periods.length == 0) {
                popup_element = '#failureBox';
                message = 'There not Build Maps on the map to clean.';
                duration_ms = 3000;
                showPopupForALittle(popup_element, message, duration_ms);
            } else {
                create_build_map_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                    function (canProceed_and_dialog) {
                        let canProceed = canProceed_and_dialog[0];
                        let dialog = canProceed_and_dialog[1];
                        if (canProceed) {
                            let checkboxes = $('input[id^="myCustomCheckBox"]');
                            let selected_buildmap_periods_to_clear = get_selected_buildmap_period_checkboxes(
                                checkboxes,
                                global_active_build_map_periods
                            );
                            global_active_build_map_periods = subtract_two_object_arrays(
                                global_active_build_map_periods,
                                selected_buildmap_periods_to_clear
                            );
                            selected_buildmap_periods_to_clear.forEach(function (selected_option) {
                                let buildmap_id = selected_option['id'];
                                postRemoveMapLoad(
                                    buildmap_id,
                                    LOADURL,
                                    document.getElementById('csrf').querySelector('input').value
                                );
                                dialog.remove();
                                form_element.remove();
                            });
                        }
                    }
                );
            }
        } else if (clickedBtnID === startBuildMapElement.id) {
            buttonDelayer(startBuildMapElement);
            let isUserSelectionCorrect = check_user_selection_correctness(selected_drones, selectedDrone_ids);
            if (!isUserSelectionCorrect) return;

            selectedDrone_ids.forEach(function (selectedDrone_id) {
                console.log(selectedDrone_id);
                let localBuildMapStats = getLocalBuildMapState(selectedDrone_id);
                if (localBuildMapStats === BUILD_MAP_STARTED || localBuildMapStats === BUILD_MAP_ABOUT_TO_START) {
                    showPopupForALittle('#buildMapAlreadyStarted', '', 3000);
                    return;
                }

                let selectedDrone = get_drone_object(selectedDrone_id);
                let altitude = selectedDrone.droneInfo.currentCoordinate[2];
                if (altitude < 25) {
                    //Do not let user activate the Build Map functionality under 25m. There is no point.
                    showPopupForALittle('#droneTooLowForBuildMap', '', 3000);
                    return;
                }
                // showPopupForALittle('#buildMapAlreadyStarted','',3000 )
                updateBuildMapStatusLocally(selectedDrone_id, BUILD_MAP_ABOUT_TO_START);
                updateBuildMapStatusOnAPI(selectedDrone_id, true);
                popup_BuildMapLoading(SHOW_POPUP, selectedDrone_id, 0);
            });
        } else if (clickedBtnID === stopBuildMapElement.id) {
            buttonDelayer(stopBuildMapElement);
            let isUserSelectionCorrect = check_user_selection_correctness(selected_drones, selectedDrone_ids);
            if (!isUserSelectionCorrect) return;
            selectedDrone_ids.forEach(function (selectedDrone_id) {
                let localBuildMapStats = getLocalBuildMapState(selectedDrone_id);

                if (localBuildMapStats === BUILD_MAP_STOPPED || localBuildMapStats === BUILD_MAP_ABOUT_TO_STOP) {
                    showPopupForALittle('#buildMapNotActive', '', 2000);
                    return;
                }

                updateBuildMapStatusLocally(selectedDrone_id, BUILD_MAP_ABOUT_TO_STOP);
                updateBuildMapStatusOnAPI(selectedDrone_id, false);
            });
            popup_BuildMapLoadingToStop(SHOW_POPUP, USER_WANTS_TO_STOP_IT);
            setTimeout(goOn, 1000);

            //Once the build map is stopped, on the backend, the "end_time" of this session is updated
            //with the function updateBuildMapStatusOnAPI (used above).
            //SOmetimes however, the following code (trying to get last build map period) is executed before the
            //above. And since the "end_time" is still null, there is an error on the response. For this reason,
            //we execute the last build map period fetch after some delay
            function goOn() {
                $.ajax({
                    type: 'POST',
                    url: dutils.urls.resolve('get_last_build_map', {
                        operation_name: CURRENT_OP,
                    }),
                    data: { drone_id: selectedDrone_id },
                    headers: {
                        'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                    },
                    success: function (response) {
                        response = JSON.parse(response);
                        global_active_build_map_periods.push(response);
                    },
                });
            }

            try {
                clearInterval(intervalForPhotoLoader);
            } catch (error) {}
            hidePopup('#buildMapFailedToStart');
            // update_global_active_periods() //User just stopped a build map period. Wait for it to be published on API and retrieve it.
        }
    });

    /*Updates the global active periods with the last period that just finished now
     * It continuously checks for a few seconds, if a new build map period was added to the API.
     * */
    function update_global_active_periods() {
        get_build_map_periods_from_api().then(function (periods1) {
            let currPeriods = $.csv.toObjects(periods1);
            let prevPeriods = currPeriods;
            let iterations = 0;
            let tempInterval = setInterval(function () {
                get_build_map_periods_from_api().then(function (periods2) {
                    currPeriods = $.csv.toObjects(periods2);
                    if (currPeriods.length > prevPeriods.length) {
                        console.log('The last completed period is now added to active periods!');
                        global_active_build_map_periods.push(currPeriods[currPeriods.length - 1]);
                        clearInterval(tempInterval);
                    }
                    prevPeriods = currPeriods;
                    if (iterations === 6) {
                        clearInterval(tempInterval);
                    }

                    iterations++;
                });
            }, 2000);
        });
    }

    /*
     * Retrieves the build map periods from the API
     * */
    function get_build_map_periods_from_api() {
        return $.ajax({
            url: WEBSERVER_URL_FOR_BUILDMAP_METADATA,
            cache: false,
            success: function (buildmap_periods) {
                buildmap_periods = $.csv.toObjects(buildmap_periods);
                return buildmap_periods;
            },
        });
    }

    function updateBuildMapStatusOnAPI(drone_id, canStartBuildMap) {
        // if (document.getElementById('selectedCamera').value === 'Default') {
        //     multispectral_build_map = false;
        // } else {
        //     multispectral_build_map = true;
        // }
        multispectral_build_map = false;
        let settings = {
            url: dutils.urls.resolve('start_build_map', {
                operation_name: CURRENT_OP,
            }),
            method: 'POST',
            timeout: 0,
            headers: {
                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
            },
            data: {
                drone_id: drone_id,
                start_build_map_boolean: canStartBuildMap,
                overlap: document.getElementById('overlapValue').innerHTML,
                start_multispectral_build_map: multispectral_build_map,
            },
        };
        console.log('in updateBuildMapStatusOnAPI ');
        $.ajax(settings);
    }

    /*
     * Updates the state of the build map buttons.
     * For example, if user refreshes while there is a multispectral mission going on, buttons should
     * have their previous state
     * */
    function updateBuildMapButtonStates(drone, isSelected) {
        let startBuildMapElement = document.getElementById('startBuildMapBtn');

        let buildMapStatus = getLocalBuildMapState(drone.droneID);
        if (isSelected && buildMapStatus === BUILD_MAP_STARTED) {
            hightlightElement(startBuildMapElement, ACTIVATE_HIGHLIGHT);
        } else if (!isSelected) {
            hightlightElement(startBuildMapElement, DEACTIVATE_HIGHLIGHT);
        }
    }

    /*
     * A simple yes or no pop up dialog
     * */
    function confirmation(question) {
        var defer = $.Deferred();
        $('<div></div>')
            .html(question)
            .dialog({
                autoOpen: true,
                modal: true,
                title: 'Confirmation',
                buttons: {
                    Yes: function () {
                        defer.resolve(true); //answer
                        $(this).dialog('close');
                    },
                    No: function () {
                        defer.resolve(false); //answer
                        $(this).dialog('close');
                    },
                },
                close: function () {
                    $(this).remove(); //removes this dialog div from DOM
                },
            });
        return defer.promise();
    }

    /*Returns which checkboxes are checked*/
    function get_selected_buildmap_period_checkboxes(checkboxes, all_buildmap_periods) {
        let selected_buildmap_periods = [];
        $(checkboxes).each(function () {
            let checkbox = this;

            if (checkbox.checked) {
                //for every checked checkbox, find its index
                for (let i = 0; i < all_buildmap_periods.length; i++) {
                    let drone_id = all_buildmap_periods[i]['drone_id'];
                    let start_time = convertUTCDateToLocalDate(all_buildmap_periods[i]['start_time']);
                    let valueToCheck = drone_id + start_time;
                    if (valueToCheck === $(checkbox).val()) {
                        selected_buildmap_periods.push(all_buildmap_periods[i]);
                    }
                }
            }
        });

        return selected_buildmap_periods;
    }
    /*
        Post data of the remove image build
    */
    function postRemoveMapLoad(build_map_id, url, csrftoken) {
        $.ajax({
            type: 'POST',
            url: url,
            data: JSON.stringify({ build_map_id: build_map_id }),
            headers: {
                'X-CSRFToken': csrftoken,
            },
            success: function (response) {
                response = JSON.parse(response);
                if (response.length === 0) {
                    return;
                }
                response.forEach(function (image, index) {
                    let image_id = image['id'].toString();
                    let layer_id = 'Layers' + image['id'].toString();
                    if (map.getLayer(layer_id)) {
                        map.removeLayer(layer_id);
                    }
                    if (map.getSource(image_id)) {
                        map.removeSource(image_id);
                    }
                });
            },
        });
    }

    /*
        Post data of the image build
    */
    function postBuildMapLoad(build_map_id, url, csrftoken) {
        $.ajax({
            type: 'POST',
            url: url,
            data: JSON.stringify({ build_map_id: build_map_id }),
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': csrftoken,
            },
            success: function (response) {
                response = JSON.parse(response);
                if (response.length === 0) {
                    return;
                }
                response.forEach(function (image, index) {
                    let image_url = dutils.urls.resolve('build_map_image_path', { image_path: image['path'] });
                    let image_id = image['id'].toString();
                    let layer_id = 'Layers' + image['id'].toString();
                    let drone = undefined;
                    if (!map.getSource(image_id)) {
                        map.addSource(image_id, {
                            type: 'image',
                            url: image_url,
                            coordinates: [
                                [image['top_right'][0], image['top_right'][1]],
                                [image['bottom_right'][0], image['bottom_right'][1]],
                                [image['bottom_left'][0], image['bottom_left'][1]],
                                [image['top_left'][0], image['top_left'][1]],
                            ],
                        });
                        let allDrones = get_all_drone_info_array();
                        if (allDrones.length > 0) {
                            drone = allDrones[0].droneModel.id;
                        }
                        if (drone !== undefined) {
                            map.addLayer(
                                {
                                    id: layer_id,
                                    type: 'raster',
                                    source: image_id,
                                    paint: { 'raster-opacity': 0.6 },
                                },
                                drone
                            );
                        } else {
                            map.addLayer({
                                id: layer_id,
                                type: 'raster',
                                source: image_id,
                                paint: { 'raster-opacity': 0.6 },
                            });
                        }
                        if (index == 0) {
                            map.flyTo({
                                zoom: 20,
                                center: [image['centre'][0], image['centre'][1]],
                                bearing: image['bearing'],
                            });
                        }
                    }
                });
            },
        });
    }

    /*
     * Attaches on the map the build map photos from periods that user selected to load
     * */

    function attachOldBuildMapPhotos(drone_id, csv_url, photo_folder_url, end_time) {
        $.ajax({
            url: csv_url,
            success: function (data) {
                let mydata = $.csv.toObjects(data); //Get all the rows from this csv file of this build map period

                if (mydata.length === 0) {
                    return;
                }
                mydata.forEach(function (photo) {
                    let photo_url = photo_folder_url + '/' + photo.photo_id + '.jpeg';
                    $.ajax({
                        url: photo_url, //check first if the photo was actually created
                        success: function () {
                            let photo_source_id = '';
                            let photo_layer_id = '';
                            if (photo_url.includes(IN_PROGRESS_STRING)) {
                                photo_source_id = 'source_' + photo_url.replace(IN_PROGRESS_STRING, '');
                                photo_layer_id = 'layer_' + photo_url.replace(IN_PROGRESS_STRING, '');
                            } else {
                                photo_source_id = 'source_' + photo_url.replace(end_time, '');
                                photo_layer_id = 'layer_' + photo_url.replace(end_time, '');
                            }

                            if (!map.getSource(photo_source_id)) {
                                map.addSource(photo_source_id, {
                                    type: 'image',
                                    url: photo_url,
                                    coordinates: [
                                        [parseFloat(photo.p1lon), parseFloat(photo.p1lat)],
                                        [parseFloat(photo.p2lon), parseFloat(photo.p2lat)],
                                        [parseFloat(photo.p3lon), parseFloat(photo.p3lat)],
                                        [parseFloat(photo.p4lon), parseFloat(photo.p4lat)],
                                    ],
                                });

                                let allDrones = get_all_drone_info_array();
                                let firstDroneModelLayerID = undefined;
                                if (allDrones.length > 0) {
                                    firstDroneModelLayerID = allDrones[0].droneModel.id;
                                }

                                if (firstDroneModelLayerID !== undefined) {
                                    map.addLayer(
                                        {
                                            id: photo_layer_id,
                                            source: photo_source_id,
                                            type: 'raster',
                                            paint: { 'raster-opacity': 0.6 },
                                        },
                                        firstDroneModelLayerID
                                    );
                                } else {
                                    map.addLayer({
                                        id: photo_layer_id,
                                        source: photo_source_id,
                                        type: 'raster',
                                        paint: { 'raster-opacity': 0.6 },
                                    });
                                }
                            }
                        },
                    });
                });
            },
        });
    }

    function check_user_selection_correctness(selected_drones, selectedDrone_ids) {
        // if (selected_drones.length > 1)
        // {
        //     showPopupForALittle('#more_than_one_drone_box', 'One Drone At a Time', 5000)
        //     return false
        // }
        if (selected_drones.length < 1) {
            showPopupForALittle('#noDroneBox', 'No Drone Found', 2000);
            return false;
        }
        return true;
    }

    function getActiveBuildMapPeriods() {
        return global_active_build_map_periods;
    }

    /*
     * This function is executed once every drone is inserted.
     * It is needed in case user closes the page in the middle of the build map
     * process. In such case we need to retrieve the state of the ROS build map
     * */
    // function update_local_build_map_status_only_once_for_all_drones(new_drone_ids)
    // {
    //     for (let i = 0; i < new_drone_ids.length; i++)
    //     {
    //         let droneID = new_drone_ids[i]
    //         getBuildMapStatusFromAPI(droneID).then(function (build_map_stats_obj)
    //         {
    //             let buildMapStarted = ( (JSON.stringify(build_map_stats_obj["start_build_map"])) === 'true');
    //             if (buildMapStarted)
    //             {
    //
    //                 updateBuildMapStatusLocally(droneID,BUILD_MAP_WANT_TO_START) //This will let the platform get in the code that attaches the photos on the map
    //             }
    //         });
    //     }
    // }
}
