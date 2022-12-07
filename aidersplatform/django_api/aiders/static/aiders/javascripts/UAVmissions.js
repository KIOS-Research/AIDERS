/*
 * Handles the mission operation of drones
 * */

function hasClass(target, className) {
    return new RegExp('(\\s|^)' + className + '(\\s|$)').test(target.className);
}

function isActive(target) {
    return hasClass(target, 'active');
}

var selectElement = document.getElementById('clickSelect');
var goElement = document.getElementById('clickGo');
var cancelElement = document.getElementById('clickCancel');
var at_least_one_drone_selected = false;
var latest_drone_selected_model;

/*
 * Adding button functionality for the buttons that convern the drone mission
 * */
$('.msn').click(function (e) {
    let allDrones;
    let sel_drones;
    let selected_drone_index;
    let selected_drone_id;
    let drone_obj;
    let sel_drone_ids;
    if (this === selectElement || this === goElement || this === cancelElement) {
        allDrones = get_all_drone_info_array();
        sel_drones = get_selected_drones();
        sel_drone_ids = get_selected_drone_ids();
        if (sel_drones.length === 1) {
            selected_drone_id = get_selected_drone_id();
            selected_drone_index = get_drone_index(selected_drone_id);
            drone_obj = get_drone_object(selected_drone_id);
        }
    }

    e.preventDefault();

    //User clicks on "select" element
    if (this === selectElement) {
        // if (selected_drones.length > 1)
        // {
        //     showPopupForALittle('#more_than_one_drone_box', '', 3000)
        //     return
        // }
        buttonDelayer(selectElement);
        if (!at_least_one_drone_selected) {
            //Dont let user select locations if he didn't choose drone yet
            showPopupForALittle('#noDroneBox', '', 2000);
        } else if (missionInProgress(sel_drones) || missionInPause(sel_drones)) {
            //Dont let user select locations if another mission in progress
            showPopupForALittle('#missionInProgressBox', '', 3000);
        } //If no active mission and a drone was selected, user is now able to make selection of locations
        else {
            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_SELECT_MISSION);
            // keep_trying_if_fails(sel_drones,WANT_TO_SELECT_MISSION, 0)
        }
    }
    //User clicks "Go" to start a mission
    else if (this === goElement) {
        buttonDelayer(goElement);
        if (!at_least_one_drone_selected) {
            //Dont let user select locations if he didn't choose drone yet
            showPopupForALittle('#noDroneBox', '', 2000);
        }
        let missionPath = getMissionPath(sel_drone_ids[0]);

        if (missionInProgress(sel_drones)) {
            sendMissionToAPIForSelectedDrones(sel_drones, false, false, MISSION_ACTIONS.PAUSE_MISSION, undefined);

            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_PAUSE_MISSION);

            keep_trying_if_fails(sel_drones, WANT_TO_PAUSE_MISSION, 0);
        } else if (missionInPause(sel_drones)) {
            sendMissionToAPIForSelectedDrones(sel_drones, false, false, MISSION_ACTIONS.RESUME_MISSION, undefined);

            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_RESUME_MISSION);

            keep_trying_if_fails(sel_drones, WANT_TO_RESUME_MISSION, 0);
        } else {
            let currentAlts = get_current_alts(sel_drones);
            let limit = 5;
            if (!isHeightOK(currentAlts, limit)) {
                showPopupForALittle('#droneTooLowForMission', '', 3000);
                return;
            }
            // if (isActive(selectElement) && missionPath.length > 0) //Only start the mission if the "Select" button is active and user selected at least one location
            if (missionPath.length > 0) {
                //Only start the mission if the "Select" button if user selected at least one location
                //MISSION CAN SUCCESSFULLY START
                handleMissionStartProcess(
                    drone_obj,
                    selected_drone_index,
                    selected_drone_id,
                    sel_drones,
                    missionPath,
                    currentAlts
                );
            } //Don't start the mission if we don't have at least 1 location yet
            else {
                console.log('You have to select at least one location to start a mission');
                showPopupForALittle('#noLocationBox', '', 2000);
            }
        }
    }
    //User clicks Cancel button to cancel the current mission or current seleciton of locations
    else if (this === cancelElement) {
        buttonDelayer(cancelElement);
        if (selectionInProgress(sel_drones)) {
            //If "Select" is active, it means user is still on selection process.
            if (confirm('Are you sure you want to clear all of your location selections for this drone?')) {
                //User wants to cancel current selection
                remove_markers_from_selected_drones(sel_drones);
                // reset_marker_counter(selected_drone_id)
                reset_marker_counter_for_selected_drones(sel_drones);
                updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_CANCEL_MISSION);
                keep_trying_if_fails(sel_drones, WANT_TO_CANCEL_MISSION, 0);

                $(selectElement).removeClass('active');
                enableDroneSelection(); //Let user be able to select/unselect drone again
                sel_drones.forEach(function (selDrone) {
                    clearMissionPathLine(selDrone);
                });
                emptyMissionPathForSelectedDrones(sel_drones);
            }
        } else if (missionInProgress(sel_drones) || missionInPause(sel_drones)) {
            ///If "Go" is active, it means mission is in progress
            if (confirm('There is an ongoing mission. Are you sure you want to cancel it?')) {
                //User wants to cancel current mission
                $(goElement).removeClass('active');

                sendMissionToAPIForSelectedDrones(sel_drones, false, false, MISSION_ACTIONS.CANCEL_MISSION, undefined);
                updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_CANCEL_MISSION);

                keep_trying_if_fails(sel_drones, WANT_TO_CANCEL_MISSION, 0);
            } else {
                console.log('Mission not cancelled');
            }
        } else {
            console.log("There isn't anything to cancel");
            showPopupForALittle('#noMissionBox', '', 2000);
        }
    }
});
map.on(
    'click',
    function (
        e // Fired when map is clicked
    ) {
        let allDrones = get_all_drone_info_array();

        let selectedDrone_id = get_selected_drone_id(allDrones);
        let selected_drones = get_selected_drones();
        if (at_least_one_drone_selected && selectionInProgress(selected_drones)) {
            let el = create_marker_point(allDrones, selectedDrone_id);
            console.log('Marker added at ' + e.lngLat);

            let marker = new maplibregl.Marker(el).setLngLat(e.lngLat).addTo(map);

            increase_waypoint_marker_number(selectedDrone_id);

            let locArray = [e.lngLat.lng, e.lngLat.lat];
            selected_drones.forEach(function (selDrone) {
                pushLocToMissionPath(locArray, selDrone.droneID);
                push_marker_to_selected_drone(marker, undefined, selDrone.droneID);
            });
        }
    }
);

function handleMissionStartProcess(drone_obj, selected_drone_index, selected_drone_id, selected_drones, missionPath) {
    create_mission_confirmation_dialog().then(function (willProceedAndisChecked) {
        let canProceed = willProceedAndisChecked[0];
        if (canProceed) {
            let radioBtnSelected = willProceedAndisChecked[1];
            let altitude = willProceedAndisChecked[2];
            let captureAndStoreImages = willProceedAndisChecked[3];
            if (altitude === 0) {
                altitude = get_current_alts(selected_drones);
            } else {
                alt = [];
                selected_drones.forEach((element) => {
                    alt.push(altitude);
                });
                altitude = alt;
            }
            // var altitude = getLastAltitude(selected_drone_id);
            // missionPath = insertHeightToMissionPath(altitude,selected_drone_id);
            let alts = get_current_alts(selected_drones);
            updateSelectedDronesMissionType(selected_drones, radioBtnSelected);
            if (radioBtnSelected === SEARCH_AND_RESCUE_MISSION) {
                let correctSelection = isDroneSelectionCorrect(SEARCH_AND_RESCUE_MISSION, missionPath);
                if (!correctSelection) return;
                fetch_path_plans(missionPath, selected_drones, altitude).then(function (paths) {
                    paths.forEach((e) => addLineLayer1(e));
                    updateSelectedDronesMissionPath(selected_drones, paths);
                    instertHeightToMissionPathForSelectedDrones(selected_drones, altitude);
                    updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_START_MISSION);
                    keep_trying_if_fails(selected_drones, WANT_TO_START_MISSION, 0, false, captureAndStoreImages);
                    sendMissionToAPIForSelectedDrones(
                        selected_drones,
                        false,
                        captureAndStoreImages,
                        MISSION_ACTIONS.START_MISSION,
                        'SEARCH_AND_RESCUE_MISSION'
                    );
                });

                // console.log("PATHS", paths)
            } else if (radioBtnSelected === MAP_MISSION) {
                instertHeightToMissionPathForSelectedDrones(selected_drones, altitude);
                updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_START_MISSION);
                keep_trying_if_fails(selected_drones, WANT_TO_START_MISSION, 0, true, captureAndStoreImages);
                sendMissionToAPIForSelectedDrones(
                    selected_drones,
                    true,
                    captureAndStoreImages,
                    MISSION_ACTIONS.START_MISSION,
                    'GRID_MISSION'
                );
            } else {
                console.log('in SEARCH_AND_RESCUE_MISSION');
                instertHeightToMissionPathForSelectedDrones(selected_drones, altitude);
                updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_START_MISSION);
                keep_trying_if_fails(selected_drones, WANT_TO_START_MISSION, 0, false, captureAndStoreImages);
                sendMissionToAPIForSelectedDrones(
                    selected_drones,
                    false,
                    captureAndStoreImages,
                    MISSION_ACTIONS.START_MISSION,
                    'NORMAL_MISSION'
                );
            }
        }
    });
}

function isDroneSelectionCorrect(typeOfMission, waypoints) {
    if (typeOfMission === SEARCH_AND_RESCUE_MISSION && waypoints.length !== 4) {
        let msg = 'Search and Rescue mission requires exactly 4 waypoints! ';
        create_popup_for_a_little(WARNING_ALERT, msg, 3000);
        return false;
    }
    return true;
}
function get_latest_selected_drone() {
    return latest_drone_selected_model;
}

function create_marker_point(allDrones, droneID = undefined) {
    let el = document.createElement('div');
    el.className = 'marker';
    let color = 'background-color:' + get_selected_drone_marker_color(droneID) + ';';
    let test = '<span style=' + color + '><b>';
    let markerCounter = allDrones[get_drone_index(droneID)].droneMarkerCounter;

    el.innerHTML = test + (markerCounter + 1) + '</b></span>';
    return el;
}

function addLineLayer1(pathObj) {
    let path = pathObj['path'];
    path.pop();
    let droneID = pathObj['drone_id'];
    if (map.getLayer('path_' + droneID)) {
        map.removeLayer('path_' + droneID);
    }
    if (map.getSource('path_' + droneID)) {
        map.removeSource('path_' + droneID);
    }
    map.addSource('path_' + droneID, {
        type: 'geojson',
        data: {
            type: 'Feature',
            properties: {},
            geometry: {
                type: 'LineString',
                coordinates: path,
            },
        },
    });
    map.addLayer(
        {
            id: 'path_' + droneID,
            type: 'line',
            source: 'path_' + droneID,
            layout: {
                'line-join': 'round',
                'line-cap': 'round',
            },
            paint: {
                'line-color': getBasicColor(get_drone_index(droneID)),
                'line-width': 8,
            },
        },
        get_all_drone_info_array()[get_drone_index(droneID)].droneModel.id
    );
}

function clearMissionPathLine(droneID) {
    if (map.getLayer('path_' + droneID)) {
        map.removeLayer('path_' + droneID);
    }
    if (map.getSource('path_' + droneID)) {
        map.removeSource('path_' + droneID);
    }
}
/*Updates the state of the buttons. This can be triggered for example if user refreshed their
 * browser while there was an on-going mission.
 * */
function updateMissionButtonStates(selectedObject) {
    let allDrones = get_all_drone_info_array();
    let clickGo = $('#clickGo');
    let clickSelect = $('#clickSelect');
    let clickCancel = $('#clickCancel');
    if (at_least_one_drone_selected) {
        for (let i = 0; i < allDrones.length; i++) {
            if (selectedObject.uuid === allDrones[i].droneObject.uuid) {
                clickSelect.removeClass('active');
                clickGo.removeClass('active');
                clickCancel.removeClass('active');
                clickGo.text('GO');
                clickGo.removeClass('btn-outline-primary');
                clickGo.addClass('btn-outline-success');
                let state = allDrones[i].droneMissionState;
                if (state === IN_MISSION) {
                    clickGo.addClass('active');
                    clickGo.text('PAUSE');
                    clickGo.removeClass('btn-outline-success');
                    clickGo.addClass('btn-outline-primary');
                } else if (state === PAUSED_MISSION) {
                    clickGo.text('RESUME');
                    // if (hasClass(clickGo,'btn-outline-success'))
                    // {
                    // console.log("HAS_SUCCESS")
                    clickGo.removeClass('btn-outline-success');
                    clickGo.addClass('btn-outline-primary');
                    // }
                    clickGo.addClass('active');
                }
            }
        }
    } else {
        clickSelect.removeClass('active');
        clickGo.removeClass('active');
        clickCancel.removeClass('active');
        clickGo.text('GO');
        clickGo.removeClass('btn-outline-primary');
        clickGo.addClass('btn-outline-success');
    }
}

function atLeastOneDroneSelected() {
    return at_least_one_drone_selected;
}

function disableBuildingSelection() {
    tb.enableSelectingFeatures = false;
}

function enableBuildingSelection() {
    tb.enableSelectingFeatures = true;
}

function disableDroneSelection() {
    tb.enableSelectingObjects = false;
}

function enableDroneSelection() {
    tb.enableSelectingObjects = true;
}

function sendMissionToAPIForSelectedDrones(
    selectedDrones,
    mappingGrid = false,
    captureAndStoreImages = false,
    action,
    mission_type
) {
    for (let i = 0; i < selectedDrones.length; i++) {
        let path = selectedDrones[i].droneMissionPath;
        let drone_name = selectedDrones[i].droneID;
        new_path = [];
        path.forEach((element, index) => {
            new_path[index] = [element[0], element[1], element[2 + i]];
        });
        sendMissionToAPI(new_path, drone_name, mappingGrid, captureAndStoreImages, action, mission_type);
    }

    //==========================================================
}
function sendMissionToAPI(
    missionPath,
    drone_name,
    mappingGrid = false,
    captureAndStoreImages = false,
    action,
    mission_type
) {
    let url = dutils.urls.resolve('mission', { operation_name: CURRENT_OP, drone_name: drone_name });
    console.log(
        'action: ' +
            action +
            ' mission_type: ' +
            mission_type +
            ' grid: ' +
            mappingGrid +
            ' capture_images: ' +
            captureAndStoreImages +
            ' mission_points: ' +
            missionPath
    );
    var settings = {
        url: url,
        method: 'POST',
        headers: {
            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
            'Content-Type': 'application/json',
        },

        data: JSON.stringify({
            action: action,
            mission_type: mission_type,
            grid: mappingGrid,
            captureAndStoreImages: captureAndStoreImages,
            mission_points: missionPath,
        }),
    };

    $.ajax(settings)
        .fail(function (error) {
            console.log('Something went wrong. Mission not started');
            console.log('ERROR: ' + error);
            hidePopup('#loadingBox');
            showPopupForALittle('#failureBox', '', 2000);
        })
        .done(function (response) {
            console.log('RESPONSE: ', response);
        });
}

function missionInProgress(selected_drones) {
    for (let i = 0; i < selected_drones.length; i++) {
        if (selected_drones[i].droneMissionState === IN_MISSION) {
            return true;
        }
    }

    return false;
}

function selectionInProgress(selected_drones) {
    for (let i = 0; i < selected_drones.length; i++) {
        if (selected_drones[i].droneMissionIntent === WANT_TO_SELECT_MISSION) {
            return true;
        }
    }

    return false;
}

function missionInPause(selected_drones) {
    for (let i = 0; i < selected_drones.length; i++) {
        if (selected_drones[i].droneMissionState === PAUSED_MISSION) {
            return true;
        }
    }

    return false;
}

function clearMap(allDrones) {
    for (let i = 0; i < allDrones.length; i++) {
        tb.remove(allDrones[i].droneObject);
        map.removeLayer(allDrones[i].droneLineLayer.id);
        // map.removeLayer(allDrones[i].droneColumnLayer.id)

        allDrones[i].droneLineData = [];
        allDrones[i].droneColumnData = [];
        // map.removeLayer(allDrones[i].droneModel.id)
        // allDrones.splice(i,1)

        allDrones[i].droneLineLayer = createLineLayer(allDrones[i].droneID);
        map.addLayer(allDrones[i].droneLineLayer);
    }
    return allDrones;
}

let want_to_start_mission_retry_timer;
let want_to_pause_mission_retry_timer;
let want_to_cancel_mission_retry_timer;
let want_to_resume_mission_retry_timer;

function clear_interval(whichInterval) {
    if (whichInterval === WANT_TO_START_MISSION) {
        if (want_to_start_mission_retry_timer !== undefined) {
            clearInterval(want_to_start_mission_retry_timer);
        }
    } else if (whichInterval === WANT_TO_PAUSE_MISSION) {
        if (want_to_pause_mission_retry_timer !== undefined) {
            clearInterval(want_to_pause_mission_retry_timer);
        }
    } else if (whichInterval === WANT_TO_RESUME_MISSION) {
        if (want_to_resume_mission_retry_timer !== undefined) {
            clearInterval(want_to_resume_mission_retry_timer);
        }
    } else if (whichInterval === WANT_TO_CANCEL_MISSION) {
        if (want_to_cancel_mission_retry_timer !== undefined) {
            clearInterval(want_to_cancel_mission_retry_timer);
        }
    }
}

function start_retrying(droneMissionIntent, drone, retryCount, mappingGrid = false, storeImages = false) {
    let missionPath = getMissionPath(drone.droneID);

    if (droneMissionIntent === WANT_TO_START_MISSION) {
        showPopup('#loadingBox', '');
        want_to_start_mission_retry_timer = setTimeout(function () {
            if (drone.droneMissionState === IN_MISSION) {
                clearInterval(want_to_start_mission_retry_timer);
            } else {
                retryCount += 1;
                if (retryCount === 4) {
                    hidePopup('#mission_failure_box_retry');
                    hidePopup('#loadingBox');
                    let msg = 'Something went wrong! Mission failed to start after ' + (retryCount - 1) + ' retries';
                    console.log('Send again mission');
                    showPopupForALittle('#mission_failure_box', msg, 3000);
                    let allDrones = get_all_drone_info_array();
                    // emptyMissionPath()

                    updateDroneMissionIntentLocally(drone.droneID, WANT_TO_SELECT_MISSION);

                    start_retrying(WANT_TO_SELECT_MISSION, drone, 0);

                    reset_marker_counter(drone.droneID);
                    return;
                }
                hidePopup('#loadingBox');
                let msg = 'Something went wrong! Mission failed to start. Sending mission again...(' + retryCount + ')';

                showPopup('#mission_failure_box_retry', msg);
                // create_popup_for_a_little(WARNING_ALERT,"Mission failed to start. Sending mission again...",2000)
                sendMissionToAPI(missionPath, drone.droneID, mappingGrid, storeImages, MISSION_ACTIONS.START_MISSION);
                start_retrying(droneMissionIntent, drone, retryCount, mappingGrid, storeImages);
            }
        }, 15000);
        // }, 2000);
    } else if (droneMissionIntent === WANT_TO_PAUSE_MISSION) {
        showPopup('#pausingMissionBox', '');
        want_to_pause_mission_retry_timer = setTimeout(function () {
            console.log('DRONE MISSION STATE: ', drone.droneMissionState);
            if (drone.droneMissionState === PAUSED_MISSION) {
                clearInterval(want_to_pause_mission_retry_timer);
            } else {
                hidePopup('#pausingMissionBox');
                retryCount += 1;
                if (retryCount === 4) {
                    hidePopup('#pause_failure_box_retry');
                    let msg = 'Something went wrong! Mission failed to pause after ' + (retryCount - 1) + ' retries';
                    showPopupForALittle('#pause_failure_box', msg, 3000);
                    return;
                }
                hidePopup('#pausingMissionBox');
                let msg =
                    'Something went wrong! Mission failed to pause. Trying to pause mission again...(' +
                    retryCount +
                    ')';
                showPopup('#pause_failure_box_retry', msg);
                sendMissionToAPI([], drone.droneID, mappingGrid, storeImages);
                start_retrying(
                    droneMissionIntent,
                    drone,
                    retryCount,
                    mappingGrid,
                    storeImages,
                    MISSION_ACTIONS.PAUSE_MISSION
                );
            }
        }, 5000);
    } else if (droneMissionIntent === WANT_TO_RESUME_MISSION) {
        showPopup('#resumingMissionBox', '');
        want_to_resume_mission_retry_timer = setTimeout(function () {
            if (drone.droneMissionState === IN_MISSION) {
                clearInterval(want_to_resume_mission_retry_timer);
            } else {
                retryCount += 1;
                if (retryCount === 4) {
                    hidePopup('#resume_failure_box_retry');
                    let msg = 'Something went wrong! Mission failed to resume after ' + (retryCount - 1) + ' retries';
                    showPopupForALittle('#resume_failure_box', msg, 3000);
                    return;
                }
                hidePopup('#resumingMissionBox');
                let msg =
                    'Something went wrong! Mission failed to resume. Trying to resume mission again...(' +
                    retryCount +
                    ')';
                showPopup('#resume_failure_box_retry', msg);
                sendMissionToAPI([], drone.droneID, mappingGrid, storeImages, MISSION_ACTIONS.RESUME_MISSION);
                start_retrying(droneMissionIntent, drone, retryCount, mappingGrid, storeImages);
            }
        }, 5000);
    } else if (droneMissionIntent === WANT_TO_CANCEL_MISSION) {
        showPopup('#cancellingMissionBox', '');
        want_to_cancel_mission_retry_timer = setTimeout(function () {
            if (drone.droneMissionState === FLYING) {
                clearInterval(want_to_cancel_mission_retry_timer);
            } else {
                hidePopup('#cancellingMissionBox');
                retryCount += 1;
                if (retryCount === 4) {
                    hidePopup('#cancel_failure_box_retry');
                    let msg = 'Something went wrong! Mission failed to cancel after ' + (retryCount - 1) + ' retries';
                    showPopupForALittle('#cancel_failure_box', msg, 3000);
                    return;
                }
                hidePopup('#cancellingMissionBox');
                let msg =
                    'Something went wrong! Mission failed to cancel. Trying to cancel mission again...(' +
                    retryCount +
                    ')';
                showPopup('#cancel_failure_box_retry', msg);
                sendMissionToAPI(
                    [],
                    drone.droneID,
                    mappingGrid,
                    storeImages,
                    MISSION_ACTIONS.CANCEL_MISSION,
                    undefined
                );
                start_retrying(droneMissionIntent, drone, retryCount, mappingGrid, storeImages);
            }
        }, 5000);
    }
}
function keep_trying_if_fails(
    selected_drones,
    droneMissionIntent,
    retryCount,
    mappingGrid = false,
    storeImages = false
) {
    for (let i = 0; i < selected_drones.length; i++) {
        // console.log("INDEX: ", i)
        // console.log("SELECTED DRONE1: ", selected_drones[i])
        start_retrying(droneMissionIntent, selected_drones[i], retryCount, mappingGrid, storeImages);
    }
}

function replayMission() {
    //Make it like build map where a pop up appears and user chooses a build map period
}

function customWaypoint(long, lat) {
    let allDrones = get_all_drone_info_array();
    let selectedDrone_id = get_selected_drone_id(allDrones);
    let selected_drones = get_selected_drones();
    updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_SELECT_MISSION);
    if (at_least_one_drone_selected && selectionInProgress(selected_drones)) {
        let el = create_marker_point(allDrones, selectedDrone_id);
        console.log('Marker added at ' + [long, lat]);
        let marker = new maplibregl.Marker(el).setLngLat([long, lat]).addTo(map);

        increase_waypoint_marker_number(selectedDrone_id);
        selected_drones.forEach(function (selDrone) {
            pushLocToMissionPath([long, lat], selDrone.droneID);
            push_marker_to_selected_drone(marker, undefined, selDrone.droneID);
        });
    }
}
