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
var goElementSecondary = document.getElementById('clickGoSecondary');
var cancelElement = document.getElementById('clickCancel');
var cancelElementSecondary = document.getElementById('clickCancelSecondary');
var at_least_one_drone_selected = false;
var latest_drone_selected_model;

/*
 * Adding button functionality for the buttons that convert the drone mission
 * */
$('.msn').click(function (e) {
    let allDrones;
    let sel_drones;
    let selected_drone_index;
    let selected_drone_id;
    let drone_obj;
    let sel_drone_ids;
    if (this === selectElement || this === goElement || this === goElementSecondary || this === cancelElement || this === cancelElementSecondary) {
        allDrones = get_all_drone_info_array();
        sel_drones = get_selected_drones();
        sel_drone_ids = get_selected_drone_ids();
        if (sel_drones.length === 1) {
            selected_drone_id = get_selected_drone_id();
            selected_drone_index = get_drone_index(selected_drone_id);
            drone_obj = get_drone_object(selected_drone_id);
        }
        if (this === selectElement) {
            postElementId('Mission Select', 'Click');
        } else if (this === cancelElement || this === cancelElementSecondary) {
            postElementId('Mission Cancel', 'Click');
        } else {
            postElementId('Mission ' + this.innerHTML, 'Click');
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
        // buttonDelayer(selectElement);
        if (!at_least_one_drone_selected) {
            //Dont let user select locations if he didn't choose drone yet
            showPopupForALittle('#noDroneBox', '', 2000);
        } else if (missionInProgress(sel_drones) || missionInPause(sel_drones)) {
            //Dont let user select locations if another mission in progress
            showPopupForALittle('#missionInProgressBox', '', 3000);
        } //If no active mission and a drone was selected, user is now able to make selection of locations
        else {
            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_SELECT_MISSION);
        }
    }


    //User clicks "Go" to start a mission
    else if (this === goElement || this === goElementSecondary) {
        buttonDelayer(goElement);
        if (!at_least_one_drone_selected) {
            //Dont let user select locations if he didn't choose drone yet
            showPopupForALittle('#noDroneBox', '', 2000);
        }
        // let missionPath = getMissionPath(sel_drone_ids[0]);
        let missionPath = getDroneAttributeValue(sel_drone_ids[0], "droneMissionPath")

        if (missionInProgress(sel_drones)) {
            sendMissionToAPIForSelectedDrones(sel_drones, undefined, undefined, undefined, false, false, MISSION_ACTIONS.PAUSE_MISSION, undefined);

            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_PAUSE_MISSION);

            // keep_trying_if_fails(sel_drones, WANT_TO_PAUSE_MISSION, 0);
        } 
        else if (missionInPause(sel_drones)) {
            sendMissionToAPIForSelectedDrones(sel_drones, undefined, undefined, undefined, false, false, MISSION_ACTIONS.RESUME_MISSION, undefined);

            updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_RESUME_MISSION);

            // keep_trying_if_fails(sel_drones, WANT_TO_RESUME_MISSION, 0);
        } 
        else {
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
                handleMissionStartProcess(drone_obj, selected_drone_index, selected_drone_id, sel_drones, missionPath, currentAlts);
            } //Don't start the mission if we don't have at least 1 location yet
            else {
                console.log('You have to select at least one location to start a mission');
                showPopupForALittle('#noLocationBox', '', 2000);
            }
        }
    }


    //User clicks Cancel button to cancel the current mission or current seleciton of locations
    else if (this === cancelElement || this === cancelElementSecondary) {
        buttonDelayer(cancelElement);
        if (selectionInProgress(sel_drones)) {
            //If "Select" is active, it means user is still on selection process.
            if (confirm('Are you sure you want to clear all of your location selections for this drone?')) {
                //User wants to cancel current selection
                remove_markers_from_selected_drones(sel_drones);
                // reset_marker_counter(selected_drone_id)
                reset_marker_counter_for_selected_drones(sel_drones);
                updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_CANCEL_MISSION);
                // keep_trying_if_fails(sel_drones, WANT_TO_CANCEL_MISSION, 0);

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

                sendMissionToAPIForSelectedDrones(
                    sel_drones,
                    undefined,
                    undefined,
                    undefined,
                    false,
                    false,
                    MISSION_ACTIONS.CANCEL_MISSION,
                    undefined
                );
                updateSelectedDroneMissionIntentsLocally(sel_drones, WANT_TO_CANCEL_MISSION);

                // keep_trying_if_fails(sel_drones, WANT_TO_CANCEL_MISSION, 0);
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

            selected_drones.forEach(function (selDrone) {
                let locArray = [e.lngLat.lng, e.lngLat.lat];
                pushLocToMissionPath(locArray, selDrone.droneID);
                push_marker_to_selected_drone(marker, undefined, selDrone.droneID);
            });
        }
    }
);







// mission setup wizard
function handleMissionStartProcess(drone_obj, selected_drone_index, selected_drone_id, selected_drones, missionPath) {

    create_mission_dialog_types(selected_drones, missionPath).then(function (dialog_types_checks) { // "finish" button pressed
        let canProceed = dialog_types_checks[0];
        if (canProceed) {
            let missionType = Object.keys(dialog_types_checks[1])[0];
            let missionAlt = Object.keys(dialog_types_checks[2])[0];
            let missionAltData = dialog_types_checks[2][missionAlt];
            let missionSpeed = Object.keys(dialog_types_checks[3])[0];
            let missionSpeedData = dialog_types_checks[3][missionSpeed];
            let missionGimbal = Object.keys(dialog_types_checks[4])[0];
            let missionGimbalData = dialog_types_checks[4][missionGimbal];
            let missionRepeat = Object.keys(dialog_types_checks[5])[0];
            let missionRepeatData = parseInt(dialog_types_checks[5][missionRepeat]);
            let missionImages = Object.keys(dialog_types_checks[6])[0];
            let missionImagesData = dialog_types_checks[6][missionImages];

            // console.log("//////////////////////////");
            // console.log("missionAlt");
            // console.log(missionAlt);
            // console.log("missionAltData");
            // console.log(missionAltData);
            // console.log("//////////////////////////");

            if (missionAlt == ALL_SAME_ALTITUDE) {
                // temp = [];
                // for (let index = 0; index < selected_drones.length; index++) {
                //     temp.push(parseInt(missionAltData));
                // }
                // missionAltData = temp;
            } 
            else if (missionAlt == ALL_DIFFERENT_ALTITUDE) {
                temp = [];
                for (let index = 0; index < selected_drones.length; index++) {
                    const drone = selected_drones[index].droneID;
                    temp.push(parseInt(missionAltData[index][drone]));
                    // temp.push([parseInt(missionAltData[index][drone]),parseInt(missionAltData[index][drone])])
                }
                missionAltData = temp;
            } 
            else if (missionAlt == ALL_CURRENT_ALTITUDE) {
                missionAltData = get_current_alts(selected_drones);
                // for (let index = 0; index < selected_drones.length; index++) {
                //     const drone = selected_drones[index].droneID;
                //     temp = [];
                //     // HUH ?
                //     for (let indexPath = 0; indexPath < missionPath.length; indexPath++) {
                //         temp.push(missionAltData[index]);
                //         // temp.push([missionAltData[index],missionAltData[index],missionAltData[index],missionAltData[index]]);
                //     }
                //     missionAltData[index] = temp;
                // }
            } 
            else if (missionAlt == CUSTOM_ALTITUDE) {
                // dronesCurrentAlt = get_current_alts(selected_drones);
                // for (let index = 0; index < selected_drones.length; index++) {
                //     for (let indexPath = 0; indexPath < missionPath.length; indexPath++) {
                //         if (missionAltData[index][indexPath] == '') {
                //             missionAltData[index][indexPath] = dronesCurrentAlt[index];
                //         }
                //     }
                // }
            }

            console.log("+++++++++++++++++++++++++++++++++");
            console.log("missionAltData PROCESSED");
            console.log(missionAltData);
            console.log("+++++++++++++++++++++++++++++++++");

            if (missionSpeed == ALL_SAME_SPEED) {
                temp = [];
                for (let index = 0; index < selected_drones.length; index++) {
                    temp.push(missionSpeedData);
                }
                missionSpeedData = temp;
            } else if (missionSpeed == ALL_DIFFERENT_SPEED) {
                temp = [];
                for (let index = 0; index < selected_drones.length; index++) {
                    const drone = selected_drones[index].droneID;
                    temp.push(missionSpeedData[index]);
                }
                missionSpeedData = temp;
            }

            updateSelectedDronesMissionType(selected_drones, missionType);



            if (missionType == SEARCH_AND_RESCUE_MISSION) {
                let correctSelection = isDroneSelectionCorrect(SEARCH_AND_RESCUE_MISSION, missionPath);
                if (!correctSelection) return;
                fetch_path_plans(missionPath, selected_drones, missionAltData, missionSpeedData).then(function (paths) {

                    // SafeDrones
                    if(paths[0] == -1) {
                        // console.log("AOU");
                        // showPopupForALittle('#mission_failure_box', "The mission was not sent due to a high risk of collision.", 5000);
                        create_popup_for_a_little(FAILED_ALERT, "The mission was not sent due to a high risk of collision. (P_danger_risk: " + (paths[1]*100) + "%, P_collision_risk: " + (paths[2]*100) + "%)", 15000);
                        return;
                    }

                    paths.forEach((e) => addLineLayer1(e));
                    updateSelectedDronesMissionPath(selected_drones, paths);
                    updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_START_MISSION);
                    sendMissionToAPIForSelectedDrones(
                        selected_drones,
                        missionSpeedData,
                        'None',
                        missionRepeatData,
                        false,
                        missionImagesData,
                        MISSION_ACTIONS.START_MISSION,
                        'SEARCH_AND_RESCUE_MISSION'
                    );
                });
            } 

            else if (missionType == GRID_MISSION) {
                
            }

            // point to point
            else {
                insertHeightToMissionPathForSelectedDrones(selected_drones, missionAltData);
                updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_START_MISSION);
                console.log("WANT_TO_START_MISSION");
                sendMissionToAPIForSelectedDrones(
                    selected_drones,
                    missionSpeedData,
                    missionGimbalData,
                    missionRepeatData,
                    false,
                    missionImagesData,
                    MISSION_ACTIONS.START_MISSION,
                    'NORMAL_MISSION'
                );
            }
        }
    });
}






















function sendMissionToAPIForSelectedDrones(
    selectedDrones,
    dronesSpeed = 4,
    droneGimbal = 'None',
    repeatMission = 0,
    mappingGrid = false,
    captureAndStoreImages = false,
    action,
    mission_type
) {
    console.log("MISSION: sendMissionToAPIForSelectedDrones()");
    for (let i = 0; i < selectedDrones.length; i++) {
        let drone_name = selectedDrones[i].droneID;
        let path = selectedDrones[i].droneMissionPath;
        console.log(path);

        new_gimbal = 'None';
        if (droneGimbal != '') {
            if (droneGimbal[i] != '') {
                new_gimbal = droneGimbal[i];
            }
        }
        sendMissionToAPI(path, drone_name, dronesSpeed[i], new_gimbal, repeatMission, mappingGrid, captureAndStoreImages, action, mission_type);
    }
}







function sendMissionToAPI(
    missionPath,
    drone_name,
    dronesSpeed = 0,
    droneGimbal = 'None',
    repeatMission = 0,
    mappingGrid = false,
    captureAndStoreImages = false,
    action,
    mission_type
) {
    console.log("MISSION: sendMissionToAPI()");
    if (droneGimbal == '') {
        droneGimbal = 'None';
    }
    let url = dutils.urls.resolve('mission', { operation_name: CURRENT_OP, drone_name: drone_name });

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
            mission_speeds: dronesSpeed,
            mission_gimbal: droneGimbal,
            mission_repeat: repeatMission,
        }),
    };

    $.ajax(settings)
        .fail(function (error) {
            console.log('Something went wrong. Mission not started');
            hidePopup('#loadingBox');
            create_popup_for_a_little(WARNING_ALERT, 'Something went wrong. Mission not started.', 2000);
            showPopupForALittle('#failureBox', '', 2000);
        })
        .done(function (response) {
            console.log('RESPONSE: ', response);
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
        get_all_drone_info_array()[0].droneModel.id
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



































/**************/
/* NOT USED ? */
/**************/



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

function customWaypoints(long, lat) {
    for (let i = 0; i < long.length; i++) {
        let allDrones = get_all_drone_info_array();
        let selectedDrone_id = get_selected_drone_id(allDrones);
        let selected_drones = get_selected_drones();
        updateSelectedDroneMissionIntentsLocally(selected_drones, WANT_TO_SELECT_MISSION);
        if (at_least_one_drone_selected && selectionInProgress(selected_drones)) {
            let el = create_marker_point(allDrones, selectedDrone_id);
            console.log('Marker added at ' + [long[i], lat[i]]);
            let marker = new maplibregl.Marker(el).setLngLat([long[i], lat[i]]).addTo(map);

            increase_waypoint_marker_number(selectedDrone_id);
            selected_drones.forEach(function (selDrone) {
                pushLocToMissionPath([long[i], lat[i]], selDrone.droneID);
                push_marker_to_selected_drone(marker, undefined, selDrone.droneID);
            });
        }
    }
}










// function start_retrying(
//     droneMissionIntent,
//     drone,
//     dronesSpeeds,
//     droneGimbal = "None",
//     repeatMission,
//     retryCount,
//     mappingGrid = false,
//     storeImages = false,
//     path = []
// ) {
//     console.log("start_retrying")
//     let missionPath = path;
//     if (droneMissionIntent === WANT_TO_START_MISSION) {
//         showPopup('#loadingBox', '');
//         want_to_start_mission_retry_timer = setTimeout(function () {
//             if (drone.droneMissionState === IN_MISSION) {
//                 clearInterval(want_to_start_mission_retry_timer);
//             } else {
//                 retryCount += 1;
//                 if (retryCount === 4) {
//                     hidePopup('#mission_failure_box_retry');
//                     hidePopup('#loadingBox');
//                     let msg = 'Something went wrong! Mission failed to start after ' + (retryCount - 1) + ' retries';
//                     console.log('Send again mission');
//                     showPopupForALittle('#mission_failure_box', msg, 3000);
//                     let allDrones = get_all_drone_info_array();
//                     // emptyMissionPath()

//                     updateDroneMissionIntentLocally(drone.droneID, WANT_TO_SELECT_MISSION);

//                     start_retrying(WANT_TO_SELECT_MISSION, drone, 0);

//                     reset_marker_counter(drone.droneID);
//                     return;
//                 }
//                 hidePopup('#loadingBox');
//                 let msg = 'Something went wrong! Mission failed to start. Sending mission again...(' + retryCount + ')';

//                 showPopup('#mission_failure_box_retry', msg);

//                 // create_popup_for_a_little(WARNING_ALERT,"Mission failed to start. Sending mission again...",2000)

//                 sendMissionToAPI(
//                     missionPath,
//                     drone.droneID,
//                     dronesSpeeds,
//                     droneGimbal,
//                     repeatMission,
//                     mappingGrid,
//                     storeImages,
//                     MISSION_ACTIONS.START_MISSION
//                     // PARAM MISSING (mission type)
//                 );
//                 start_retrying(
//                     droneMissionIntent,
//                     drone,
//                     dronesSpeeds,
//                     droneGimbal,
//                     repeatMission,
//                     retryCount,
//                     mappingGrid,
//                     storeImages,
//                     missionPath
//                 );
//             }
//         }, 15000);
//     } else if (droneMissionIntent === WANT_TO_PAUSE_MISSION) {
//         showPopup('#pausingMissionBox', '');
//         want_to_pause_mission_retry_timer = setTimeout(function () {
//             console.log('DRONE MISSION STATE: ', drone.droneMissionState);
//             if (drone.droneMissionState === PAUSED_MISSION) {
//                 clearInterval(want_to_pause_mission_retry_timer);
//             } else {
//                 hidePopup('#pausingMissionBox');
//                 retryCount += 1;
//                 if (retryCount === 4) {
//                     hidePopup('#pause_failure_box_retry');
//                     let msg = 'Something went wrong! Mission failed to pause after ' + (retryCount - 1) + ' retries';
//                     showPopupForALittle('#pause_failure_box', msg, 3000);
//                     return;
//                 }
//                 hidePopup('#pausingMissionBox');
//                 let msg = 'Something went wrong! Mission failed to pause. Trying to pause mission again...(' + retryCount + ')';
//                 showPopup('#pause_failure_box_retry', msg);
//                 sendMissionToAPI([], drone.droneID, dronesSpeeds, droneGimbal, repeatMission, mappingGrid, storeImages);
//                 start_retrying(
//                     droneMissionIntent,
//                     drone,
//                     dronesSpeeds,
//                     droneGimbal,
//                     repeatMission,
//                     retryCount,
//                     mappingGrid,
//                     storeImages,
//                     MISSION_ACTIONS.PAUSE_MISSION,
//                     []
//                 );
//             }
//         }, 5000);
//     } else if (droneMissionIntent === WANT_TO_RESUME_MISSION) {
//         showPopup('#resumingMissionBox', '');
//         want_to_resume_mission_retry_timer = setTimeout(function () {
//             if (drone.droneMissionState === IN_MISSION) {
//                 clearInterval(want_to_resume_mission_retry_timer);
//             } else {
//                 retryCount += 1;
//                 if (retryCount === 4) {
//                     hidePopup('#resume_failure_box_retry');
//                     let msg = 'Something went wrong! Mission failed to resume after ' + (retryCount - 1) + ' retries';
//                     showPopupForALittle('#resume_failure_box', msg, 3000);
//                     return;
//                 }
//                 hidePopup('#resumingMissionBox');
//                 let msg = 'Something went wrong! Mission failed to resume. Trying to resume mission again...(' + retryCount + ')';
//                 showPopup('#resume_failure_box_retry', msg);
//                 sendMissionToAPI([], drone.droneID, mappingGrid, storeImages, MISSION_ACTIONS.RESUME_MISSION);
//                 start_retrying(droneMissionIntent, drone, dronesSpeeds, droneGimbal, repeatMission, retryCount, mappingGrid, storeImages, []);
//             }
//         }, 5000);
//     } else if (droneMissionIntent === WANT_TO_CANCEL_MISSION) {
//         showPopup('#cancellingMissionBox', '');
//         want_to_cancel_mission_retry_timer = setTimeout(function () {
//             if (drone.droneMissionState === FLYING) {
//                 clearInterval(want_to_cancel_mission_retry_timer);
//             } else {
//                 hidePopup('#cancellingMissionBox');
//                 retryCount += 1;
//                 if (retryCount === 4) {
//                     hidePopup('#cancel_failure_box_retry');
//                     let msg = 'Something went wrong! Mission failed to cancel after ' + (retryCount - 1) + ' retries';
//                     showPopupForALittle('#cancel_failure_box', msg, 3000);
//                     return;
//                 }
//                 hidePopup('#cancellingMissionBox');
//                 let msg = 'Something went wrong! Mission failed to cancel. Trying to cancel mission again...(' + retryCount + ')';
//                 showPopup('#cancel_failure_box_retry', msg);
//                 sendMissionToAPI([], drone.droneID, mappingGrid, storeImages, MISSION_ACTIONS.CANCEL_MISSION, undefined);
//                 start_retrying(droneMissionIntent, drone, dronesSpeeds, droneGimbal, repeatMission, retryCount, mappingGrid, storeImages, []);
//             }
//         }, 5000);
//     }
// }
















// function keep_trying_if_fails(
//     selected_drones,
//     droneMissionIntent,
//     retryCount,
//     dronesSpeeds = '',
//     droneGimbal = '',
//     repeatMission = 0,
//     mappingGrid = false,
//     storeImages = false
// ) {
//     console.log("MISSION: keep_trying_if_fails()");
//     for (let i = 0; i < selected_drones.length; i++) {  // loop selected drones
//         let path = selected_drones[i].droneMissionPath;
//         let speed = dronesSpeeds;
//         new_path = [];
//         path.forEach((element, index) => {
//             if (Array.isArray(element[2 + i][0])) {
//                 new_path[index] = [element[0], element[1], element[2 + i][0]];
//             } else {
//                 new_path[index] = [element[0], element[1], element[2 + i]];
//             }
//         });
//         new_gimbal = '';
//         if (droneGimbal != '') {
//             if (droneGimbal[i] == '') {
//                 new_gimbal = 'None';
//             } else {
//                 new_gimbal = droneGimbal[i];
//                 if (isNaN(new_gimbal)) {
//                     new_gimbal = 'None';
//                 }
//             }
//         }
//         // sendMissionToAPI(new_path, drone_name, dronesSpeed[i], new_gimbal, repeatMission, mappingGrid, captureAndStoreImages, action, mission_type); // HERE
//         start_retrying(droneMissionIntent, selected_drones[i], speed[i], new_gimbal, repeatMission, retryCount, mappingGrid, storeImages, new_path);
//     }
// }
