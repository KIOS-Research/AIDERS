/*
 * Contains all of the operations that have to do with drone
 * (Adding drones to the platform, removing etc.
 * */

{
    let allDroneInfo = [];

    function add_Drone(droneID, dronePK) {
        allDroneInfo.push({
            droneURL: undefined,
            droneTelemetryURL: undefined,
            missionPointsURL: undefined,
            lastDetectionFrameURL: undefined,
            liveStreamHLSUrl: undefined,
            missionURL: undefined,
            droneLineData: [],
            droneInfo: {
                currentCoordinate: [],
                previousCoordinate: [],
                currentBatteryLevel: [],
                currentRosSecs: [],
            },
            droneObject: undefined,
            droneID: droneID,
            droneLineLayer: undefined,
            droneColumnLayer: undefined,
            droneColumnData: [],
            droneMissionIntent: MOVING,
            droneMissionState: '',
            previousdroneMissionState: '',
            droneMissionPath: [],
            droneMissionType: '',
            droneDetectedObject: undefined,
            droneModel: undefined,
            droneAvailability: AVAILABLE,
            droneCurrentMarkers: [],
            droneMarkerColour: getRandomColour(),
            droneMarkerCounter: 0,
            detVideoStarted: false,
            droneDetectionStatus: DETECTION_DISCONNECTED,
            droneDetectionConnectedOnce: false,
            droneBuildMapState: BUILD_MAP_INITIAL_STATE,
            drone_MS_BuildMapStatus: BUILD_MAP_INITIAL_STATE,
            droneMissionStartDateTime: '',
            dronePK: dronePK,
        });

        allDroneInfo = create_new_drone_model(allDroneInfo.length - 1);
        allDroneInfo = create_layers_for_new_drone(allDroneInfo, allDroneInfo.length - 1);

        add_layers_on_map(allDroneInfo, allDroneInfo.length - 1);
        return allDroneInfo;
    }

    function create_new_drone_model(index) {
        let droneID = allDroneInfo[index].droneID;
        droneID = droneID.replace(/\s/g, '');

        allDroneInfo[index].droneModel = {
            id: droneID + '_model',
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                var options = {
                    obj: dutils.urls.resolve('drone_glb_model'),
                    type: 'gltf',
                    scale: 15,
                    units: 'meters',
                    rotation: { x: 90, y: 0, z: 0 }, //default rotation,
                    adjustment: { x: 0.5, y: 0.5, z: 0 },
                };

                tb.loadObj(options, function (model) {
                    let coords = [33.4151176797, 35.1452954125];

                    allDroneInfo[index].droneObject = model.setCoords(coords);
                    allDroneInfo[index].droneObject.addEventListener('SelectedChange', onSelectedChange, false);
                    tb.add(allDroneInfo[index].droneObject);
                    model.addEventListener('ObjectDragged', onDraggedObject, false);
                });
            },
            render: function (gl, matrix) {
                tb.update();
            },
        };
        return allDroneInfo;
    }

    //actions to execute onDraggedObject
    function onDraggedObject(e) {
        let draggedObject = e.detail.draggedObject;
        let draggedAction = e.detail.draggedAction;
    }

    /*Hides the disconnected/deleted drones*/
    function hide_drones_from_map(deleted_drones) {
        for (let i = 0; i < deleted_drones.length; i++) {
            let droneID = deleted_drones[i];
            let droneExists = false;
            let index = -1;
            for (let j = 0; j < allDroneInfo.length; j++) {
                if (allDroneInfo[j].droneID === droneID) {
                    droneExists = true;
                    index = j;
                    break;
                }
            }

            if (droneExists) {
                allDroneInfo[index].droneObject.visibility = false;
                allDroneInfo[index].droneAvailability = NOT_AVAILABLE;
            }
        }
    }

    function delete_drones_from_array() {
        for (let i = 0; i < allDroneInfo.length; i++) {
            allDroneInfo[i].droneLineData = [];
            tb.remove(allDroneInfo[i].droneObject);
            map.removeLayer(allDroneInfo[i].droneLineLayer.id);
            map.removeLayer(allDroneInfo[i].droneModel.id);
        }

        allDroneInfo = [];

        return allDroneInfo;
    }

    /*Creates an object for each of the newly added drones, then inserts it to the drones array, and at the end it creates the necessary layers and drone 3d models for it*/
    function add_new_drones_to_array(new_drone_names, all_drone_names_and_ids) {
        let isEMpty = false;
        //Get the primary key of each new drone name
        let new_drone_pks = [];
        all_drone_names_and_ids.forEach(function (drone) {
            if (new_drone_names.includes(drone.drone_name)) {
                new_drone_pks.push(drone.pk);
            }
        });

        if (allDroneInfo.length === 0) {
            isEMpty = true;
            for (let i = 0; i < new_drone_names.length; i++) {
                allDroneInfo = add_Drone(new_drone_names[i], new_drone_pks[i]);
            }
        }

        if (isEMpty) {
            return allDroneInfo;
        }

        for (let i = 0; i < new_drone_names.length; i++) {
            let droneID = new_drone_names[i];
            let droneExists = false;
            let index = -1;
            for (let j = 0; j < allDroneInfo.length; j++) {
                if (allDroneInfo[j].droneID === droneID) {
                    droneExists = true;
                    index = j;
                    break;
                }
            }

            if (droneExists) {
                allDroneInfo[index].droneObject.visibility = true;
                allDroneInfo[index].droneAvailability = AVAILABLE;
            } else {
                add_Drone(droneID, new_drone_pks[i]);
            }
        }
        return allDroneInfo;
    }

    /*Returns the last value of altitude from the drones array*/
    function getLastAltitude(selectedDrone_id) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneID === selectedDrone_id) {
                if (allDroneInfo[i].droneInfo.currentCoordinate[2] === undefined) {
                    return DEFAULT_ALTITUDE;
                }
                return allDroneInfo[i].droneInfo.currentCoordinate[2];
            }
        }
    }

    function updateDroneTooltip(drone) {
        let droneID = drone.droneID;
        let dronedata = drone.droneInfo;
        // console.log("DRONE ID: " + dronedata.drondroneID)
        let lon = dronedata.currentCoordinate[0];
        let lat = dronedata.currentCoordinate[1];
        let alt = dronedata.altitude;
        if (alt === undefined) alt = 0;
        let velocity = dronedata.velocity;
        let heading = dronedata.heading;
        let updatedTooltip = `<strong>${droneID}
                     <br>AltFromHome: ${alt} m                 
                     <br>Heading: ${heading.toFixed(6)} °                 
                     <br>Velocity: ${velocity.toFixed(6)} m/s                
                      <br>Lon: ${lon.toFixed(6)} °   
                     <br>Lat: ${lat.toFixed(6)} °                 
           </strong>`;
        drone.droneObject.tooltip.element.innerHTML =
            '' +
            '<div class="marker maplibregl-popup-anchor-bottom" style="width: fit-content;">' +
            '<div class="maplibregl-popup-tip" style="width: fit-content;" ></div>' +
            '<div class="maplibregl-popup-content" ' +
            `<strong> ${updatedTooltip}</strong></div></div>`;

        // drone.droneObject.addTooltip(`<strong>${dronedata.droneID}
        //              <br>AltFromHome: ${dronedata.altitude} m
        //              <br>Heading: ${dronedata.heading.toFixed(6)} °
        //              <br>Velocity: ${dronedata.velocity.toFixed(6)} m/s
        //               <br>Lon: ${lon.toFixed(6)} °
        //              <br>Lat: ${lat.toFixed(6)} °
        //    </strong>`,true)
    }

    /*
     * Adds a tooltip to the 3d models
     * */
    function addTooltipOnDrones(new_drone_ids) {
        for (var i = 0; i < allDroneInfo.length; i++) {
            for (var j = 0; j < new_drone_ids.length; j++) {
                if (allDroneInfo[i].droneID === new_drone_ids[j]) {
                    allDroneInfo[i].droneObject.addTooltip(allDroneInfo[i].droneID, true);
                }
            }
        }
        return allDroneInfo;
    }

    /*Reads the content of the URL that contains a csv file with all the drone ids*/
    function get_drone_ids() {
        return $.ajax({
            url: BASE_URL_DRONE_IDS_API,
            success: function (droneIDs) {
                return droneIDs;
            },
        });
    }

    function get_all_drone_info_array() {
        return allDroneInfo;
    }

    function get_drone_object(droneID) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (droneID === allDroneInfo[i].droneID) {
                return allDroneInfo[i];
            }
        }

        return -1;
    }

    function is_query_contains_drone_id(droneID) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneID.includes(droneID)) {
                return allDroneInfo[i];
            }
        }

        return -1;
    }

    function get_selected_drone_id() {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                return allDroneInfo[i].droneID;
            }
        }

        return NOT_FOUND_STRING;
    }

    function is_drone_selected() {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                return true;
            }
        }

        return false;
    }

    function get_selected_drone_info(selectedObject) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (selectedObject.uuid === allDroneInfo[i].droneObject.uuid) {
                return allDroneInfo[i];
            }
        }
        return 'NOT FOUND';
    }

    function push_marker_to_selected_drone(marker, selectedObject = undefined, droneID = undefined) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (typeof selectedObject !== 'undefined' && typeof droneID === 'undefined') {
                if (selectedObject.uuid === allDroneInfo[i].droneObject.uuid) {
                    allDroneInfo[i].droneCurrentMarkers.push(marker);
                }
            } else if (typeof selectedObject === 'undefined' && typeof droneID !== 'undefined') {
                if (droneID === allDroneInfo[i].droneID) {
                    allDroneInfo[i].droneCurrentMarkers.push(marker);
                }
            }
        }
        return allDroneInfo;
    }

    function getDroneCurrentMarkersOnMap(drone_id) {
        let index = get_drone_index(drone_id);
        return allDroneInfo[index].droneCurrentMarkers.length;
    }

    function remove_drone_markers(droneID) {
        let drone_index = get_drone_index(droneID);
        for (let j = allDroneInfo[drone_index].droneCurrentMarkers.length - 1; j >= 0; j--) {
            allDroneInfo[drone_index].droneCurrentMarkers[j].remove();
        }
        allDroneInfo[drone_index].droneCurrentMarkers = [];
        emptyMissionPath(droneID);
        return allDroneInfo;
    }

    function remove_markers_from_selected_drones(drones) {
        drones.forEach(function (drone) {
            remove_drone_markers(drone.droneID);
        });
    }
    /*
     * If there is any drone selected, it removes the selection box
     * */
    function remove_drone_boxes() {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                allDroneInfo[i].droneObject.selected = false;
                return;
            }
        }
    }

    function get_selected_drone_marker_color(droneID = undefined) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (typeof droneID !== 'undefined' && droneID === allDroneInfo[i].dronePK) {
                return allDroneInfo[i].droneMarkerColour;
            } else if (typeof allDroneInfo[i].droneObject === 'undefined') {
                continue; //Probably we got here after refresh and trying to place back the map waypoints. Objects are not created yet
            }
            // else if (allDroneInfo[i].droneObject.selected)
            else {
                return allDroneInfo[get_drone_index(droneID)].droneMarkerColour;
            }
        }

        return DEFAULT_COLOR;
    }

    function unselect_toggled_drone(droneIndex) {
        allDroneInfo[droneIndex].droneObject.selected = false;
    }

    function select_toggled_drone(droneIndex) {
        allDroneInfo[droneIndex].droneObject.selected = true;
    }

    function get_selected_drones() {
        let selected_drones = [];
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                selected_drones.push(allDroneInfo[i]);
            }
        }
        return selected_drones;
    }

    function get_selected_drone_ids() {
        let selected_drone_ids = [];
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                selected_drone_ids.push(allDroneInfo[i].droneID);
            }
        }
        return selected_drone_ids;
    }
    function get_drone_index(droneID) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (droneID === allDroneInfo[i].dronePK) {
                return i;
            }
            if (droneID === allDroneInfo[i].droneID) {
                return i;
            }
        }
        return -1;
    }

    function update_drone_detection_status_locally(index, status) {
        allDroneInfo[index].droneDetectionStatus = status;
    }

    function update_all_drone_array(updatedArray) {
        allDroneInfo = updatedArray;
    }

    function updateBuildMapStatusLocally(drone_id, buildMapStatus) {
        let index = get_drone_index(drone_id);
        allDroneInfo[index].droneBuildMapState = buildMapStatus;
    }

    function update_MS_BuildMapStatusLocally(drone_id, buildMapStatus) {
        let index = get_drone_index(drone_id);
        allDroneInfo[index].drone_MS_BuildMapStatus = buildMapStatus;
    }
    function getLocalBuildMapState(drone_id) {
        let index = get_drone_index(drone_id);
        return allDroneInfo[index].droneBuildMapState;
    }

    function getLocal_MS_BuildMapStatus(drone_id) {
        let index = get_drone_index(drone_id);
        return allDroneInfo[index].drone_MS_BuildMapStatus;
    }

    function updateDroneMissionIntentLocally(drone_index, droneStats) {
        allDroneInfo[drone_index].droneMissionIntent = droneStats;
    }

    function updateSelectedDroneMissionIntentsLocally(selected_drones, droneStats) {
        selected_drones.forEach(function (drone, i) {
            let indx = get_drone_index(drone.droneID);
            allDroneInfo[indx].droneMissionIntent = droneStats;
        });
    }

    /*
     * Callback method that is activated once any 3D (drone) object is clicked
     * It's a handler for the object selection where we can get if the drone object is selected or not
     * */
    function onSelectedChange(eventArgs) {
        let selectedObject = eventArgs.detail; //we get the object selected/unselected
        at_least_one_drone_selected = get_selected_drones().length > 0;

        let selected_drone = get_selected_drone_info(selectedObject);
        let toggleID = 'drone-toggle' + selected_drone.droneID;
        let toggle = document.getElementById(toggleID);

        if (selected_drone.droneObject.selected) {
            // if drone selected
            $(toggle).bootstrapToggle('on');
        } else {
            $(toggle).bootstrapToggle('off');
        }

        latest_drone_selected_model = selectedObject;
        updateMissionButtonStates(selectedObject);

        // updateBuildMapButtonStates(selected_drone, selected_drone.droneObject.selected)
        // update_MS_BuildMapButtonStates(selected_drone, selected_drone.droneObject.selected)
    }

    function update_drones_local_availability(removed_drone_ids, availability) {
        for (let i = 0; i < removed_drone_ids.length; i++) {
            for (let j = 0; j < allDroneInfo.length; j++) {
                if (removed_drone_ids[i] === allDroneInfo[j].droneID) {
                    allDroneInfo[j].droneAvailability = availability;
                }
            }
        }
    }

    function get_drone_availability_status(drone_id) {
        let index = get_drone_index(drone_id);
        return allDroneInfo[index].droneAvailability;
    }

    function reset_marker_counter(droneID) {
        let droneIndex = get_drone_index(droneID);
        allDroneInfo[droneIndex].droneMarkerCounter = 0;
    }

    function reset_marker_counter_for_selected_drones(drones) {
        drones.forEach(function (drone) {
            reset_marker_counter(drone.droneID);
        });
    }
    function increase_waypoint_marker_number(droneID) {
        let droneIndex = get_drone_index(droneID);

        allDroneInfo[droneIndex].droneMarkerCounter++;
    }

    function pushLocToMissionPath(locArray, droneID) {
        let droneIndex = get_drone_index(droneID);

        allDroneInfo[droneIndex].droneMissionPath.push(locArray);
    }

    function setDroneMissionPath(droneID, missionPath) {
        let droneIndex = get_drone_index(droneID);

        allDroneInfo[droneIndex].droneMissionPath = missionPath;
    }

    function getMissionPath(droneID) {
        let droneIndex = get_drone_index(droneID);
        return allDroneInfo[droneIndex].droneMissionPath;
    }

    function getMissionType(droneID) {
        let droneIndex = get_drone_index(droneID);
        return allDroneInfo[droneIndex].droneMissionType;
    }
    function emptyMissionPath(droneID) {
        let droneIndex = get_drone_index(droneID);
        allDroneInfo[droneIndex].droneMissionPath = [];
    }

    function emptyMissionPathForSelectedDrones(drones) {
        drones.forEach(function (drone) {
            emptyMissionPath(drone.droneID);
        });
    }
    function insertHeightToMissionPath(height, droneID) {
        let droneIndex = get_drone_index(droneID);

        if (allDroneInfo[droneIndex].droneMissionPath[0].length === 2) {
            //If we don't have height, add it.
            for (let i = 0; i < allDroneInfo[droneIndex].droneMissionPath.length; i++) {
                allDroneInfo[droneIndex].droneMissionPath[i].push(height);
            }
        }

        // console.log("UPDATED MISSION PATH: ")
        // console.log(missionPath)
        return allDroneInfo[droneIndex].droneMissionPath;
    }

    function instertHeightToMissionPathForSelectedDrones(drones, alts) {
        for (let i = 0; i < drones.length; i++) {
            console.log(alts[i], drones[i].droneID);
            insertHeightToMissionPath(alts[i], drones[i].droneID);
        }
    }

    function updateDroneMissionPath(droneID, path) {
        let indx = get_drone_index(droneID);
        allDroneInfo[indx].droneMissionPath = path;
    }
    function updateDroneMissionType(droneID, missionType) {
        let indx = get_drone_index(droneID);
        allDroneInfo[indx].droneMissionType = missionType;
    }

    function updateSelectedDronesMissionPath(drones, paths) {
        for (let i = 0; i < drones.length; i++) {
            updateDroneMissionPath(drones[i].droneID, paths[i]['path']);
        }
    }
    function updateSelectedDronesMissionType(drones, missionType) {
        for (let i = 0; i < drones.length; i++) {
            updateDroneMissionType(drones[i].droneID, missionType);
        }
    }

    function place_waypoints_on_map(waypoints, droneID) {
        let allDrones = get_all_drone_info_array();
        for (let i = 0; i < waypoints.length; i++) {
            let mp = create_marker_point(allDrones, droneID);
            let marker = new maplibregl.Marker(mp).setLngLat(waypoints[i]).addTo(map);
            increase_waypoint_marker_number(droneID);

            push_marker_to_selected_drone(marker, undefined, droneID);
            let location = [waypoints[i][0], waypoints[i][1]];
            pushLocToMissionPath(location, droneID);
        }
    }

    function get_current_alts(drones) {
        let alts = [];
        drones.forEach(function (drone) {
            alts.push(drone.droneInfo.currentCoordinate[2]);
        });
        return alts;
    }

    function isHeightOK(heights, limit) {
        for (let i = 0; i < heights.length; i++) {
            let n1 = parseInt(heights[i]);
            let n2 = parseInt(limit);
            if (n1 < n2) {
                return false;
            }
        }
        return true;
    }

    function updateDroneMissionDateTimeLocally(droneID, dt) {
        let indx = get_drone_index(droneID);
        allDroneInfo[indx].droneMissionStartDateTime = dt;
    }
}
