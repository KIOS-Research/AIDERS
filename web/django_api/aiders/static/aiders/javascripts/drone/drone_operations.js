
/* 
    NOT CALLED 
*/

// function delete_drones_from_array() {
//     for (let i = 0; i < allDroneInfo.length; i++) {
//         allDroneInfo[i].droneLineData = [];
//         tb.remove(allDroneInfo[i].droneObject);
//         map.removeLayer(allDroneInfo[i].droneLineLayer.id);
//         map.removeLayer(allDroneInfo[i].droneModel.id);
//     }
//     allDroneInfo = [];
//     return allDroneInfo;
// }


// function getDroneCurrentMarkersOnMap(drone_id) {
//     let index = get_drone_index(drone_id);
//     return allDroneInfo[index].droneCurrentMarkers.length;
// }


// /*
//  * If there is any drone selected, it removes the selection box
//  * */
// function remove_drone_boxes() {
//     for (let i = 0; i < allDroneInfo.length; i++) {
//         if (allDroneInfo[i].droneObject.selected) {
//             allDroneInfo[i].droneObject.selected = false;
//             return;
//         }
//     }
// }



/*
 * Contains all of the operations that have to do with drone
 * (Adding drones to the platform, removing etc.
 * */

{
    let allDroneInfo = [];


    function get_all_drone_info_array() {
        return allDroneInfo;
    }

    function update_all_drone_array(updatedArray) {
        allDroneInfo = updatedArray;
    }


    function get_drone_index(droneID) {
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (droneID === allDroneInfo[i].droneID) {
                return i;
            }
        }
        return -1;
    }


    function get_drone_object(droneID) {
        let i = get_drone_index(droneID);
        if(i >= 0) {
            return allDroneInfo[i];
        }
        return -1;
    }


    function getDroneAttributeValue(_droneName, _attrName) {
        let index = get_drone_index(_droneName);
        return allDroneInfo[index][_attrName];
    }
    function getDroneAttributeValueByIndex(_droneIndex, _attrName) {
        return allDroneInfo[_droneIndex][_attrName];
    }


    function setDroneAttributeValue(_droneName, _attrName, _value) {
        // console.log("setDroneAttributeValue " + _attrName + " " + _value);
        let index = get_drone_index(_droneName);
        allDroneInfo[index][_attrName] = _value;
    }
    function setDroneAttributeValueByIndex(_droneIndex, _attrName, _value) {
        allDroneInfo[_droneIndex][_attrName] = _value;
    }


    function add_Drone(droneID, dronePK, droneType) {
        allDroneInfo.push({
            droneURL: dutils.urls.resolve('drone', { operation_name: CURRENT_OP, drone_name: droneID }),
            missionPointsURL: dutils.urls.resolve('mission_points', { operation_name: CURRENT_OP, drone_name: droneID }),
            missionURL: dutils.urls.resolve('mission', { operation_name: CURRENT_OP, drone_name: droneID }),
            droneLineData: [],
            droneInfo: {
                currentCoordinate: [],
                previousCoordinate: [],
                currentBatteryLevel: [],
                satellites: [],
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
            droneType: droneType,
            vtolState: undefined
        });

        allDroneInfo = create_new_drone_model(allDroneInfo.length - 1);
        allDroneInfo = create_layers_for_new_drone(allDroneInfo, allDroneInfo.length - 1);

        add_layers_on_map(allDroneInfo, allDroneInfo.length - 1);
        return allDroneInfo;
    }








    function create_new_drone_model(index) {
        console.log("create_new_drone_model");
        let { droneID, droneType } = allDroneInfo[index];
        droneID = droneID.replace(/\s/g, '');

        let droneModel = "drone_glb_model";
        let initialRotationY = 0;
        let modelScale = 15;
        
        if (droneType == "MAVLINK") {
            droneModel = "mavlink_glb_model";
            initialRotationY = -90;
            modelScale = 90;
        }

        allDroneInfo[index].droneModel = {
            id: droneID + '_model',
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                var options = {
                    obj: dutils.urls.resolve(droneModel),
                    type: 'gltf',
                    scale: modelScale,
                    units: 'meters',
                    rotation: { x: 90, y: initialRotationY, z: 0 }, //default rotation,
                    adjustment: { x: 0.5, y: 0.5, z: 0 },
                };

                tb.loadObj(options, function (model) {
                    allDroneInfo[index].droneObject = model.setCoords([0, 0]);
                    allDroneInfo[index].droneObject.addEventListener('SelectedChange', onSelectedDroneChange, false);
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
        console.log("onDraggedObject");
        let draggedObject = e.detail.draggedObject;
        let draggedAction = e.detail.draggedAction;
    }

    /*Hides the disconnected/deleted drones*/
    function hide_drones_from_map(deleted_drones) {
        console.log("hide_drones_from_map");
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



    /*Creates an object for each of the newly added drones, then inserts it to the drones array, and at the end it creates the necessary layers and drone 3d models for it*/
    function add_new_drones_to_array(new_drone_names, all_drones_core_info) {
        console.log("add_new_drones_to_array");
        let isEMpty = false;
        //Get the primary key of each new drone name
        let new_drone_pks = [];
        let new_drone_types = [];
        all_drones_core_info.forEach(function (drone) {
            if (new_drone_names.includes(drone.drone_name)) {
                new_drone_pks.push(drone.drone_pk);
                new_drone_types.push(drone.drone_type);
            }
        });

        if (allDroneInfo.length === 0) {
            isEMpty = true;
            for (let i = 0; i < new_drone_names.length; i++) {
                allDroneInfo = add_Drone(new_drone_names[i], new_drone_pks[i], new_drone_types[i]);
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
                add_Drone(droneID, new_drone_pks[i], new_drone_types[i]);
            }
        }
        return allDroneInfo;
    }



    /*
     * Adds a tooltip to the 3d models
     * */
    function addTooltipOnDrones(new_drone_ids) {
        console.log("addTooltipOnDrones");
        for (var i = 0; i < allDroneInfo.length; i++) {
            for (var j = 0; j < new_drone_ids.length; j++) {
                if (allDroneInfo[i].droneID === new_drone_ids[j]) {
                    allDroneInfo[i].droneObject.addTooltip(allDroneInfo[i].droneID, true, 5);
                }
            }
        }
        return allDroneInfo;
    }





    function is_query_contains_drone_id(droneID) {
        console.log("is_query_contains_drone_id");
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneID.includes(droneID)) {
                return allDroneInfo[i];
            }
        }
        return -1;
    }

    function get_selected_drone_id() {
        console.log("get_selected_drone_id");
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                return allDroneInfo[i].droneID;
            }
        }

        return NOT_FOUND_STRING;
    }


    function get_selected_drone_info(selectedObject) {
        console.log("get_selected_drone_info");
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (selectedObject.uuid === allDroneInfo[i].droneObject.uuid) {
                return allDroneInfo[i];
            }
        }
        return 'NOT FOUND';
    }

    function push_marker_to_selected_drone(marker, selectedObject = undefined, droneID = undefined) {
        console.log("push_marker_to_selected_drone");
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


    function remove_drone_markers(droneID) {
        console.log("remove_drone_markers");
        let drone_index = get_drone_index(droneID);
        for (let j = allDroneInfo[drone_index].droneCurrentMarkers.length - 1; j >= 0; j--) {
            allDroneInfo[drone_index].droneCurrentMarkers[j].remove();
        }
        allDroneInfo[drone_index].droneCurrentMarkers = [];
        allDroneInfo[drone_index].droneMissionPath = [];
        return allDroneInfo;
    }

    function remove_markers_from_selected_drones(drones) {
        console.log("remove_markers_from_selected_drones");
        drones.forEach(function (drone) {
            remove_drone_markers(drone.droneID);
        });
    }



    function get_selected_drone_marker_color(droneID = undefined) {
        console.log("get_selected_drone_marker_color");
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (typeof droneID !== 'undefined' && droneID === allDroneInfo[i].droneID) {
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
        console.log("unselect_toggled_drone");
        allDroneInfo[droneIndex].droneObject.selected = false;
    }

    function select_toggled_drone(droneIndex) {
        console.log("select_toggled_drone");
        allDroneInfo[droneIndex].droneObject.selected = true;
    }

    function get_selected_drones() {
        // console.log("get_selected_drones");
        let selected_drones = [];
        for (let i = 0; i < allDroneInfo.length; i++) {
            try {
                if (allDroneInfo[i].droneObject.selected) {
                    selected_drones.push(allDroneInfo[i]);
                }
            }
            catch (err) { }
        }
        return selected_drones;
    }

    function get_selected_drone_ids() {
        console.log("get_selected_drone_ids");
        let selected_drone_ids = [];
        for (let i = 0; i < allDroneInfo.length; i++) {
            if (allDroneInfo[i].droneObject.selected) {
                selected_drone_ids.push(allDroneInfo[i].droneID);
            }
        }
        return selected_drone_ids;
    }


    function updateSelectedDroneMissionIntentsLocally(selected_drones, droneStats) {
        selected_drones.forEach(function (drone, i) {
            setDroneAttributeValue(drone.droneID, "droneMissionIntent", droneStats)
        });
    }

    /*
     * Callback method that is activated once any 3D (drone) object is clicked
     * It's a handler for the object selection where we can get if the drone object is selected or not
     * */
    function onSelectedDroneChange(eventArgs) {
        console.log("onSelectedDroneChange");
        let selectedObject = eventArgs.detail; //we get the object selected/unselected
        at_least_one_drone_selected = get_selected_drones().length > 0;

        let selected_drone = get_selected_drone_info(selectedObject);
        let toggleID = 'drone-select-toggle-' + selected_drone.droneID;
        let toggle = document.getElementById(toggleID);

        if (selected_drone.droneObject.selected) {
            // if drone selected
            $(toggle).bootstrapToggle('on');
        } else {
            $(toggle).bootstrapToggle('off');
        }

        latest_drone_selected_model = selectedObject;
        updateMissionButtonStates(selectedObject);

    }

    function update_drones_local_availability(removed_drone_ids, availability) {
        console.log("update_drones_local_availability");
        for (let i = 0; i < removed_drone_ids.length; i++) {
            for (let j = 0; j < allDroneInfo.length; j++) {
                if (removed_drone_ids[i] === allDroneInfo[j].droneID) {
                    allDroneInfo[j].droneAvailability = availability;
                }
            }
        }
    }





    function reset_marker_counter_for_selected_drones(drones) {
        console.log("reset_marker_counter_for_selected_drones");
        drones.forEach(function (drone) {
            setDroneAttributeValue(drone.droneID, "droneMarkerCounter", 0);
        });
    }

    function increase_waypoint_marker_number(droneID) {
        console.log("increase_waypoint_marker_number");
        let droneIndex = get_drone_index(droneID);
        allDroneInfo[droneIndex].droneMarkerCounter++;
    }

    function pushLocToMissionPath(locArray, droneID) {
        console.log("pushLocToMissionPath");
        let droneIndex = get_drone_index(droneID);
        allDroneInfo[droneIndex].droneMissionPath.push(locArray);
    }

    
    function emptyMissionPathForSelectedDrones(drones) {
        console.log("emptyMissionPathForSelectedDrones");
        drones.forEach(function (drone) {
            setDroneAttributeValue(drone.droneID, "droneMissionPath", []);
        });
    }

    function insertHeightToMissionPath(height, droneID) {
        console.log("insertHeightToMissionPath");
        let droneIndex = get_drone_index(droneID);
        allDroneInfo[droneIndex].droneMissionPath.forEach((element) => {
            element[2] = height;
        });
    }

    function insertHeightToMissionPathForSelectedDrones(drones, alts) {
        console.log("insertHeightToMissionPathForSelectedDrones");
        for (let i = 0; i < drones.length; i++) {
            insertHeightToMissionPath(alts[i], drones[i].droneID);
        }
    }

    function updateSelectedDronesMissionPath(drones, paths) {
        console.log("updateSelectedDronesMissionPath");
        for (let i = 0; i < drones.length; i++) {
            setDroneAttributeValue(drones[i].droneID, "droneMissionPath", paths[i]['path']);
        }
    }
    
    function updateSelectedDronesMissionType(drones, missionType) {
        console.log("updateSelectedDronesMissionType");
        for (let i = 0; i < drones.length; i++) {
            setDroneAttributeValue(drones[i].droneID, "droneMissionType", missionType);
        }
    }

    function place_waypoints_on_map(waypoints, droneID) {
        console.log("place_waypoints_on_map");
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
        console.log("get_current_alts");
        let alts = [];
        drones.forEach(function (drone) {
            alts.push(drone.droneInfo.currentCoordinate[2]);
        });
        return alts;
    }

    function isHeightOK(heights, limit) {
        console.log("isHeightOK");
        for (let i = 0; i < heights.length; i++) {
            let n1 = parseInt(heights[i]);
            let n2 = parseInt(limit);
            if (n1 < n2) {
                return false;
            }
        }
        return true;
    }


    function updateAllDroneModelsToStayAbove(){
        get_all_drone_info_array().forEach((drone) => {
            map.moveLayer(drone.droneLineLayer.id);
            map.moveLayer(drone.droneModel.id);
        });
    
    }
}
