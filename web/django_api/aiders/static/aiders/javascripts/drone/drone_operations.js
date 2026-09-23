
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
    // let allDroneInfo = [];
    let fovLineLayers = {};

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
        if (i >= 0) {
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


    function add_Drone(droneID, dronePK, droneType, droneConfiguration) {
        var droneColor = getRandomColour();
        allDroneInfo.push({
            droneURL: dutils.urls.resolve('drone', { operation_name: CURRENT_OP, drone_name: droneID }),
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
            droneMissionState: '',
            previousdroneMissionState: '',
            droneMissionPath: [],
            droneMissionType: '',
            droneDetectedObject: undefined,
            droneModel: undefined,
            droneCurrentMarkers: [],
            droneMarkerColour: droneColor,
            droneMarkerCounter: 0,
            droneHomeMarker: null,
            detVideoStarted: false,
            droneDetectionStatus: DETECTION_DISCONNECTED,
            droneDetectionConnectedOnce: false,
            droneMissionStartDateTime: '',
            dronePK: dronePK,
            droneType: droneType,
            droneConfiguration: droneConfiguration,
            vtolState: undefined,
            isConnected: true,
        });

        allDroneInfo = create_new_drone_model(allDroneInfo.length - 1);
        allDroneInfo = create_layers_for_new_drone(allDroneInfo, allDroneInfo.length - 1, droneColor);

        add_layers_on_map(allDroneInfo, allDroneInfo.length - 1);

    }


    function handleConnectedDrones(_wsDronesList) {
        _wsDronesList.forEach((wsDrone) => {
            let found = false;
            // Loop through allDroneInfo
            allDroneInfo.forEach((drone) => {
                if (wsDrone.drone_name === drone.droneID) {
                    if(drone.droneObject.visibility === false) {
                        drone.droneObject.visibility = true;
                        drone.isConnected = true;
                        createDroneFieldOfViewPolygons(wsDrone.drone_name);
                        addNewDronesToSidePanel(wsDrone.drone_name);
                        create_video_elements(wsDrone.drone_name);
                        create_det_video_elements(wsDrone.drone_name);
                        createDroneTelemetryElement(wsDrone.drone_name);
                        checkAndAddMoreFeatureToPanel(wsDrone.drone_name, wsDrone.weather_station_available, wsDrone.lidar_available, wsDrone.water_sampler_available);
                        createDroneHomeLocationMarker(wsDrone.drone_name, wsDrone.telemetry.homeLat, wsDrone.telemetry.homeLon);
                    }
                    found = true;
                }
            });
            if (!found) {
                add_Drone(wsDrone.drone_name, wsDrone.id, wsDrone.type, wsDrone.configuration);
                createDroneFieldOfViewPolygons(wsDrone.drone_name);
                addNewDronesToSidePanel(wsDrone.drone_name);
                create_video_elements(wsDrone.drone_name);
                create_det_video_elements(wsDrone.drone_name);
                createDroneTelemetryElement(wsDrone.drone_name);
                checkAndAddMoreFeatureToPanel(wsDrone.drone_name, wsDrone.weather_station_available, wsDrone.lidar_available, wsDrone.water_sampler_available);
                createDroneHomeLocationMarker(wsDrone.drone_name, wsDrone.telemetry.homeLat, wsDrone.telemetry.homeLon);
            }
        });

        // Return the updated allDroneInfo array
        return allDroneInfo;
    }

    function handleDisconnectedDrones(_currentActiveDrones) {
        // Create a set of drone names from the dronesToUpdate array for faster lookup
        const updatedDroneNames = new Set(_currentActiveDrones.map(drone => drone.drone_name));
        allDroneInfo.forEach(drone => {
            // Check if the current drone is not in the updatedDroneNames set
            if (!updatedDroneNames.has(drone.droneID)) {
                // If the drone does not exist in dronesToUpdate, update its visibility and availability
                drone.droneObject.visibility = false;
                drone.droneObject.selected = false;
                drone.isConnected = false;
                removeDroneFieldOfViewPolygons(drone.droneID);
                removeDronesFromSidePanel(drone.droneID);
                remove_video_element(drone.droneID);
                remove_det_video_elements(drone.droneID)
                deleteDroneTelemetryElement(drone.droneID)
                removeDroneWaypoints(drone)
                removeDroneMoreFeatureToPanel(drone.droneID)
                // Remove home marker if it exists
                if (drone.droneHomeMarker) {
                    drone.droneHomeMarker.remove();
                    drone.droneHomeMarker = null;
                }
            }
        });
        // Return the updated allDroneInfo array
        return allDroneInfo;
    }

    function handleUpdatedDrones(_wsDronesList) {
        _wsDronesList.forEach((wsDrone) => {
            allDroneInfo.forEach((drone) => {
                if (drone.droneID === wsDrone.drone_name && drone.droneObject !== undefined) {
                    let wsDroneTelemetry = wsDrone.telemetry;
                    updateDroneModelAndPathOnMap(
                        drone,
                        wsDroneTelemetry.lat,
                        wsDroneTelemetry.lon,
                        wsDroneTelemetry.alt,
                        wsDroneTelemetry.heading,
                        wsDroneTelemetry.velocity,
                    );
                    updateDroneFieldOfViewOnMap(
                        drone,
                        wsDroneTelemetry.fov_coordinates,
                        wsDroneTelemetry.lat,
                        wsDroneTelemetry.lon,
                        wsDroneTelemetry.alt,
                        wsDroneTelemetry.gimbal_angle
                    );
                    updateDroneTelemetryElement(
                        wsDrone.drone_name,
                        wsDroneTelemetry.time,
                        wsDroneTelemetry.drone_state,
                        wsDroneTelemetry.battery_percentage,
                        wsDroneTelemetry.satellites,
                        wsDroneTelemetry.lat,
                        wsDroneTelemetry.lon,
                        wsDroneTelemetry.alt,
                        wsDroneTelemetry.velocity,
                        wsDroneTelemetry.heading,
                        wsDroneTelemetry.gimbal_angle,
                        drone.droneType,
                        drone.droneConfiguration,
                        wsDroneTelemetry.vtol_state,
                        wsDroneTelemetry.gps_signal,
                        wsDroneTelemetry.crps_requested,
                        wsDroneTelemetry.crps_responding,
                    );
                    updateDroneLeftPanelData(
                        wsDrone.drone_name,
                        drone.droneType,
                        drone.droneConfiguration,
                        wsDroneTelemetry.alt,
                        wsDroneTelemetry.battery_percentage,
                        wsDroneTelemetry.drone_state,
                        wsDroneTelemetry.vtol_state,
                        wsDroneTelemetry.crps_requested,
                        wsDroneTelemetry.crps_responding,
                    );
                    updateDroneStateAndMission(drone, wsDrone.mission_id, wsDroneTelemetry.drone_state)
                    updateDetectionState(wsDrone.drone_name, wsDrone.detection_active)
                    checkIfDroneBuildMapIsActivatedOrDeactivated(drone, wsDrone.build_map_activated)

                    // Update the home marker
                    if(drone.droneObject.selected) {
                        updateDroneHomeLocationMarker(drone.droneID, wsDroneTelemetry.homeLat, wsDroneTelemetry.homeLon);
                    }
                    else {
                        // make the home marker not visible for this drone
                        if (drone.droneHomeMarker) {
                            drone.droneHomeMarker.getElement().style.display = 'none';
                            // hide the marker's popup
                            if (drone.droneHomeMarker.getPopup()) {
                                drone.droneHomeMarker.getPopup().remove();
                            }
                        }
                    }
                }
            });
        });
        updateMissionButtonStates()
        checkForDroneWarnings(allDroneInfo);
    }



    function createDroneHomeLocationMarker(_droneName, _homeLat, _homeLon) {
        // Check if coordinates are valid
        if (!_homeLat || !_homeLon || isNaN(_homeLat) || isNaN(_homeLon)) {
            console.log("Invalid home location coordinates for drone:", _droneName);
            return;
        }

        let droneIndex = get_drone_index(_droneName);
        if (droneIndex === -1) {
            console.log("Drone not found:", _droneName);
            return;
        }

        let drone = allDroneInfo[droneIndex];
        
        // Create a DOM element for the home marker
        const homeMarkerElement = document.createElement('div');
        homeMarkerElement.className = 'marker home-marker';
        homeMarkerElement.style.backgroundColor = drone.droneMarkerColour;
        homeMarkerElement.style.border = '2px solid #fff';
        homeMarkerElement.style.borderRadius = '50%';
        homeMarkerElement.style.width = '22px';
        homeMarkerElement.style.height = '22px';
        homeMarkerElement.style.display = 'flex';
        homeMarkerElement.style.alignItems = 'center';
        homeMarkerElement.style.justifyContent = 'center';
        homeMarkerElement.style.fontSize = '12px';
        homeMarkerElement.style.fontWeight = 'bold';
        homeMarkerElement.style.color = '#fff';
        homeMarkerElement.innerHTML = 'H';
        homeMarkerElement.style.cursor = 'pointer';

        // Create popup content with drone name and coordinates
        const popupContent = `
            <div style="text-align: center; padding: 5px;">
                <strong>${_droneName}</strong>
                <br />
                <strong><i class="fa fa-house" style="color: ${drone.droneMarkerColour};"></i> Home Location</strong>
                <br>
                <span style="font-size: 12px;">
                    Lat: ${_homeLat.toFixed(8)}°<br>
                    Lon: ${_homeLon.toFixed(8)}°
                </span>
            </div>
        `;

        // Create the popup
        const homePopup = new maplibregl.Popup({ 
            offset: 10,
            maxWidth: '200px'
        }).setHTML(popupContent);

        // Create and add the marker to the map
        drone.droneHomeMarker = new maplibregl.Marker(homeMarkerElement)
            .setLngLat([_homeLon, _homeLat])
            .setPopup(homePopup)
            .addTo(map);

        console.log(`Home location marker created for drone ${_droneName} at ${_homeLat}, ${_homeLon}`);
    }


    function updateDroneHomeLocationMarker(_droneName, _homeLat, _homeLon) {
        // Check if coordinates are valid
        if (!_homeLat || !_homeLon || isNaN(_homeLat) || isNaN(_homeLon)) {
            console.log("Invalid home location coordinates for drone:", _droneName);
            return;
        }

        let droneIndex = get_drone_index(_droneName);
        if (droneIndex === -1) {
            console.log("Drone not found:", _droneName);
            return;
        }

        let drone = allDroneInfo[droneIndex];
        
        // Remove existing home marker if it exists
        if (drone.droneHomeMarker) {
            // if the position changed, update the position of the existing marker
            if (drone.droneHomeMarker.getLngLat().lat !== _homeLat || drone.droneHomeMarker.getLngLat().lng !== _homeLon) {
                console.log(`*** Updating home location marker for drone ${_droneName} to ${_homeLat}, ${_homeLon}`);
                drone.droneHomeMarker.setLngLat([_homeLon, _homeLat]);
                // update the popup content
                const popupContent = `
                    <div style="text-align: center; padding: 5px;">
                        <strong>${_droneName}</strong>
                        <br />
                        <strong><i class="fa fa-house" style="color: ${drone.droneMarkerColour};"></i> Home Location</strong>
                        <br>
                        <span style="font-size: 12px;">
                            Lat: ${_homeLat.toFixed(8)}°<br>
                            Lon: ${_homeLon.toFixed(8)}°
                        </span>
                    </div>
                `;
                drone.droneHomeMarker.setPopup(new maplibregl.Popup({ 
                    offset: 10,
                    maxWidth: '200px'
                }).setHTML(popupContent));
            }
            drone.droneHomeMarker.getElement().style.display = 'block';
        }
    }



    function create_new_drone_model(index) {
        console.log("create_new_drone_model");
        let { droneID, droneType, droneConfiguration } = allDroneInfo[index];
        droneID = droneID.replace(/\s/g, '');

        let droneModel = "drone_glb_model";
        let initialRotationY = 0;
        let modelScale = 15;

        if (droneConfiguration == "VTOL" || droneConfiguration == "FIXED-WING") {
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
                    allDroneInfo[index].droneObject.addTooltip(allDroneInfo[index].droneID, true, 5);
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


    function createDroneFieldOfViewPolygons(droneName) {
        const layerId = `${droneName}-to-fov-line`;
        const fovDirectionLayerId = `${droneName}-fov-direction-lines`;
        const polygonSourceId = `fov-polygon-${droneName}`;
        const polygonLayerId = `fov-polygon-layer-${droneName}`;
        const outlineLayerId = `fov-polygon-outline-${droneName}`;

        var newData_fov = {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [] // Ensure the polygon is closed
            }
        };

        // Check if polygonSourceId exists as a source, if not add it
        if (!map.getSource(polygonSourceId)) {
            map.addSource(polygonSourceId, {
                type: "geojson",
                data: newData_fov,
            });
        }

        // Check if lineLayer already exists
        let lineLayer = map.getLayer(layerId);
        if (!lineLayer) {
            lineLayer = new MapboxLayer({
                id: layerId,
                type: LineLayer,
                data: [],
                fp64: false,
                widthScale: 0.1,
                getWidth: 15,
                opacity: 0.2,
                widthUnit: "meters",
                getSourcePosition: ({ source }) => source,
                getTargetPosition: ({ dest }) => dest,
                getColor: [250, 50, 50],
            });
            map.addLayer(lineLayer);
        }

        // Check if FOV direction lines layer already exists
        let fovDirectionLayer = map.getLayer(fovDirectionLayerId);
        if (!fovDirectionLayer) {
            fovDirectionLayer = new MapboxLayer({
                id: fovDirectionLayerId,
                type: LineLayer,
                data: [],
                fp64: false,
                widthScale: 0.3,
                getWidth: 8,
                opacity: 0.8,
                widthUnit: "meters",
                getSourcePosition: ({ source }) => source,
                getTargetPosition: ({ dest }) => dest,
                getColor: [50, 50, 250],
            });
            map.addLayer(fovDirectionLayer);
        }

        // Check if polygonLayer already exists
        let polygonLayer = map.getLayer(polygonLayerId);
        if (!polygonLayer) {
            polygonLayer = {
                id: polygonLayerId,
                type: "fill",
                source: polygonSourceId,
                layout: {},
                paint: {
                    "fill-color": "#888888",
                    "fill-opacity": 0.2,
                },
            };
            map.addLayer(polygonLayer);
        }

        // Check if outlineLayer already exists
        let outlineLayer = map.getLayer(outlineLayerId);
        if (!outlineLayer) {
            outlineLayer = {
                id: outlineLayerId,
                type: "line",
                source: polygonSourceId,
                paint: {
                    "line-color": "#ff5555",
                    "line-width": 2,
                    "line-opacity": 0.6,
                },
            };
            map.addLayer(outlineLayer);
        }

        // Add the lineLayer to fovLineLayers with droneName as key
        fovLineLayers[droneName] = lineLayer;
        // Store reference to FOV direction lines layer
        fovLineLayers[droneName + '_direction'] = fovDirectionLayer;
     
    }

    // Helper function to calculate FOV direction lines based on drone heading and horizontal FOV
    function calculateFOVDirectionLines(droneLat, droneLon, droneAlt, droneHeading, horizontalFOV = 68, lineLength = 2000) {
        // Validate inputs
        if (typeof droneLat !== 'number' || typeof droneLon !== 'number' || 
            typeof droneAlt !== 'number' || typeof droneHeading !== 'number') {
            console.warn('Invalid input parameters for calculateFOVDirectionLines');
            return [];
        }
        
        // Convert heading to radians
        const headingRad = (droneHeading * Math.PI) / 180;
        
        // Calculate half FOV in radians
        const halfFOVRad = (horizontalFOV / 2) * Math.PI / 180;
        
        // Calculate the two FOV boundary headings
        const leftHeading = headingRad - halfFOVRad;
        const rightHeading = headingRad + halfFOVRad;
        
        // Earth radius in meters
        const earthRadius = 6378137;
        
        // Calculate end points for the FOV lines
        function calculateEndPoint(heading) {
            const deltaLat = (lineLength * Math.cos(heading)) / earthRadius * (180 / Math.PI);
            const deltaLon = (lineLength * Math.sin(heading)) / (earthRadius * Math.cos(droneLat * Math.PI / 180)) * (180 / Math.PI);
            
            return [droneLon + deltaLon, droneLat + deltaLat, droneAlt];
        }
        
        const leftEndPoint = calculateEndPoint(leftHeading);
        const rightEndPoint = calculateEndPoint(rightHeading);
        
        return [
            {
                source: [droneLon, droneLat, droneAlt],
                dest: leftEndPoint
            },
            {
                source: [droneLon, droneLat, droneAlt],
                dest: rightEndPoint
            }
        ];
    }

    function checkAndAddMoreFeatureToPanel(_droneName, _droneHasWeatherStation, _droneHasLidar, _droneHasWaterSampler){
        if (_droneHasWeatherStation) {
            addDroneToPanelListWeatherStation(_droneName);
        }
        if (_droneHasLidar) {
            addDroneToPanelListLidar(_droneName);
        }
        if (_droneHasWaterSampler) {
            addDroneToPanelListWaterSamper(_droneName);
        }
    }

    function removeDroneMoreFeatureToPanel(_droneName) {
        removeDroneToPanelListWeatherStation(_droneName)
        removeDroneToPanelListLidar(_droneName)
        removeDroneToPanelListWaterSamper(_droneName)
    }


    function updateDroneFieldOfViewOnMap(_localDrone, _fovCoordinates, _droneLatitude, _droneLongitude, _droneAltitude, _gimbalAngle) {
        if (typeof(_localDrone.droneObject) === "undefined") {
            console.log("no local drone object");
            return;
        }
        let droneName = _localDrone.droneID
        // hide layers if drone not selected
        if (!_localDrone.droneObject.selected) {
            if (map.getLayer("fov-polygon-layer-" + droneName)) {
                map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'none');
                map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'none');
                map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'none');
                map.setLayoutProperty(droneName + '-fov-direction-lines', 'visibility', 'none');
            }
            return;
        } else {
            if (map.getLayer("fov-polygon-layer-" + droneName)) {
                if (_gimbalAngle < DRONE_GIMBAL_ANGLE_MAX && _gimbalAngle <= DRONE_GIMBAL_ANGLE_FOV_MAX) {
                    // gimbal angle is in range, show FOV polygons
                    map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'visible');
                    map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'visible');
                    map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'visible');
                    // Hide FOV direction lines
                    map.setLayoutProperty(droneName + '-fov-direction-lines', 'visibility', 'none');
                } else {
                    // gimbal angle is out of range, hide FOV polygons and show FOV direction lines
                    map.setLayoutProperty("fov-polygon-layer-" + droneName, 'visibility', 'none');
                    map.setLayoutProperty("fov-polygon-outline-" + droneName, 'visibility', 'none');
                    map.setLayoutProperty(droneName + '-to-fov-line', 'visibility', 'none');
                    // Show FOV direction lines
                    map.setLayoutProperty(droneName + '-fov-direction-lines', 'visibility', 'visible');
                    
                    // Calculate and update FOV direction lines
                    const droneHeading = _localDrone.droneInfo.heading;
                    if (typeof droneHeading === 'number') {
                        const fovDirectionLines = calculateFOVDirectionLines(_droneLatitude, _droneLongitude, _droneAltitude, droneHeading);
                        
                        // Update FOV direction lines layer
                        if (fovLineLayers[droneName + '_direction'] && fovDirectionLines.length > 0) {
                            fovLineLayers[droneName + '_direction'].setProps({
                                data: fovDirectionLines,
                                getColor: [50, 150, 250],
                            });
                        }
                    }
                }
           
            }
        }

        var fovCoordinatesList = JSON.parse(_fovCoordinates);
        var fovPoints = [];

        if (fovCoordinatesList === null) {
            map.getSource("fov-polygon-" + droneName).setData({
                type: "Feature",
                geometry: {
                    type: "Polygon",
                    coordinates: [],
                },
            });
            fovLineLayers[droneName].setProps({
                data: [],
            });
            return;
        }


        fovCoordinatesList.forEach(function (point, i) {
            fovPoints.push([point[1], point[0]]);
        });

        var newData_fov = {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [fovPoints.concat([fovPoints[0]])] // Ensure the polygon is closed
            }
        };

        // Check if the source exists before setting data
        if (map.getSource("fov-polygon-" + droneName)) {
            map.getSource("fov-polygon-" + droneName).setData(newData_fov);
            // calculate the center of the polygon
            let sumLat = 0, sumLon = 0;
            for (let i = 0; i < fovCoordinatesList.length; i++) {
                sumLat += fovCoordinatesList[i][0];
                sumLon += fovCoordinatesList[i][1];
            }
            // update the view direction line
            fovLineLayers[droneName].setProps({
                data: [
                    {
                        source: [_droneLongitude, _droneLatitude, _droneAltitude],
                        dest: [fovCoordinatesList[0][1], fovCoordinatesList[0][0], fovCoordinatesList[0][2]]
                    },
                    {
                        source: [_droneLongitude, _droneLatitude, _droneAltitude],
                        dest: [fovCoordinatesList[1][1], fovCoordinatesList[1][0], fovCoordinatesList[1][2]]
                    },
                    {
                        source: [_droneLongitude, _droneLatitude, _droneAltitude],
                        dest: [fovCoordinatesList[2][1], fovCoordinatesList[2][0], fovCoordinatesList[2][2]]
                    },
                    {
                        source: [_droneLongitude, _droneLatitude, _droneAltitude],
                        dest: [fovCoordinatesList[3][1], fovCoordinatesList[3][0], fovCoordinatesList[3][2]]
                    },
                ],
            });
        }
    }

    function removeDroneFieldOfViewPolygons(droneName) {
        const layerId = `${droneName}-to-fov-line`;
        const fovDirectionLayerId = `${droneName}-fov-direction-lines`;
        const polygonSourceId = `fov-polygon-${droneName}`;
        const polygonLayerId = `fov-polygon-layer-${droneName}`;
        const outlineLayerId = `fov-polygon-outline-${droneName}`;
        
        // Remove line layer if it exists
        if (map.getLayer(layerId)) {
            map.removeLayer(layerId);
        }
        // Remove FOV direction lines layer if it exists
        if (map.getLayer(fovDirectionLayerId)) {
            map.removeLayer(fovDirectionLayerId);
        }
        // Remove polygon layer if it exists
        if (map.getLayer(polygonLayerId)) {
            map.removeLayer(polygonLayerId);
        }
        // Remove outline layer if it exists
        if (map.getLayer(outlineLayerId)) {
            map.removeLayer(outlineLayerId);
        }
        // Remove source if it exists
        if (map.getSource(polygonSourceId)) {
            map.removeSource(polygonSourceId);
        }
        // Remove references from fovLineLayers
        delete fovLineLayers[droneName];
        delete fovLineLayers[droneName + '_direction'];
    }


    //actions to execute onDraggedObject
    function onDraggedObject(e) {
        console.log("onDraggedObject");
        let draggedObject = e.detail.draggedObject;
        let draggedAction = e.detail.draggedAction;
    }



    function updateDroneModelAndPathOnMap(_drone, _latitude, _longitude, _altitude, _heading, _velocity) {
        if (_drone.droneInfo.currentCoordinate !== null) {
            _drone.droneInfo.previousCoordinate = _drone.droneInfo.currentCoordinate; //Previous and current location needed for the line layer
        }
        _drone.droneInfo.currentCoordinate = [_longitude, _latitude, _altitude];
        _drone.droneInfo.heading = _heading;
        _drone.droneInfo.velocity = _velocity;
        _drone.droneInfo.altitude = _altitude;

        _drone.droneObject.setCoords([_longitude, _latitude, _altitude]);
        _drone.droneObject.setRotation(-_heading);
        const currentLineData = {
            source: _drone.droneInfo.previousCoordinate,
            dest: _drone.droneInfo.currentCoordinate,
            color: [23, 184, 190],
        }
        updateDroneLineLayer(_drone, currentLineData);
    }

    function updateDroneStateAndMission(_drone, _missionId, _droneState) {
        if (_drone.droneMissionState === _droneState) {
            return
        }
        if ((_drone.droneMissionState !== "In_Mission" || _droneState.droneMissionState !== "Paused_Mission") && _missionId !== null && (_droneState === "In_Mission" || _droneState === "Paused_Mission")) {
            postGetMissionPointsFromMissionId({ "missionId": _missionId }).then(function (response) {
                missionData = response.data
                if (_drone.droneMarkerCounter > 0) {
                    return
                }
                if (missionData.mission_type === 'SEARCH_AND_RESCUE_MISSION') {
                    addSearchAndRescueMissionLineOnMap(_drone.droneID, missionData.mission_points);
                    // TODO: FIX BECAUSE WE DON'T HAVE WAYPOINTS
                    // place_waypoints_on_map(missionData.mission_points[0], _drone);
                    // place_waypoints_on_map(missionData.mission_points[8], _drone);
                    // place_waypoints_on_map(missionData.mission_points[81], _drone);
                    // place_waypoints_on_map(missionData.mission_points[missionData.mission_points.length - 1], _drone);
                } else if (missionData.mission_type === 'NORMAL_MISSION') {
                    missionData.mission_points.forEach((point) => {
                        place_waypoints_on_map(point, _drone);
                    })
                }
                map.flyTo({ center: [missionData.mission_points[0][0], missionData.mission_points[0][1]] });
            })
        }
        else if (_droneState !== "In_Mission" && _droneState !== "Paused_Mission") {
            removeDroneWaypoints(_drone)
        }
        _drone.droneMissionState = _droneState
    }

    function addSearchAndRescueMissionLineOnMap(_droneName, _missionPoints) {
        let droneIndex = get_drone_index(_droneName);
        let drone = allDroneInfo[droneIndex];

        if (map.getLayer("path_" + _droneName)) {
            map.removeLayer("path_" + _droneName);
        }
        if (map.getSource("path_" + _droneName)) {
            map.removeSource("path_" + _droneName);
        }
        map.addSource("path_" + _droneName, {
            type: "geojson",
            data: {
                type: "Feature",
                properties: {},
                geometry: {
                    type: "LineString",
                    coordinates: _missionPoints,
                },
            },
        });
        map.addLayer({
            id: 'path_' + _droneName,
            type: 'line',
            source: 'path_' + _droneName,
            layout: {
                'line-join': 'round',
                'line-cap': 'round',
            },
            paint: {
                'line-color': drone.droneMarkerColour,
                'line-width': 8,
            },
        });
        updateAllDroneModelsToStayAbove();
    }

    function removeSearchAndRescueMissionLineOnMap(_droneName) {
        if (map.getLayer("path_" + _droneName)) {
            map.removeLayer("path_" + _droneName);
        }
        if (map.getSource("path_" + _droneName)) {
            map.removeSource("path_" + _droneName);
        }
    }

    function updateDetectionState(_droneName, _detectionActive) {
        if (document.getElementById('drone-cv-toggle-' + _droneName) !== null) {
            const targetElement = document.getElementById('drone-cv-toggle-' + _droneName).parentNode;
            if (_detectionActive == true && targetElement.classList.contains('off')) {
                targetElement.classList.remove('off');
                targetElement.classList.add('on');
            } else if (_detectionActive == false && !targetElement.classList.contains('off')) {
                targetElement.classList.remove('on');
                targetElement.classList.add('off');
            }
        }
    }

    function updateDroneLeftPanelData(_droneName, _droneType, _droneConfiguration, _altitude, _batteryPercentage, _droneState, _vtolState, _crpsRequested, _crpsResponding) {
        // update sidebar data
        document.getElementById('sidepanel-drone-altitude-' + _droneName).innerHTML = Math.round(_altitude) + "m";
        document.getElementById('sidepanel-drone-battery-' + _droneName).innerHTML = Math.round(_batteryPercentage) + "%";

        setDroneAttributeValue(_droneName, "droneState", _droneState)
        setDroneAttributeValue(_droneName, "vtolState", _vtolState);

        // adjust MAVLINK buttons based on the drone's status
        if (_droneType == "MAVLINK") {
            if (_droneState == "Landed") {
                document.getElementById('mavlink-takeoff-land-' + _droneName).innerHTML = "<i class='fa fa-plane-departure'></i> Takeoff";
                document.getElementById('mavlink-return-' + _droneName).disabled = true;
                if (_droneConfiguration == "VTOL") {
                    document.getElementById('mavlink-transition-mc-' + _droneName).disabled = true;
                    document.getElementById('mavlink-transition-fw-' + _droneName).disabled = true;
                }
            }
            else {
                document.getElementById('mavlink-takeoff-land-' + _droneName).innerHTML = "<i class='fa fa-plane-arrival'></i> Land";
                document.getElementById('mavlink-return-' + _droneName).disabled = false;
                if (_droneConfiguration == "VTOL") {
                    document.getElementById('mavlink-transition-mc-' + _droneName).disabled = false;
                    document.getElementById('mavlink-transition-fw-' + _droneName).disabled = false;
                }
            }
        }


        // let iconColor = (_droneState == "In_Mission") ? "#9dffb3" : "#bbb";
        // let missionIcon = '<i class="fa fa-life-ring" style="color: ' + iconColor + ';"></i>';
        // document.getElementById('sidepanel-drone-status-' + _droneName).innerHTML = missionIcon;

        let statusIcon = "fa fa-mountain";
        if (_droneState == "In_Mission") {
            statusIcon = "fa fa-life-ring";
        }
        else if (_droneState == "Paused_Mission") {
            statusIcon = "fa fa-circle-pause";
        }
        else if (_droneState == "Flying") {
            statusIcon = "fa fa-paper-plane";
        }

        let statusIconColor = "white";
        let crpsStatus = "Disengaged";
        if (_crpsRequested == true) {
            statusIcon = "fa fa-sitemap";
            statusIconColor = "orange";
            crpsStatus = "Requested";
        }
        else if (_crpsResponding == true) {
            statusIcon = "fa fa-sitemap";
            statusIconColor = "#4efcb4";
            crpsStatus = "Responding";
        }

        statusIconHtml = '<i class="' + statusIcon + '" style="color: ' + statusIconColor + '; font-size: 14px;" title="' + _droneState + '"></i>';

        let droneStatusHeader = document.getElementById('sidepanel-drone-status-' + _droneName)
        droneStatusHeader.innerHTML = statusIconHtml;
        $(droneStatusHeader).attr("title", _droneState);

    }

    function checkForDroneWarnings(drones) {
     
        
        var altitudes = [];
        for (let i = 0; i < drones.length; i++) {
            if (drones[i].isConnected) {
                if (drones[i].droneInfo.altitude > 5) { // Only consider drones with altitude greater than 5m
                    altitudes.push(drones[i].droneInfo.altitude);
                }
            }
        }

        if (altitudes.length > 1) {           
            var warningsDiv = document.getElementById("drone-warnings");

            if (hasDifferenceWithinRange(altitudes, altitudeSafetyDistance - 1)) {
                warningsDiv.innerHTML = "CAUTION: Two or more UAVs are flying at similar altitudes (less than " + altitudeSafetyDistance + "m).";
                warningsDiv.style.display = "inline-block";
            }
            else {
                warningsDiv.innerHTML = "";
                warningsDiv.style.display = "none";
            }
        }
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

    function removeDroneWaypoints(_drone) {
        console.log("DELETE POINTS")
        _drone.droneCurrentMarkers.forEach(function (marker) {
            marker.remove();
        });
        removeSearchAndRescueMissionLineOnMap(_drone.droneID);
        _drone.droneCurrentMarkers = [];
        _drone.droneMissionPath = [];
        _drone.droneMarkerCounter = 0;
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
    }


    function reset_marker_counter_for_selected_drones(drones) {
        console.log("reset_marker_counter_for_selected_drones");
        drones.forEach(function (drone) {
            setDroneAttributeValue(drone.droneID, "droneMarkerCounter", 0);
        });
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

    function place_waypoints_on_map(_waypoint, _drone) {
        console.log("place_waypoints_on_map");
        _drone.droneMarkerCounter++;
        let marker = new maplibregl.Marker(createMarkerPoint(_drone)).setLngLat(_waypoint).addTo(map);
        _drone.droneCurrentMarkers.push(marker);
        _drone.droneMissionPath.push([_waypoint[0], _waypoint[1]]);
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


    function updateAllDroneModelsToStayAbove() {
        get_all_drone_info_array().forEach((drone) => {
            map.moveLayer(drone.droneLineLayer.id);
            map.moveLayer(drone.droneModel.id);
        });

    }
}
