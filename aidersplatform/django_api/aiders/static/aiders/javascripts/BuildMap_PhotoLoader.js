/*
 * An interval that runs indefinitely, and adds the photos that were taken by the drone, on the map,
 * once they are available and only if user requested to start a build map
 * */

{
    function startListeningForBuildMap(droneID) {
        let startBuildMapElement = document.getElementById('startBuildMapBtn');
        let stopBuildMapElement = document.getElementById('stopBuildMapBtn');
        let photoIndex = 0;
        let toggle = document.getElementById('drone-toggle' + droneID);
        let corruptedRetries = 0;
        let fileNotFoundCounter = 0;
        let showSuccessPopupFlag = true;
        let executeOnlyOnce = false;

        function getBuildMapStatusFromAPI(drone_id) {
            let returnObjectStatus;
            webSocketMessage['drones'].forEach(function (drone) {
                if (drone['drone_name'] == droneID) {
                    returnObjectStatus = drone['build_map_activated'];
                }
            });
            return new Promise((resolve, reject) => resolve(returnObjectStatus));
        }
        let intervalForPhotoLoader = setInterval(function () {
            let buildMapSt = getLocalBuildMapState(droneID);
            let selectedDrone = get_drone_object(droneID);
            let altitude = selectedDrone.droneInfo.currentCoordinate[2];

            //
            // if (altitude < 25 && (localBuildMapStats === BUILD_MAP_STARTED || localBuildMapStats === BUILD_MAP_ABOUT_TO_START)) //If the altitude is below 25meters and there's active build map mission, stop it
            // {
            //     getBuildMapStatusFromAPI(droneID).then(function (build_map_stats_obj)
            //     {
            //         let buildMapStarted = ((JSON.stringify(build_map_stats_obj["start_build_map"])) === 'true');
            //
            //         if (buildMapStarted)
            //         {
            //             updateBuildMapStatusLocally(droneID, BUILD_MAP_ABOUT_TO_STOP)
            //             updateBuildMapStatusOnAPI(droneID, false)
            //             popup_BuildMapLoadingToStop(SHOW_POPUP, TOO_LOW_ALTITUDE_FOR_BUILD_MAP)
            //         }
            //     });
            // }
            let drone_availability = get_drone_availability_status(droneID);
            if (drone_availability === NOT_AVAILABLE) {
                clearInterval(intervalForPhotoLoader);
                return;
            }

            /*If user started a build map, it takes a few seconds to start it.
             * Once it is started though
             * */

            getBuildMapStatusFromAPI(droneID).then(function (build_map_stats_obj) {
                let altitude = selectedDrone.droneInfo.currentCoordinate[2];

                if (build_map_stats_obj.toString() == 'true') {
                    executeOnlyOnce = true;
                    if (altitude < 25) {
                        updateBuildMapStatusLocally(droneID, BUILD_MAP_ABOUT_TO_STOP);
                        updateBuildMapStatusOnAPI(droneID, false);
                        popup_BuildMapLoadingToStop(SHOW_POPUP, TOO_LOW_ALTITUDE_FOR_BUILD_MAP);
                        return;
                    }
                    attachPhotos(droneID); //Attach photos and come back
                } else if (build_map_stats_obj.toString() == 'false' && executeOnlyOnce) {
                    updateBuildMapStatusLocally(droneID, BUILD_MAP_STOPPED);

                    if ($('#buildMapLoadingToStopDueToLowAltitude').is(':visible')) {
                        hidePopup('#buildMapLoadingToStopDueToLowAltitude');
                        showPopupForALittle('#buildMapStoppedDueToLowAltitude', '', 3000);
                    } else {
                        hidePopup('#buildMapLoadingToStop');
                        hidePopup('#loadingBuildMapBox');
                        showPopupForALittle('#buildMapStopped', '', 3000);
                    }

                    clearLoadingToCancelBuildMapPopupTimer();
                    popup_BuildMapLoading(REMOVE_POPUP, 'SelectedDrone', 0);
                    hightlightElement(startBuildMapElement, DEACTIVATE_HIGHLIGHT);
                    photoIndex = 0;
                    showSuccessPopupFlag = true;
                    fileNotFoundCounter = 0;
                    corruptedRetries = 0;
                    executeOnlyOnce = false;
                }
            });
        }, 1000);

        function attachPhotos(drone_id) {
            for (let j = 0; j < webSocketMessage.drones.length; j++) {
                if (webSocketMessage.drones[j].drone_name === drone_id) {
                    if (webSocketMessage.drones[j].build_map_last_image != '') {
                        for (let i = 0; i < webSocketMessage.drones[j].build_map_last_image.length; i++) {
                            image = webSocketMessage.drones[j].build_map_last_image[i];
                            image_id = image['id'].toString();
                            layer_id = image['id'].toString();
                            image_url = dutils.urls.resolve('build_map_image_path', { image_path: image['path'] });
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
                                            paint: { 'raster-opacity': 1 },
                                        },
                                        drone
                                    );
                                } else {
                                    map.addLayer({
                                        id: layer_id,
                                        type: 'raster',
                                        source: image_id,
                                        paint: { 'raster-opacity': 1 },
                                    });
                                }
                                clearBuildMapLoadingPopupTimer(drone_id);
                            }
                        }
                    }
                }
            }
        }
    }

    function extractFolderNameFromPath(path) {
        let n = path.lastIndexOf('/');
        return path.substring(n + 1);
    }
}
