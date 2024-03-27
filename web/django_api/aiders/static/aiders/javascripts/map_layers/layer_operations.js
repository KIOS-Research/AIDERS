{
    /*Each time a new drone is pushed to the drones array, we also have to add their respective layers on the map*/
    function add_layers_on_map(allDrones, index) {
        map.addLayer(allDrones[index].droneModel); //The 3D model object
        map.addLayer(allDrones[index].droneLineLayer);
    }
    /**
     * Each time a new device is pushed to the devices array, we also have to add their respective layers on the map
     *
     */
    function add_layers_device_on_map(allDevice, index) {
        map.addLayer(allDevice[index].deviceModel); //The 3D model object
        map.addLayer(allDevice[index].deviceLineLayer);
    }
    /**
     * Each time a new balora is pushed to the baloras array, we also have to add their respective layers on the map
     *
     */
    function add_layers_balora_on_map(allBalora, index) {
        map.addLayer(allBalora[index].baloraModel); //The 3D model object
        map.addLayer(allBalora[index].baloraLineLayer);
    }
    /*
     * Removes the line layer from the map e.g when user chooses to clear the layer from the map
     * */
    function remove_line_layers_from_map_drone(allDrones) {
        for (let i = 0; i < allDrones.length; i++) {
            allDrones[i].droneLineData = [];
        }
    }
    /*
     * Removes the line layer from the map e.g when user chooses to clear the layer from the map
     * */
    function remove_line_layers_from_map_device(allDevices) {
        for (let i = 0; i < allDevices.length; i++) {
            allDevices[i].deviceLineData = [];
        }
    }
    /*
     * Removes the line layer from the map e.g when user chooses to clear the layer from the map
     * */
    function remove_line_layers_from_map_balora(allBaloras) {
        for (let i = 0; i < allBaloras.length; i++) {
            allBaloras[i].baloraLineData = [];
        }
    }
    /*
     * Updates lineLayer Data for the specified drone. It takes the values until this point and appends to them the new values
     * */
    function updateDroneLineLayer(currentDrone, currentLineData) {
        // dont update the drone's trajectory if the source or destination is 0 (GPS position is not locked)
        if(currentLineData.source[0] == 0 || currentLineData.dest[0] == 0){
            return currentDrone;
        }
        //At first, we don't have a previous coordinate so we need to check until we get one
        if (currentDrone.droneInfo.previousCoordinate[0] !== undefined) {
            currentDrone.droneLineData = currentDrone.droneLineData.concat(currentLineData);
            // console.log(currentLineData);
            currentDrone.droneLineLayer.setProps({
                data: currentDrone.droneLineData,
            });
        }
        return currentDrone;
    }

    function updateDeviceLineLayer(currentDevice, currentLineData) {
        // dont update the device's trajectory if the source or destination is 0 (GPS position is not locked)
        if (currentLineData.source[0] == 0 || currentLineData.dest[0] == 0) {
            return currentDevice;
        }        
        //At first, we don't have a previous coordinate so we need to check until we get one
        if (currentDevice.deviceInfo.previousCoordinate[0] !== undefined) {
            currentDevice.deviceLineData = currentDevice.deviceLineData.concat(currentLineData);

            currentDevice.deviceLineLayer.setProps({
                data: currentDevice.deviceLineData,
            });
        }
        return currentDevice;
    }

    function updateBaloraLineLayer(currentBalora, currentLineData) {
        // dont update the balora's trajectory if the source or destination is 0 (GPS position is not locked)
        if (currentLineData.source[0] == 0 || currentLineData.dest[0] == 0) {
            return currentBalora;
        }          
        //At first, we don't have a previous coordinate so we need to check until we get one
        if (currentBalora.baloraInfo.previousCoordinate[0] !== undefined) {
            currentBalora.baloraLineData = currentBalora.baloraLineData.concat(currentLineData);

            currentBalora.baloraLineLayer.setProps({
                data: currentBalora.baloraLineData,
            });
        }
        return currentBalora;
    }

    /*
     * Button functionality to clear the drone trajectories
     * */
    {
        $('.btn').click(function (e) {
            let clickedBtnID = e.target.id;
            let clearLineLayers = document.getElementById('clearLineLayers');
            if (clickedBtnID === clearLineLayers.id) {
                let okButton = 'Yes';
                let cancelButton = 'No';
                let message = 'Are you sure you want to clear all trajectories? ';
                let dialogTitle = 'Confirm';
                create_confirmation_dialog(okButton, cancelButton, message, dialogTitle).then(function (willClear) {
                    if (willClear) {
                        postElementId('Clear Line layer', true);
                        let allDrones = get_all_drone_info_array();
                        remove_line_layers_from_map_drone(allDrones);
                        let allDevices = get_all_device_info_array();
                        remove_line_layers_from_map_device(allDevices);
                        let allBaloras = get_all_balora_info_array();
                        remove_line_layers_from_map_balora(allBaloras);
                    }
                });
            }
        });
    }

    function layerExists(layer_id) {
        if (map.getLayer(layer_id)) {
            return true;
        }
        return false;
    }

    function isLayerVisible(layerid) {
        return map.getLayoutProperty(layerid, 'visibility') === 'visible';
    }

    function get_num() {
        return 1;
    }

    function load_fire_geojson(geojson_url, geojson_id, fill_opacity, maxID) {
        let source_id = geojson_id;
        map.addSource(source_id, {
            type: 'geojson',
            data: geojson_url,
        });

        // Add a new layer to visualize the polygon.
        let allDrones = get_all_drone_info_array();
        let drone = undefined;
        if (allDrones.length > 0) {
            drone = allDrones[0].droneModel.id;
        }
        if (drone !== undefined) {
            map.addLayer(
                {
                    id: source_id,
                    type: 'fill',
                    source: source_id, // reference the data source
                    layout: {},
                    paint: {
                        'fill-color': [
                            'interpolate',
                            // ['cubic-bezier', 0, 0.5, 1, 0.5],
                            ['linear'],
                            ['get', 'ID'],
                            1,
                            '#e7e2e2',
                            maxID,
                            '#ff0000',
                        ],
                        'fill-opacity': fill_opacity,
                    },
                },
                drone
            );

            map.addLayer(
                {
                    id: source_id + '_outline',
                    type: 'line',
                    source: source_id,
                    layout: {},
                    paint: {
                        'line-color': '#ff0000',
                        'line-width': 4,
                    },
                },
                drone
            );
        } else {
            map.addLayer({
                id: source_id,
                type: 'fill',
                source: source_id, // reference the data source
                layout: {},
                paint: {
                    'fill-color': [
                        'interpolate',
                        // ['cubic-bezier', 0, 0.5, 1, 0.5],
                        ['linear'],
                        ['get', 'ID'],
                        1,
                        '#e7e2e2',
                        maxID,
                        '#ff0000',
                    ],
                    'fill-opacity': fill_opacity,
                },
            });

            map.addLayer({
                id: source_id + '_outline',
                type: 'line',
                source: source_id,
                layout: {},
                paint: {
                    'line-color': '#ff0000',
                    'line-width': 4,
                },
            });
        }
    }
    function load_geojson(geojson_url, geojson_id, fill_opacity, fill_color, outline_color) {
        let source_id = geojson_id;
        map.addSource(source_id, {
            type: 'geojson',
            data: geojson_url,
        });

        map.addLayer({
            id: source_id,
            type: 'fill',
            source: source_id, // reference the data source
            layout: {},
            paint: {
                'fill-color': fill_color,
                'fill-opacity': fill_opacity,
            },
        });
        // Add  outline around the polygon.
        map.addLayer({
            id: source_id + 'outline',
            type: 'line',
            source: source_id,
            layout: {},
            paint: {
                'line-color': outline_color,
                'line-width': 4,
            },
        });
    }
}
