{
    /*Each time a new drone is pushed to the drones array, we also have to add their respective layers on the map*/
    function add_layers_on_map(allDrones, index) {
        map.addLayer(allDrones[index].droneModel); //The 3D model object
        map.addLayer(allDrones[index].droneLineLayer);
    }

    /*
     * Removes the line layer from the map e.g when user chooses to clear the layer from the map
     * */
    function remove_line_layers_from_map(allDrones) {
        for (let i = 0; i < allDrones.length; i++) {
            allDrones[i].droneLineData = [];
        }
    }

    /*
     * Updates lineLayer Data for the specified drone. It takes the values until this point and appends to them the new values
     * */
    function updateLineLayer(currentDrone, currentLineData) {
        //At first, we don't have a previous coordinate so we need to check until we get one
        if (currentDrone.droneInfo.previousCoordinate[0] !== undefined) {
            currentDrone.droneLineData = currentDrone.droneLineData.concat(currentLineData);

            currentDrone.droneLineLayer.setProps({
                data: currentDrone.droneLineData,
            });
        }
        return currentDrone;
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
                let message = 'Are you sure you want to clear all drone trajectories? ';
                let dialogTitle = 'Confirm';
                create_confirmation_dialog(okButton, cancelButton, message, dialogTitle).then(function (willClear) {
                    if (willClear) {
                        let allDrones = get_all_drone_info_array();
                        remove_line_layers_from_map(allDrones);
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
