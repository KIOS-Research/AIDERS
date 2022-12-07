{
    function startLidarData(droneID) {
        data = {
            lidar: {
                drone_name: droneID,
                operation_name: CURRENT_OP,
                activate: true,
            },
        };
        socket.send(JSON.stringify(data));
    }
    function stopLidarData(droneID) {
        data = {
            lidar: {
                drone_name: droneID,
                operation_name: CURRENT_OP,
                activate: false,
            },
        };
        socket.send(JSON.stringify(data));
    }

    let global_active_lidar_points_periods = [];

    function load_points_meshes() {
        $.ajax({
            type: 'GET',
            url: dutils.urls.resolve('lidar_3d_points', {
                operation_name: CURRENT_OP,
            }),
            success: function (response) {
                response = JSON.parse(response);
                let not_active_ms_lidar_periods = subtract_two_object_arrays(
                    response,
                    global_active_lidar_points_periods
                );
                if (not_active_ms_lidar_periods.length == 0) {
                    popup_element = '#failureBox';
                    message = 'There not 3D Points available to load.';
                    duration_ms = 3000;
                    showPopupForALittle(popup_element, message, duration_ms);
                } else {
                    let okButton = 'Load 3D Points';
                    let cancelButton = 'Cancel';
                    let dialogTitle = 'Choose 3D Points sessions to load';
                    form_element = create_form_with_checkboxes_for_3d_mesh(not_active_ms_lidar_periods);
                    create_3d_mesh_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                        function (canProceed_and_dialog) {
                            let canProceed = canProceed_and_dialog[0];
                            let dialog = canProceed_and_dialog[1];
                            if (canProceed) {
                                let checkboxes = $('input[id^="myCustomCheckBox"]');
                                let selected_periods = get_selected_mesh_period_checkboxes(checkboxes, response);
                                global_active_lidar_points_periods.push(...selected_periods);
                                selected_periods.forEach(function (selected_option) {
                                    let pointCloudLayer;
                                    $.ajax({
                                        type: 'POST',
                                        url: dutils.urls.resolve('lidar_3d_points', {
                                            operation_name: CURRENT_OP,
                                        }),
                                        data: { mesh_id: selected_option.id },
                                        headers: {
                                            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                                        },
                                        success: function (response) {
                                            response = JSON.parse(response);
                                            load_points(response.data, selected_option.id, response.coordinates);
                                        },
                                    });
                                    dialog.remove();
                                    form_element.remove();
                                });
                            }
                        }
                    );
                }
            },
        });
    }

    function clean_points_meshes() {
        let form_element = create_form_with_checkboxes_for_3d_mesh(global_active_lidar_points_periods);
        let okButton = 'Clear';
        let cancelButton = 'Cancel';
        let dialogTitle = 'Choose 3D mesh sessions to clear';
        if (global_active_lidar_points_periods.length == 0) {
            popup_element = '#failureBox';
            message = 'There not 3D Meshes on the map to clean.';
            duration_ms = 3000;
            showPopupForALittle(popup_element, message, duration_ms);
        } else {
            create_3d_mesh_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                function (canProceed_and_dialog) {
                    let canProceed = canProceed_and_dialog[0];
                    let dialog = canProceed_and_dialog[1];
                    if (canProceed) {
                        let checkboxes = $('input[id^="myCustomCheckBox"]');
                        let selected_periods = get_selected_mesh_period_checkboxes(
                            checkboxes,
                            global_active_lidar_points_periods
                        );
                        global_active_lidar_points_periods = subtract_two_object_arrays(
                            global_active_lidar_points_periods,
                            selected_periods
                        );
                        selected_periods.forEach(function (selected_option) {
                            let points_id = selected_option['id'];
                            if (map.getLayer('3d_points_' + points_id)) {
                                map.removeLayer('3d_points_' + points_id);
                            }
                            dialog.remove();
                            form_element.remove();
                        });
                    }
                }
            );
        }
    }

    let global_active_lidar_mesh_periods = [];

    function load_triangle_meshes() {
        $.ajax({
            type: 'GET',
            url: dutils.urls.resolve('lidar_3d_meshes', {
                operation_name: CURRENT_OP,
            }),
            success: function (response) {
                response = JSON.parse(response);
                let not_active_ms_lidar_periods = subtract_two_object_arrays(
                    response,
                    global_active_lidar_mesh_periods
                );
                if (not_active_ms_lidar_periods.length == 0) {
                    popup_element = '#failureBox';
                    message = 'There not 3D Meshes available to load.';
                    duration_ms = 3000;
                    showPopupForALittle(popup_element, message, duration_ms);
                } else {
                    let okButton = 'Load 3D Mesh';
                    let cancelButton = 'Cancel';
                    let dialogTitle = 'Choose 3D mesh sessions to load';
                    form_element = create_form_with_checkboxes_for_3d_mesh(not_active_ms_lidar_periods);
                    create_3d_mesh_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                        function (canProceed_and_dialog) {
                            let canProceed = canProceed_and_dialog[0];
                            let dialog = canProceed_and_dialog[1];
                            if (canProceed) {
                                let checkboxes = $('input[id^="myCustomCheckBox"]');
                                let selected_periods = get_selected_mesh_period_checkboxes(checkboxes, response);
                                global_active_lidar_mesh_periods.push(...selected_periods);
                                selected_periods.forEach(function (selected_option) {
                                    $.ajax({
                                        type: 'POST',
                                        url: dutils.urls.resolve('lidar_3d_meshes', {
                                            operation_name: CURRENT_OP,
                                        }),
                                        data: { mesh_id: selected_option.id },
                                        headers: {
                                            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                                        },
                                        success: function (response) {
                                            response = JSON.parse(response);
                                            load_mesh(response);
                                        },
                                    });
                                    dialog.remove();
                                    form_element.remove();
                                });
                            }
                        }
                    );
                }
            },
        });
    }
    function clean_triangle_meshes() {
        let form_element = create_form_with_checkboxes_for_3d_mesh(global_active_lidar_mesh_periods);
        let okButton = 'Clear';
        let cancelButton = 'Cancel';
        let dialogTitle = 'Choose 3D mesh sessions to clear';
        if (global_active_lidar_mesh_periods.length == 0) {
            popup_element = '#failureBox';
            message = 'There not 3D Meshes on the map to clean.';
            duration_ms = 3000;
            showPopupForALittle(popup_element, message, duration_ms);
        } else {
            create_3d_mesh_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                function (canProceed_and_dialog) {
                    let canProceed = canProceed_and_dialog[0];
                    let dialog = canProceed_and_dialog[1];
                    if (canProceed) {
                        let checkboxes = $('input[id^="myCustomCheckBox"]');
                        let selected_periods = get_selected_mesh_period_checkboxes(
                            checkboxes,
                            global_active_lidar_mesh_periods
                        );
                        global_active_lidar_mesh_periods = subtract_two_object_arrays(
                            global_active_lidar_mesh_periods,
                            selected_periods
                        );
                        selected_periods.forEach(function (selected_option) {
                            let mesh_id = selected_option['id'];
                            if (map.getLayer('3d_mesh_' + mesh_id)) {
                                map.removeLayer('3d_mesh_' + mesh_id);
                            }
                            dialog.remove();
                            form_element.remove();
                        });
                    }
                }
            );
        }
    }

    function get_selected_mesh_period_checkboxes(checkboxes, all_mesh_periods) {
        let selected_mesh_periods = [];
        $(checkboxes).each(function () {
            let checkbox = this;
            if (checkbox.checked) {
                //for every checked checkbox, find its index
                for (let i = 0; i < all_mesh_periods.length; i++) {
                    let drone_id = all_mesh_periods[i]['id'];
                    let start_time = convertUTCDateToLocalDate(all_mesh_periods[i]['start_time']);
                    let valueToCheck = drone_id + start_time;
                    if (valueToCheck === $(checkbox).val()) {
                        selected_mesh_periods.push(all_mesh_periods[i]);
                    }
                }
            }
        });

        return selected_mesh_periods;
    }
    function load_points(points_data, id, coordinates) {
        // later add color to points
        const { MapboxLayer, PointCloudLayer, LineLayer, ColumnLayer, COORDINATE_SYSTEM } = deck;

        var newPointCloud = [];
        allPoints = points_data;

        for (var key in allPoints) {
            var objectToPush = {
                position: allPoints[key].coordinates,
                color: allPoints[key].color,
            };
            newPointCloud.push(objectToPush);
        }

        var pointCloudLayer = new MapboxLayer({
            id: '3d_points_' + id,
            type: PointCloudLayer,
            // coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
            coordinateSystem: COORDINATE_SYSTEM.METER_OFFSETS,
            coordinateOrigin: [coordinates[1], coordinates[0]],
            data: newPointCloud,
            getPosition: (d) => d.position,
            getColor: (d) => d.color,
            sizeUnits: 'meters',
            pointSize: 0.01,
            opacity: 1,
        });
        map.addLayer(pointCloudLayer);
        map.flyTo({
            center: [coordinates[1], coordinates[0]],
            zoom: 20,
        });
    }
    function load_mesh(mesh_data) {
        let file_path = mesh_data.file_path;
        var origin = [mesh_data.long, mesh_data.lat, mesh_data.height];
        let obj_mesh;
        if (!map.getLayer('3d_mesh_' + mesh_data.id)) {
            map.addLayer({
                id: '3d_mesh_' + mesh_data.id,
                type: 'custom',
                renderingMode: '3d',
                onAdd: function (map, mbxContext) {
                    var options = {
                        obj: dutils.urls.resolve('build_map_image_path', {
                            image_path: file_path,
                        }),
                        type: 'gltf',
                        scale: 1,
                        units: 'meters',
                        rotation: { x: 90, y: 0, z: 90 }, //default rotation
                        anchor: 'center',
                    };

                    tb.loadObj(options, function (model) {
                        obj_mesh = model.setCoords(origin);
                        tb.add(obj_mesh);
                    });
                },
                onRemove: function (map, mbxContext) {
                    tb.remove(obj_mesh);
                },
                render: function (gl, matrix) {
                    tb.update();
                },
            });
            map.flyTo({
                center: [mesh_data.long, mesh_data.lat],
                zoom: 20,
            });
        }
    }

    function process_mesh() {
        $.ajax({
            type: 'GET',
            url: dutils.urls.resolve('lidar_process', {
                operation_name: CURRENT_OP,
            }),
            success: function (response) {
                response = JSON.parse(response);
                if (response.length == 0) {
                    popup_element = '#failureBox';
                    message = 'There not 3D points to process.';
                    duration_ms = 3000;
                    showPopupForALittle(popup_element, message, duration_ms);
                } else {
                    let okButton = 'Load Build Map';
                    let cancelButton = 'Cancel';
                    let dialogTitle = 'Choose 3D mesh sessions to process';
                    form_element = create_form_with_checkboxes_for_3d_mesh(response);
                    create_3d_mesh_dialog_with_checkboxes_form(form_element, okButton, cancelButton, dialogTitle).then(
                        function (canProceed_and_dialog) {
                            let canProceed = canProceed_and_dialog[0];
                            let dialog = canProceed_and_dialog[1];
                            if (canProceed) {
                                let checkboxes = $('input[id^="myCustomCheckBox"]');
                                let selected_periods_lidar = get_selected_mesh_period_checkboxes(checkboxes, response);
                                selected_periods_lidar.forEach(function (selected_option) {
                                    $.ajax({
                                        type: 'POST',
                                        url: dutils.urls.resolve('lidar_process', {
                                            operation_name: CURRENT_OP,
                                        }),
                                        data: { mesh_id: selected_option.id },
                                        headers: {
                                            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                                        },
                                        success: function (response) {
                                            if (response == 500) {
                                                popup_element = '#failureBox';
                                                message = 'Error while processing mesh.';
                                                duration_ms = 3000;
                                                showPopupForALittle(popup_element, message, duration_ms);
                                            }
                                            if (response == 200) {
                                                popup_element = '#successBox';
                                                message = 'Finished processing  mesh.';
                                                duration_ms = 3000;
                                                showPopupForALittle(popup_element, message, duration_ms);
                                            }
                                        },
                                    });
                                    dialog.remove();
                                    form_element.remove();
                                });
                            }
                        }
                    );
                }
            },
        });
    }
}
