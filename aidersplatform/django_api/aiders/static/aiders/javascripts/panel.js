// var layerList;
// var inputs;
// $("#orthophotoLayerToggle").prop("checked", true)
// window.addEventListener('load', (event) => {
//     layerList = document.getElementById('sidebar-submenu');
//     inputs = layerList.getElementsByTagName('input');
//
// });

function add_list_items_to_trajectories(drone_ids, allDrones) {
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('trajectory-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode('Line Layer ' + droneID);

        // ul.style.background="#3d3e3f"
        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'trajectories-toggle-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-trajectories-' + droneID;
        // li.style.background="inherit"

        $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            toggleLayerVisibility(this.id, droneID);
        });
    }
}

function add_list_items_to_uav_missions(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('drone-selection-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        let toggle = document.createElement('input');
        var linkText = document.createTextNode('Drone ' + droneID);

        ul.append(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'drone-toggle' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-build-map-' + droneID;

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            toggleSelectDrone(this.id, droneID);
        });
    }
}
function add_list_items_to_uav_missions_lidar(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        if (document.getElementById('lidar-data-list-default')) {
            document.getElementById('lidar-data-list-default').remove();
        }
        let droneID = drone_ids[i];

        // var ul = document.getElementById("drone-selection-list");
        var li = document.createElement('li');
        var m1 = document.createElement('m1');
        var m2 = document.createElement('m2');
        li.append(m1);
        li.append(m2);
        m1.innerHTML = 'Drone ' + droneID;
        m2.innerHTML =
            `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="startLidarPointColection" onclick="startLidarData('` +
            droneID +
            `')" type="button">START</button>
        <button aria-pressed="false" class="btn btn-outline-danger panel-btn bldmp" id="StopLidarPointColection" onclick="stopLidarData('` +
            droneID +
            `')" type="button">STOP</button>`;
        document.getElementById('lidar-data-list').appendChild(li);
    }
}

function add_list_items_to_multispectral_build_map() {
    let available_indices = MULTISPECTRAL_INDEX_NAMES;
    for (let i = 0; i < available_indices.length; i++) {
        let indice = available_indices[i];
        var ul = document.getElementById('uav-multispectral-build-map-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode(indice);

        ul.append(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'build-map-index-toggle-' + indice;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-build-map-' + indice;

        $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            console.log('CHANGE DETECTED');
            toggleMultispectralPhotoLayer(this.id, indice);
        });
    }
}
function add_list_items_to_uav_missions_water_sampler(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        if (document.getElementById('water-sampler-data-list-default')) {
            document.getElementById('water-sampler-data-list-default').remove();
        }
        let droneID = drone_ids[i];

        // var ul = document.getElementById("drone-selection-list");
        var li = document.createElement('li');
        var m1 = document.createElement('m1');
        var m2 = document.createElement('m2');
        li.append(m1);
        li.append(m2);
        m1.innerHTML = 'Drone ' + droneID;
        m2.innerHTML =
            `<button aria-pressed="false" class="btn btn-outline-success panel-btn bldmp" id="water_sampler" onclick="activate_water_sampler('` +
            droneID +
            `')" type="button">START</button>`;
        document.getElementById('water-collector-list').appendChild(li);
    }
}

function add_list_items_to_video_feeds(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('video-feeds-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode('Player ' + droneID);

        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between ');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'video-toggle-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-video-' + droneID;

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            toggleVideoVisibility(this.id, droneID);
        });
    }
}

function add_list_items_to_det_video_feeds(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('det-video-feeds-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode('Detection ' + droneID);

        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'det-video-toggle-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-det-video-' + droneID;

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            toggleDetVideoVisibility(this.id, droneID);
        });
    }
}

function add_list_items_to_map_tools(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        for (let j = 0; j < webSocketMessage.drones.length; j++) {
            if (webSocketMessage.drones[j].drone_name === drone_ids[i]) {
                let droneID = drone_ids[i];
                var ul = document.getElementById('map-tools-list');
                var li = document.createElement('li');
                var div = document.createElement('div');
                var toggle = document.createElement('input');
                var linkText = document.createTextNode('Live Data ' + droneID);

                ul.appendChild(li);

                $(div).addClass('d-flex justify-content-between');
                div.appendChild(linkText);
                div.style.fontSize = '13px';
                div.style.color = '#ffffff';
                div.style.padding = '.15rem 0.55rem';
                div.style.lineHeight = '2';
                div.style.background = 'inherit';

                toggle.id = 'drone-info-' + droneID;
                toggle.type = 'checkbox';
                div.appendChild(toggle);

                li.append(div);
                li.id = 'list-item-drone-info-' + droneID;

                $(toggle).bootstrapToggle('off');
                $(toggle).on('change', function (event) {
                    droneClickInfoBox(droneID);
                });
            }
        }
    }
}

function add_list_items_to_map_tools_weather(drone_ids) {
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('map-tools-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode('Live weather ' + droneID);

        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'drone-weather-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-drone-weather-' + droneID;

        $(toggle).bootstrapToggle('off');
        $(toggle).on('change', function (event) {
            droneClickWeatherBox(droneID);
        });
        createWeatherBoxForDrone(droneID);
    }
}

function remove_list_items_from_map_tools(droneID) {
    for (let i = 0; i < droneID.length; i++) {
        drone_id = droneID[i];
        if (document.getElementById('list-item-drone-info-' + drone_id) !== undefined) {
            try {
                document.getElementById('list-item-drone-info-' + drone_id).remove();
            } catch (err) {}
        }
        if (document.getElementById('list-item-drone-weather-' + drone_id) !== undefined) {
            try {
                document.getElementById('list-item-drone-weather-' + drone_id).remove();
            } catch (err) {}
        }
    }
}

function add_no_drones_text_on_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        if (!$('#no-drones-list-' + element_id).length) {
            let ul = document.getElementById(element_id);
            let li = document.createElement('li');
            li.id = 'no-drones-list-' + element_id;
            li.append('No Available Drones Yet!');
            li.style.color = WHITE_COLOR;
            ul.append(li);
        }
    });
}

function remove_no_drones_text_from_panel_sections(section_element_ids) {
    section_element_ids.forEach((element_id) => {
        $('#no-drones-list-' + element_id).remove();
    });
}

function add_list_items_to_enable_detections_list(drone_ids) {
    let allDrones = get_all_drone_info_array();
    for (let i = 0; i < drone_ids.length; i++) {
        let droneID = drone_ids[i];
        var ul = document.getElementById('enable-detections-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        let toggle = document.createElement('input');
        var linkText = document.createTextNode('Detection ' + droneID);

        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = 'detection-toggle-' + droneID;
        toggle.type = 'checkbox';
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-detection-' + droneID;

        // $(toggle).bootstrapToggle('off');
        // handle_detection_toggle(droneID,toggle)
        $(toggle).bootstrapToggle('off');
        get_drone_detection_state(droneID).then(function (detection_drone_state) {
            if (detection_drone_state === DETECTION_CONNECTED) {
                $(toggle).bootstrapToggle('on');
                start_detection_vid(droneID);
                for (let i = 0; i < allDrones.length; i++) {
                    if (droneID === allDrones[i].droneID) {
                        // console.log("over_here_heey_hey")
                        update_drone_detection_status_locally(i, DETECTION_CONNECTED);
                        break;
                    }
                }
            } else {
                $(toggle).bootstrapToggle('off');
            }

            $(toggle).on('change', function (event) {
                toggleDetectionFunctionality(this.id, droneID);
            });
        });
    }
}

function add_list_items_to_detection_types() {
    let prototypeModels = get_prototype_models();

    create_first_checkbox();
    for (let i = 0; i < prototypeModels.length; i++) {
        let protModel = prototypeModels[i];

        var ul = document.getElementById('detection-types-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode(protModel.type);

        // ul.style.background="#3d3e3f"
        ul.appendChild(li);

        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = protModel.checkboxID;
        toggle.type = 'checkbox';
        toggle.disabled = true;
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-detection-types-' + protModel.type;
        // li.style.background="inherit"

        // $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            toggleDetObject();
        });
    }

    function create_first_checkbox() {
        let protModel = {
            type: 'All',
            checkboxID: SHOW_ALL_DETECTIONS_CHECKBOX_ID,
        };

        var ul = document.getElementById('detection-types-list');
        var li = document.createElement('li');
        var div = document.createElement('div');
        var toggle = document.createElement('input');
        var linkText = document.createTextNode(protModel.type);
        ul.appendChild(li);
        $(div).addClass('d-flex justify-content-between');
        div.appendChild(linkText);
        div.style.fontSize = '13px';
        div.style.color = '#ffffff';
        div.style.padding = '.15rem 0.55rem';
        div.style.lineHeight = '2';
        div.style.background = 'inherit';

        toggle.id = protModel.checkboxID;
        toggle.type = 'checkbox';
        toggle.disabled = true;
        div.appendChild(toggle);

        li.append(div);
        li.id = 'list-item-detection-types-' + protModel.type;
        // li.style.background="inherit"

        // $(toggle).bootstrapToggle('on');
        $(toggle).on('change', function (event) {
            toggleDetObject();
        });
    }
}

function remove_list_items_from_enable_detections(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-detection-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_trajectories(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-trajectories-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_uav_missions(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#drone-selection-list' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_video_feeds(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-video-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_det_video_feeds(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-det-video-' + deleted_drones_ids[i]).remove();
    }
}

function remove_list_items_from_detection_types() {
    let prototypeModels = get_prototype_models();

    for (let i = 0; i < prototypeModels.length; i++) {
        $('#list-item-detection-types-' + prototypeModels[i].type).remove();
    }

    $('#list-item-detection-types-All').remove();
}

function remove_list_items_from_select_drones(deleted_drones_ids) {
    for (let i = 0; i < deleted_drones_ids.length; i++) {
        $('#list-item-build-map-' + deleted_drones_ids[i]).remove();
    }
}

/*
 * Fired when any radio button is selected. The map's style is then changed according to the chosen button
 * */
function changeLayer(layerid, layer_type) {
    let urlVector = 'https://api.maptiler.com/maps/<>/style.json?key=blpQOMdNw0JJIq07I9Ln';
    let currentStyle;
    let lastDroneLocation;
    if (layer_type === MAPLIBRE_STYLE) {
        clear_timer(droneTimer);
        let allDrones = get_all_drone_info_array();

        if (allDrones.length === 0) {
            lastDroneLocation = DEFAULT_MAP_CENTER;
        } else {
            lastDroneLocation = allDrones[0].droneInfo.currentCoordinate;
        }
        currentStyle = urlVector.replace('<>', layerid);
        map.setStyle(currentStyle);
    }
    // else if (layer_type === OWN_LAYER)
    // {
    //     currentStyle = offlineStyle
    //     map.setStyle(offlineStyle)
    // }

    sessionStorage.setItem('currentStyle', currentStyle);
    sessionStorage.setItem('currentStyleRadioBtn', layerid);
    sessionStorage.setItem('lastDroneLocation', JSON.stringify(lastDroneLocation));
    window.location.reload();
}

/*Fired when the toggle button about the video is pressed.
 * It toggles the video's frame visibility
 * */
function toggleVideoVisibility(toggleID, droneID) {
    var pressed = $('#' + toggleID).is(':checked');
    var x = document.getElementById('outer-live-stream-img-div-' + droneID);

    if (pressed) {
        x.style.display = 'block';
        start_live_stream(droneID);
    } else {
        let actionButtonID = 'live-feed-action-btn-' + droneID;
        let actionBtn = document.getElementById(actionButtonID);

        //If live stream is on when user decides to toggle off the
        //live video, then before making the video disappear, pause it.
        console.log('ACTION BTN INNER HTML: ', actionBtn.textContent.charCodeAt(0));
        let pauseBtnCode = '9612';
        let currentBtnCode = actionBtn.textContent.charCodeAt(0);
        if (currentBtnCode == pauseBtnCode) {
            //The pause button is displayed currently. Means that video is currently playing
            console.log('will now PAUSE video1');
            actionBtn.click();
        }
        x.style.display = 'none';
    }
}

function toggleDetVideoVisibility(toggleID, droneID) {
    var pressed = $('#' + toggleID).is(':checked');

    var x = document.getElementById('outer-det-img-div-' + droneID);
    postElementId(toggleID, pressed);
    if (pressed) {
        x.style.display = 'block';
    } else {
        x.style.display = 'none';
    }
}

function toggleSearchVisibility(searchBoxID) {
    var pressed = $('#' + searchBoxID).is(':checked');
    postElementId(searchBoxID, pressed);
    var x = document.getElementById('geocoder');

    if (pressed) {
        x.style.display = 'block';
    } else {
        x.style.display = 'none';
    }
}

/*Triggered when toggle buttons are checked.
 * It toggles the layers' visibility
 * */
function toggleLayerVisibility(toggleID, droneID) {
    // if (!getIfConstantsAreDeclaredFromAPI() || WEB_SERVER_URL === undefined)
    // {
    //     create_popup_for_a_little(WARNING_ALERT,"Layer needs few more seconds to be initialized!",2000)
    //     return
    // }
    let allDrones = get_all_drone_info_array();
    let layerID;
    let toggleElement = $('#' + toggleID);
    var pressed = $(toggleElement).is(':checked');
    let found = false;
    let layerLoaded;
    let selectedLayer;

    postElementId(toggleID, pressed);
    for (let i = 0; i < allDrones.length; i++) {
        if (allDrones[i].droneID === droneID) {
            layerID = allDrones[i].droneLineLayer.id;
            found = true;
            break;
        }
    }
    if (!found) {
        switch (toggleID) {
            case 'threedBuildingsToggle':
                selectedLayer = threeDbuildingLayer;
                break;

            case 'roadLayerToggle':
                selectedLayer = layerRoads;
                if (pressed) {
                    add_layer_on_map(
                        layerRoads.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_roadnetwork_original.geojson',
                        }),
                        layerRoads,
                        'geojson'
                    );
                }
                break;

            case 'buildingsLayerToggle':
                selectedLayer = layerBuildings;
                if (pressed) {
                    add_layer_on_map(
                        layerBuildings.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_buildings.geojson',
                        }),
                        layerBuildings,
                        'geojson',
                        'None',
                        'None',
                        0.1,
                        true,
                        3.5
                    );
                }

                break;

            case 'damsLayerToggle':
                selectedLayer = layerDams;
                if (pressed) {
                    add_layer_on_map(
                        layerDams.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_cyprus_dams.geojson',
                        }),
                        layerDams,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'aidersplatform_geojson_files_dam_icon.png',
                        }),
                        '#000000',
                        0.5,
                        true
                    );
                }

                break;

            case 'hospitalsLayerToggle':
                selectedLayer = layerHospitals;
                if (pressed) {
                    add_layer_on_map(
                        layerHospitals.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_cyprus_hospitals.geojson',
                        }),
                        layerHospitals,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'aidersplatform_geojson_files_hospital_icon.png',
                        }),
                        '#ff0000',
                        0.5,
                        true
                    );
                }
                break;

            case 'contoutEleLayerToggle':
                selectedLayer = layerTerrainLines;
                if (pressed) {
                    add_terrain_lines_layer();
                }
                break;

            case 'polesLayerToggle':
                selectedLayer = layerPoles;
                if (pressed) {
                    add_layer_on_map(
                        layerPoles.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_aikpilwnes.geojson',
                        }),
                        layerPoles,
                        'geojson',
                        dutils.urls.resolve('cyprus_geolocation_icons', {
                            icon_path: 'aidersplatform_geojson_files_mv_pole.png',
                        }),
                        '#000000',
                        0.5,
                        true
                    );
                }
                break;

            case 'polesLinesToggle':
                selectedLayer = layerPoleLines;
                if (pressed) {
                    add_layer_on_map(
                        layerPoleLines.source,
                        dutils.urls.resolve('cyprus_geolocation', {
                            geolocation_path: 'aidersplatform_geojson_files_aikpilwnes_lines.geojson',
                        }),
                        layerPoleLines,
                        'geojson'
                    );
                }
                break;
        }

        if (pressed) {
            let msg = 'Layer is loading. Please wait...';
            let successMsg = 'Layer successfully loaded!';
            create_popup(WARNING_ALERT, msg, 'tempPopup');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
                create_popup_for_a_little(SUCCESS_ALERT, successMsg, 1000);
            });
        } else {
            if (map.getLayer(selectedLayer.id)) {
                map.removeLayer(selectedLayer.id);
            }
        }
    }
    if (layerID !== undefined) {
        if (pressed) {
            let successMsg = 'Layer successfully visible!';
            map.setLayoutProperty(layerID, 'visibility', 'visible');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
                create_popup_for_a_little(SUCCESS_ALERT, successMsg, 1000);
            });
        } else {
            map.setLayoutProperty(layerID, 'visibility', 'none');
            map.once('idle', (e) => {
                removeEl('#tempPopup');
            });
        }
    }
}

function layer_loading() {}

var canVisualizeObjects = true;

function handle_layer_loading_popup(layer_id, pressed) {
    let okButton = 'OK';

    let dialogTitle = 'Warning';
    // if(layerExists(layer_id) && pressed)
    // {
    //     let message = "Please wait a few seconds until the layer is done loading!"
    //     showPopupForALittle('#box_layerLoading',message,2000)
    // }
    if (!layerExists(layer_id) && pressed) {
        //Layer did not load yet.
        let message = 'Layer needs a few seconds to load. Please try again in a few seconds';
        create_dialog_with_one_button(okButton, message, dialogTitle, 750, 'auto');
        return false;
    }
    return true;
}
function checkPrototypeCheckbox(toggleID) {
    let detection_arrays = get_detected_objs();
    let detected_models = detection_arrays[0];
    let detected_api_objs = detection_arrays[1];
    var pressed = $('#' + toggleID).is(':checked');

    var all_dets_toggle_pressed = $('#' + ALL_DETS_TOGGLE_ID).is(':checked');
    var person_toggle_pressed = $('#' + PERSON_DETS_TOGGLE_ID).is(':checked');
    var car_toggle_pressed = $('#' + CAR_DETS_TOGGLE_ID).is(':checked');
    var motor_toggle_pressed = $('#' + MOTOR_DETS_TOGGLE_ID).is(':checked');
    console.log('TOGGLE ID: ' + toggleID);
    console.log('IS IT PRESSED? ' + pressed);

    if (pressed) {
        if (toggleID === SHOW_DETS_TOGGLE_ID) {
            document.getElementById(ALL_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(CAR_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(MOTOR_DETS_TOGGLE_ID).disabled = false;
            document.getElementById(PERSON_DETS_TOGGLE_ID).disabled = false;
        }
        console.log('all dets pressed: ' + all_dets_toggle_pressed);
        console.log('person_dets_pressed: ' + person_toggle_pressed);
        console.log('car_dets_pressed: ' + car_toggle_pressed);
        console.log('tree_dets_pressed: ' + motor_toggle_pressed);

        // canVisualizeObjects = true
        // for (let i=0; i<detected_objs.length; i++)
        // {
        //     detected_objs[i].visibility = true
        // }

        for (let i = 0; i < detected_models.length; i++) {
            if (car_toggle_pressed && detected_api_objs[i].type === CAR) {
                console.log('OVER_IN_CARS');
                detected_models[i].visibility = true;
            } else if (person_toggle_pressed && detected_api_objs[i].type === PERSON) {
                detected_models[i].visibility = true;
            } else if (motor_toggle_pressed && detected_api_objs[i].type === MOTORBIKE) {
                detected_models[i].visibility = true;
            } else if (all_dets_toggle_pressed) {
                detected_models[i].visibility = true;
            }
        }
    } else {
        if (toggleID === SHOW_DETS_TOGGLE_ID) {
            document.getElementById(ALL_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(CAR_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(MOTOR_DETS_TOGGLE_ID).disabled = true;
            document.getElementById(PERSON_DETS_TOGGLE_ID).disabled = true;
        }

        // console.log("OVER_OVER")
        // canVisualizeObjects = false
        // for (let i=0; i<detected_objs.length; i++)
        // {
        //     detected_objs[i].visibility = false
        // }

        for (let i = 0; i < detected_models.length; i++) {
            if (toggleID === CAR_DETS_TOGGLE_ID && detected_api_objs[i].type === CAR) {
                detected_models[i].visibility = false;
            } else if (toggleID === PERSON_DETS_TOGGLE_ID && detected_api_objs[i].type === PERSON) {
                detected_models[i].visibility = false;
            } else if (toggleID === MOTOR_DETS_TOGGLE_ID && detected_api_objs[i].type === MOTORBIKE) {
                detected_models[i].visibility = false;
            } else if (toggleID === ALL_DETS_TOGGLE_ID) {
                //User wants to remove all detected objects from map
                detected_models[i].visibility = false;

                // console.log("car_dets_pressed: " + car_toggle_pressed)
                // console.log("object type: " + detected_api_objs[i].type)
                // let none_from_above = true

                // if (person_toggle_pressed && detected_api_objs[i].type === PERSON) //Don't remove people if people toggle is ON
                // {
                //     detected_models[i].visibility = true
                // }
                // else if (car_toggle_pressed && detected_api_objs[i].type === CAR)
                // {
                //     detected_models[i].visibility = true
                // }
                // else if (motor_toggle_pressed && detected_api_objs[i].type === MOTORBIKE)
                // {
                //     detected_models[i].visibility = true
                // }
                // else
                // {
                //     detected_models[i].visibility = false
                // }
            } else if (toggleID === SHOW_DETS_TOGGLE_ID) {
                detected_models[i].visibility = false;
            }
        }
    }
}

function toggleDetObject() {
    let show_dets_toggle_pressed = $('#' + SHOW_DETS_TOGGLE_ID).is(':checked');
    postElementId(SHOW_DETS_TOGGLE_ID, show_dets_toggle_pressed);
    let prototypeModels = get_prototype_models();
    let detection_arrays = get_detected_objs();

    let show_all_dets_CB_checked = $('#' + SHOW_ALL_DETECTIONS_CHECKBOX_ID).is(':checked');

    let detected_models = detection_arrays[0];
    let detected_api_objs = detection_arrays[1];
    console.log('LENGTH OF CARS: ', detected_api_objs.length);
    if (show_dets_toggle_pressed) {
        enable_all_detection_checkboxes(prototypeModels);

        for (let i = 0; i < detected_models.length; i++) {
            let objType = detected_api_objs[i].type;
            let checkboxID = getCheckboxID(objType, prototypeModels);
            // console.log("CHECKBOX ID: " + checkboxID)
            let isCBchecked = $('#' + checkboxID).is(':checked');
            postElementId(SHOW_ALL_DETECTIONS_CHECKBOX_ID, show_all_dets_CB_checked);
            if (show_all_dets_CB_checked) {
                detected_models[i].visibility = true;
            } else {
                if (isCBchecked) {
                    detected_models[i].visibility = true;
                } else {
                    detected_models[i].visibility = false;
                }
            }
        }
    } else {
        disable_all_detection_checkboxes(prototypeModels);

        for (let i = 0; i < detected_models.length; i++) {
            detected_models[i].visibility = false;
        }
    }

    function getCheckboxID(objType, protModels) {
        for (let i = 0; i < protModels.length; i++) {
            if (objType === protModels[i].type) {
                return protModels[i].checkboxID;
            }
        }

        return 'NO CHECKBOX ID FOUND';
    }

    function enable_all_detection_checkboxes(protModels) {
        for (let i = 0; i < protModels.length; i++) {
            document.getElementById(protModels[i].checkboxID).disabled = false;
        }

        document.getElementById(SHOW_ALL_DETECTIONS_CHECKBOX_ID).disabled = false;
    }

    function disable_all_detection_checkboxes(protModels) {
        for (let i = 0; i < protModels.length; i++) {
            document.getElementById(protModels[i].checkboxID).disabled = true;
        }

        document.getElementById(SHOW_ALL_DETECTIONS_CHECKBOX_ID).disabled = true;
    }
}

function toggleSelectDrone(toggleID, droneid) {
    var pressed = $('#' + toggleID).is(':checked');
    postElementId(toggleID, pressed);
    let allDrones = get_all_drone_info_array();
    // for (let i = 0; i < allDrones.length; i++)
    // {
    //     if (droneid !== allDrones[i].droneID)
    //     {
    //         // allDrones[i].droneObject.selected = false
    //         unselect_toggled_drone(i)
    //     }
    // }
    for (let i = 0; i < allDrones.length; i++) {
        // console.log("DRONE ID: " + allDrones[i].droneID )
        if (droneid === allDrones[i].droneID) {
            if (pressed) {
                //If toggle is checked but drone not selected, select it
                select_toggled_drone(i);
                return;
            } else if (!pressed) {
                // if toggle is unchecked and drone selected, UNSELECT IT
                unselect_toggled_drone(i);
                return;
            }
        }

        // else
        // {
        //     // console.log("WILL NOW UNSELECT THIS DRONE: " + allDrones[i].droneID)
        //     // console.log("SELECTED DRONES: ")
        //     console.log(get_selected_drones())
        //     unselect_toggled_drone(i)
        // }
    }
}

function add_available_detector_types_on_panel() {
    $.ajax({
        type: 'GET',
        url: dutils.urls.resolve('detection_types', { operation_name: CURRENT_OP }),
        headers: {},
        success: function (response) {
            let available_detection_types = response['detection_types'];
            create_prototype_models(available_detection_types);
            add_list_items_to_detection_types();
            let protModelsLayer = get_prot_models_layer(available_detection_types); //get the layer that will host all the detected objects found by drone
            map.addLayer(protModelsLayer);
        },
    });
}
