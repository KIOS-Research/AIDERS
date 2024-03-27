/**
 * Creates an object for each of the newly added devices, then inserts it to the devices array,
 *   and at the end it creates the necessary layers and device 3d models for it
 **/
{
    let allDeviceInfo = [];



    function get_device_index(deviceID) {
        for (let i = 0; i < allDeviceInfo.length; i++) {
            if (deviceID === allDeviceInfo[i].deviceID) {
                return i;
            }
        }
        return -1;
    }

    function getDeviceAttributeValue(_deviceName, _attrName) {
        let index = get_device_index(_deviceName);
        return allDeviceInfo[index][_attrName];
    }

    function add_Device(deviceID, devicePK) {
        allDeviceInfo.push({
            deviceLineData: [],
            deviceInfo: {
                operator: '',
                currentCoordinate: [],
                previousCoordinate: [],
                currentBatteryLevel: [],
            },
            deviceObject: undefined,
            deviceID: deviceID,
            deviceLineLayer: undefined,
            deviceColumnLayer: undefined,
            deviceColumnData: [],
            deviceModel: undefined,
            deviceAvailability: AVAILABLE,
            deviceMarkerColour: getRandomColour(),
            devicePK: devicePK,
        });
        allDeviceInfo = create_new_device_model(allDeviceInfo.length - 1);
        allDeviceInfo = create_layers_for_new_device(allDeviceInfo, allDeviceInfo.length - 1);

        add_layers_device_on_map(allDeviceInfo, allDeviceInfo.length - 1);
        return allDeviceInfo;
    }

    function create_new_device_model(index) {
        let { deviceID } = allDeviceInfo[index];
        deviceID = deviceID.replace(/\s/g, '');

        allDeviceInfo[index].deviceModel = {
            id: deviceID + '_model',
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                var options = {
                    obj: dutils.urls.resolve('device_glb_model'),
                    type: 'gltf',
                    scale: 1,
                    units: 'meters',
                    rotation: { x: -45, y: 0, z: 180 }, //default rotation,
                    adjustment: { x: 0.5, y: 0.5, z: 0 },
                };
                tb.loadObj(options, function (model) {
                    allDeviceInfo[index].deviceObject = model.setCoords([0, 0]);
                    allDeviceInfo[index].deviceObject.addEventListener('SelectedChange', onSelectedDeviceChange, false);
                    tb.add(allDeviceInfo[index].deviceObject);
                    model.addEventListener('ObjectDragged', onDraggedObject, false);
                });
            },
            render: function (gl, matrix) {
                tb.update();
            },
        };
        return allDeviceInfo;
    }

    //actions to execute onDraggedObject
    function onDraggedObject(e) {
        let draggedObject = e.detail.draggedObject;
        let draggedAction = e.detail.draggedAction;
    }
    function delete_devices_from_array() {
        for (let i = 0; i < allDeviceInfo.length; i++) {
            allDeviceInfo[i].deviceLineData = [];
            tb.remove(allDeviceInfo[i].deviceObject);
            map.removeLayer(allDeviceInfo[i].deviceLineLayer.id);
            map.removeLayer(allDeviceInfo[i].deviceModel.id);
        }

        allDeviceInfo = [];

        return allDeviceInfo;
    }

    function add_new_device_to_array(new_device_names, device_names_and_ids) {
        let isEMpty = false;
        //Get the primary key of each new device name
        let new_device_pks = [];
        device_names_and_ids.forEach(function (device) {
            if (new_device_names.includes(device.device_name)) {
                new_device_pks.push(device.device_pk);
            }
        });
        device_names_and_ids.forEach(function (device) {
            startDeviceImageMarkers(device.device_pk)
        })
        
        if (allDeviceInfo.length === 0) {
            isEMpty = true;
            for (let i = 0; i < new_device_names.length; i++) {
                allDeviceInfo = add_Device(new_device_names[i], new_device_pks[i]);
            }
        }
        if (isEMpty) {
            return allDeviceInfo;
        }
        for (let i = 0; i < new_device_names.length; i++) {
            let deviceID = new_device_names[i];
            let deviceExists = false;
            let index = -1;
            for (let j = 0; j < allDeviceInfo.length; j++) {
                if (allDeviceInfo[j].deviceID === deviceID) {
                    deviceExists = true;
                    index = j;
                    break;
                }
            }
            if (deviceExists) {
                allDeviceInfo[index].deviceObject.visibility = true;
                allDeviceInfo[index].deviceAvailability = AVAILABLE;
            } else {
                add_Device(deviceID, new_device_pks[i]);
            }
        }
        return allDeviceInfo;
    }
    function update_all_device_array(updatedArray) {
        allDeviceInfo = updatedArray;
    }
    function get_selected_devices() {
        let selected_devices = [];
        for (let i = 0; i < allDeviceInfo.length; i++) {
            if (allDeviceInfo[i].deviceObject.selected) {
                selected_devices.push(allDeviceInfo[i]);
            }
        }
        return selected_devices;
    }
    function get_selected_device_info(selectedObject) {
        for (let i = 0; i < allDeviceInfo.length; i++) {
            if (selectedObject.uuid === allDeviceInfo[i].deviceObject.uuid) {
                return allDeviceInfo[i];
            }
        }
        return 'NOT FOUND';
    }
    /*
     * Callback method that is activated once any 3D (device) object is clicked
     * It's a handler for the object selection where we can get if the device object is selected or not
     * */
    function onSelectedDeviceChange(eventArgs) {
        let selectedObject = eventArgs.detail; //we get the object selected/unselected
        at_least_one_device_selected = get_selected_devices().length > 0;

        let selected_device = get_selected_device_info(selectedObject);
        let toggleID = 'device-toggle' + selected_device.deviceID;
        let toggle = document.getElementById(toggleID);
        if (selected_device.deviceObject.selected) {
            $(toggle).bootstrapToggle('on');
        } else {
            $(toggle).bootstrapToggle('off');
        }

        latest_device_selected_model = selectedObject;
    }

    /**
     * Create the device tooltip popup description
     */
    function dataTooltipOnDevice(device) {
        return `
        <strong>ID: ${device.deviceID}
        <br>Operator: ${device.deviceInfo.operator}   
        </strong>
        `;
    }

    /**
     * Retrieve data from Device
     */
    function dataOnDevice(device) {
        return `
        <strong>ID: ${device.deviceID}
        <br>User: ${device.deviceInfo.userName}   
        <br>Lat: ${device.deviceInfo.currentCoordinate[0].toFixed(6)} °   
        <br>Lon: ${device.deviceInfo.currentCoordinate[1].toFixed(6)} °
        <br>Heading: ${device.deviceInfo.heading} °
        <br>Battery: ${device.deviceInfo.currentBatteryLevel} %
        </strong>
        `;
    }

    /*
     * Adds a tooltip to the 3d models
     * */
    function addTooltipOnDevices(new_device_ids) {
        for (var i = 0; i < allDeviceInfo.length; i++) {
            for (var j = 0; j < new_device_ids.length; j++) {
                if (allDeviceInfo[i].deviceID === new_device_ids[j]) {
                    allDeviceInfo[i].deviceObject.addTooltip(
                        `<div id="device_tooltip_${allDeviceInfo[i].deviceID}">` + dataTooltipOnDevice(allDeviceInfo[i]) + '</div>',
                        true,
                        13
                    );
                }
            }
        }
        return allDeviceInfo;
    }

    /**
     * Later if want to change popup data
     *
     * Refreshed the tooltip data for devices
     *
     * first_time = 0;
     * function refreshTooltipOnDevices(device) {
     *     if (first_time == 0) {
     *         if (document.getElementById('device_tooltip_' + device.deviceID) !== null) {
     *             document.getElementById('device_tooltip_' + device.deviceID).innerHTML = dataTooltipOnDevice(device);
     *         }
     *     }
     * }
     */


    function removeDeviceImageMarker(image) {
        if (allDeviceImages[image.session_id][image.id] !== undefined) {
            allDeviceImages[image.session_id][image.id].remove();
            delete allDeviceImages[image.session_id][image.id];
        }
    }
    function get_all_device_info_array() {
        return allDeviceInfo;
    }
    function unselect_toggled_device(deviceIndex) {
        allDeviceInfo[deviceIndex].deviceObject.selected = false;
    }

    function select_toggled_device(deviceIndex) {
        allDeviceInfo[deviceIndex].deviceObject.selected = true;
    }
    /*Hides the disconnected/deleted devices*/
    function hide_devices_from_map(deleted_devices) {
        for (let i = 0; i < deleted_devices.length; i++) {
            let deviceID = deleted_devices[i];
            let deviceExists = false;
            let index = -1;
            for (let j = 0; j < allDeviceInfo.length; j++) {
                if (allDeviceInfo[j].deviceID === deviceID) {
                    deviceExists = true;
                    index = j;
                    break;
                }
            }

            if (deviceExists) {
                allDeviceInfo[index].deviceObject.visibility = false;
                allDeviceInfo[index].deviceAvailability = NOT_AVAILABLE;
            }
			get_all_device_info_array().forEach((device) => {
				console.log(device.deviceID, deleted_devices[i], device.deviceID === deleted_devices[i]);
				if (device.deviceID === deleted_devices[i]) {
					stopDeviceImageMarkers(device.devicePK);
				}
			});
        }
    }
}

