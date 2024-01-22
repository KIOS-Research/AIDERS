/**
 * Creates an object for each of the newly added baloras, then inserts it to the baloras array,
 *   and at the end it creates the necessary layers and balora 3d models for it
 **/
{
    let allBaloraInfo = [];

    function add_Balora(baloraID, baloraPK) {
        allBaloraInfo.push({
            baloraLineData: [],
            baloraInfo: {
                receiver: '',
                currentCoordinate: [],
                previousCoordinate: [],
                currentBatteryLevel: [],
            },
            baloraObject: undefined,
            baloraID: baloraID,
            baloraLineLayer: undefined,
            baloraColumnLayer: undefined,
            baloraColumnData: [],
            baloraModel: undefined,
            baloraAvailability: AVAILABLE,
            baloraMarkerColour: getRandomColour(),
            baloraPK: baloraPK,
        });
        allBaloraInfo = create_new_balora_model(allBaloraInfo.length - 1);
        allBaloraInfo = create_layers_for_new_balora(allBaloraInfo, allBaloraInfo.length - 1);

        add_layers_balora_on_map(allBaloraInfo, allBaloraInfo.length - 1);
        return allBaloraInfo;
    }

    function create_new_balora_model(index) {
        let { baloraID } = allBaloraInfo[index];
        baloraID = baloraID.replace(/\s/g, '');
        allBaloraInfo[index].baloraModel = {
            id: baloraID + '_model',
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                options = {
                    obj: dutils.urls.resolve('balora_glb_model'),
                    type: 'gltf',
                    scale: 0.05,
                    units: 'meters',
                    rotation: { x: 90, y: 0, z: 0 }, //default rotation,
                    adjustment: { x: 0.5, y: 0.5, z: 0 },
                };
                tb.loadObj(options, function (model) {
                    allBaloraInfo[index].baloraObject = model.setCoords([0, 0]);
                    allBaloraInfo[index].baloraObject.addEventListener('SelectedChange', onSelectedBaloraChange, false);
                    tb.add(allBaloraInfo[index].baloraObject);
                    model.addEventListener('ObjectDragged', onDraggedObject, false);
                });
            },
            render: function (gl, matrix) {
                tb.update();
            },
        };
        return allBaloraInfo;
    }

    //actions to execute onDraggedObject

    function onDraggedObject(e) {
        let draggedObject = e.detail.draggedObject;
        let draggedAction = e.detail.draggedAction;
    }

    function delete_baloras_from_array() {
        for (let i = 0; i < allBaloraInfo.length; i++) {
            allBaloraInfo[i].baloraLineData = [];
            tb.remove(allBaloraInfo[i].baloraObject);
            map.removeLayer(allBaloraInfo[i].baloraLineLayer.id);
            map.removeLayer(allBaloraInfo[i].baloraModel.id);
        }

        allBaloraInfo = [];

        return allBaloraInfo;
    }

    function add_new_balora_to_array(new_balora_names, balora_names_and_ids) {
        let isEMpty = false;
        //Get the primary key of each new balora name
        let new_balora_pks = [];
        balora_names_and_ids.forEach(function (balora) {
            if (new_balora_names.includes(balora.balora_name)) {
                new_balora_pks.push(balora.pk);
            }
        });
        if (allBaloraInfo.length === 0) {
            isEMpty = true;
            for (let i = 0; i < new_balora_names.length; i++) {
                allBaloraInfo = add_Balora(new_balora_names[i], new_balora_pks[i]);
            }
        }
        if (isEMpty) {
            return allBaloraInfo;
        }
        for (let i = 0; i < new_balora_names.length; i++) {
            let baloraID = new_balora_names[i];
            let baloraExists = false;
            let index = -1;
            for (let j = 0; j < allBaloraInfo.length; j++) {
                if (allBaloraInfo[j].baloraID === baloraID) {
                    baloraExists = true;
                    index = j;
                    break;
                }
            }
            if (baloraExists) {
                allBaloraInfo[index].baloraObject.visibility = true;
                allBaloraInfo[index].baloraAvailability = AVAILABLE;
            } else {
                add_Balora(baloraID, new_balora_pks[i]);
            }
        }
        return allBaloraInfo;
    }
    function update_all_balora_array(updatedArray) {
        allBaloraInfo = updatedArray;
    }
    function get_selected_baloras() {
        let selected_baloras = [];
        for (let i = 0; i < allBaloraInfo.length; i++) {
            if (allBaloraInfo[i].baloraObject.selected) {
                selected_baloras.push(allBaloraInfo[i]);
            }
        }
        return selected_baloras;
    }
    function get_selected_balora_info(selectedObject) {
        for (let i = 0; i < allBaloraInfo.length; i++) {
            if (selectedObject.uuid === allBaloraInfo[i].baloraObject.uuid) {
                return allBaloraInfo[i];
            }
        }
        return 'NOT FOUND';
    }
    /*
     * Callback method that is activated once any 3D (balora) object is clicked
     * It's a handler for the object selection where we can get if the balora object is selected or not
     * */
    function onSelectedBaloraChange(eventArgs) {
        let selectedObject = eventArgs.detail; //we get the object selected/unselected
        at_least_one_balora_selected = get_selected_baloras().length > 0;

        let selected_balora = get_selected_balora_info(selectedObject);
        let toggleID = 'balora-toggle' + selected_balora.baloraID;
        let toggle = document.getElementById(toggleID);
        if (selected_balora.baloraObject.selected) {
            $(toggle).bootstrapToggle('on');
        } else {
            $(toggle).bootstrapToggle('off');
        }

        latest_balora_selected_model = selectedObject;
    }

    /**
     * Create the balora tooltip popup description
     */
    function dataTooltipOnBalora(balora) {
        return `
            <strong>ID: ${balora.baloraID}
            </strong>
            `;
    }
    /*
     * Adds a tooltip to the 3d models
     * */
    function addTooltipOnBaloras(new_balora_ids) {
        for (let i = 0; i < allBaloraInfo.length; i++) {
            for (let j = 0; j < new_balora_ids.length; j++) {
                if (allBaloraInfo[i].baloraID === new_balora_ids[j]) {
                    allBaloraInfo[i].baloraObject.addTooltip(
                        `<div id="balora_tooltip_${allBaloraInfo[i].baloraID}">` + dataTooltipOnBalora(allBaloraInfo[i]) + '</div>',
                        true,
                        13
                    );
                }
            }
        }
        return allBaloraInfo;
    }

    /**
     * Create the balora tooltip popup description
     */
    function dataTooltipOnBalora(balora) {
        return `
        <strong>ID: ${balora.baloraID}
        </strong>
        `;
    }
    function get_all_balora_info_array() {
        return allBaloraInfo;
    }
    function unselect_toggled_balora(baloraIndex) {
        allBaloraInfo[baloraIndex].baloraObject.selected = false;
    }

    function select_toggled_balora(baloraIndex) {
        allBaloraInfo[baloraIndex].baloraObject.selected = true;
    }
    /*Hides the disconnected/deleted baloras*/
    function hide_baloras_from_map(deleted_baloras) {
        for (let i = 0; i < deleted_baloras.length; i++) {
            let baloraID = deleted_baloras[i];
            let baloraExists = false;
            let index = -1;
            for (let j = 0; j < allBaloraInfo.length; j++) {
                if (allBaloraInfo[j].baloraID === baloraID) {
                    baloraExists = true;
                    index = j;
                    break;
                }
            }

            if (baloraExists) {
                allBaloraInfo[index].baloraObject.visibility = false;
                allBaloraInfo[index].baloraAvailability = NOT_AVAILABLE;
            }
        }
    }
}
