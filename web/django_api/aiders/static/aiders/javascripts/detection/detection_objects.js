{
    let detectionObjects = {};
    let visibleDetectionObjects = {};
    let modelConfigs = {
        car: {
            color: "rgb(255, 0, 0)", // red
        },
        person: {
            color: "rgb(0, 255, 0)", // green
        },
        human: {
            color: "rgb(0, 255, 0)", // green
        },
        bicycle: {
            color: "rgb(0, 0, 255)", // blue
        },
        truck: {
            color: "rgb(255, 255, 0)", // yellow
        },
        motorbike: {
            color: "rgb(255,165,0)", // orange
        },
        boat: {
            color: "rgb(255, 69, 0)", // bright orange
        },
        skiff: {
            color: "rgb(255, 204, 0)", // yellow
        },
        sailboat: {
            color: "rgb(0, 255, 0)", // green
        },
        cruise: {
            color: "rgb(255, 69, 0)", // bright orange
        },
    };

    function normalizeDetectionObject(_object) {
        const object = { ..._object };
        object.detection_session = object.detection_session ?? object.detection_session_id;
        object.operation = object.operation ?? object.operationId;
        object.drone = object.drone ?? object.droneId;
        object.frame = object.frame ?? object.frame_id;

        if (typeof object.bounding_boxes === "string") {
            try {
                object.bounding_boxes = JSON.parse(object.bounding_boxes);
            } catch (error) {
                console.warn("Invalid bounding_boxes payload", object.bounding_boxes);
                object.bounding_boxes = null;
            }
        }

        if (object.label === "human" && modelConfigs.person !== undefined) {
            object.label = "person";
        }

        return object;
    }

    function updateDetectedObjectData(_detectionData) {
        console.log("updateDetectedObjectData", _detectionData);
        _detectionData.forEach((rawObject) => {
            const object = normalizeDetectionObject(rawObject);
            if (object.detection_session === undefined || object.track_id === undefined) {
                console.warn("Skipping detection object with missing identifiers", object);
                return;
            }
            if (modelConfigs[object.label] === undefined || modelConfigs[object.label].model === undefined) {
                console.warn("Skipping detection object with unsupported label", object.label, object);
                return;
            }

            // Create New Session if it doesn't exist
            if (detectionObjects[object.detection_session] === undefined) {
                detectionObjects[object.detection_session] = {};
            }

            // Create New Object if it doesn't exist else Update it
            if (detectionObjects[object.detection_session][object.track_id] === undefined) {
                // Initalize the tooltip data
                object.is_suspicious = false;
                object.is_following = false;
                object.description = "";
                detectionObjects[object.detection_session][object.track_id] = object;
                let mapObject;
                mapObject = modelConfigs[object.label].model.duplicate();
                if (
                    visibleDetectionObjects[object.label] === undefined ||
                    visibleDetectionObjects[object.label] === false
                ) {
                    mapObject.visibility = false;
                } else {
                    mapObject.visibility = true;
                }
                mapObject.setCoords([object.lon, object.lat]);

                // Follow: ${object.is_following}<br/>
                // Session Id: ${object.detection_session}<br/>
                tooltipElement = `
                    <div id="tooltip-${object.detection_session}-${object.track_id}">
                    Track Id: ${object.track_id}<br/>
                    Type: ${object.label}<br/>
                    Suspicious: ${object.is_suspicious}<br/>
                    Description: ${object.description}<br/>
                    <button class="btn btn-sm btn-primary" onclick="showDetectedObjectUpdateForm(${object.detection_session}, ${object.track_id}, ${object.is_suspicious}, ${object.is_following}, '${object.description}')">Update</button>
                    </div>
                `;
                // console.log("AAA-NEW", object);
                mapObject.addTooltip(tooltipElement, true, 20);
                // Add Object on the map
                tb.add(mapObject);
                detectionObjects[object.detection_session][object.track_id]["model"] = mapObject;
                detectionObjects[object.detection_session][object.track_id]["tooltipElementId"] =
                    `tooltip-${object.detection_session}-${object.track_id}`;
            } else {
                // console.log("AAA EXISTING", object);
                detectionObjects[object.detection_session][object.track_id]["model"].setCoords([
                    object.lon,
                    object.lat,
                ]);
            }
            // console.log("AAAA", detectionObjects);
        });
    }
    function updateDetectedObjectPopup(_detectionData) {
        _detectionData.forEach((rawObject) => {
            const object = normalizeDetectionObject(rawObject);
            if (object.detection_session === undefined || object.track_id === undefined) {
                return;
            }

            // Skip if session doesn't exist
            if (detectionObjects[object.detection_session] === undefined) {
                return;
            }

            // Skip if object doesn't exist else Update it
            if (detectionObjects[object.detection_session][object.track_id] === undefined) {
                return;
            } else {
                // Update the detection object data
                const detectionObject = detectionObjects[object.detection_session][object.track_id];
                detectionObject.is_suspicious = object.is_suspicious;
                // detectionObject.is_following = object.should_follow;
                detectionObject.description = object.description;
                let tooltipElement = document.getElementById(
                    detectionObjects[object.detection_session][object.track_id]["tooltipElementId"],
                );
                if (tooltipElement != null) {
                    // Follow: ${object.should_follow}<br/>
                    // Session Id: ${object.detection_session}<br/>
                    tooltipElement.innerHTML = `
                        Track Id: ${object.track_id}<br/>
                        Type: ${detectionObject.label}<br/>
                        Suspicious: ${object.is_suspicious}<br/>
                        Description: ${object.description}<br/>
                        <button class="btn btn-sm btn-primary" onclick="showDetectedObjectUpdateForm(${object.detection_session}, ${object.track_id}, ${object.is_suspicious}, ${object.is_following}, '${object.description}')">Update</button>
                    `;
                }
            }
        });
    }

    function initializeDetectedObjectOnMap() {
        for (const type in modelConfigs) {
            if (modelConfigs.hasOwnProperty(type)) {
                const config = modelConfigs[type];

                // const geometry = new THREE.BoxGeometry(1, 2, 1); // Default geometry
                const geometry = new THREE.BoxGeometry(2, 4, 2); // Default geometry


                // Assuming getMaterial is a function that takes a color and returns a material
                const material = new THREE.MeshPhongMaterial({
                    color: config.color,
                    opacity: 1,
                    transparent: true,
                    side: THREE.DoubleSide,
                });
                const mesh = new THREE.Mesh(geometry, material);

                const options = {
                    obj: mesh,
                    adjustment: { x: 0.5, y: 0.5, z: 0 },
                    units: "meters",
                    rotation: { x: 90, y: 0, z: 0 },
                    anchor: "center",
                };
                modelConfigs[type]["model"] = tb.Object3D(options); // Save the model to the detectionModels object
                createAllDetectionTypesCheckbox(type);
            }
        }
    }

    function showDetectedObjectUpdateForm(_sessionId, _trackId, _isSuspicious, _isFollowing, _description) {
        const okButton = "Submit";
        const cancelButton = "Cancel";
        const dialogTitle = "Update Object Description";
        const fields = [
            {
                id: "detection_session",
                name: "Detection Session",
                type: "number",
                value: _sessionId,
                hidden: false,
                disabled: true,
            },
            {
                id: "track_id",
                name: "Track Id",
                type: "number",
                value: _trackId,
                hidden: false,
                disabled: true,
            },
            {
                id: "is_suspicious",
                name: "Suspcious",
                type: "checkbox",
                value: _isSuspicious,
                hidden: false,
                disabled: false,
            },
            {
                id: "should_follow",
                name: "Follow",
                type: "checkbox",
                value: _isFollowing,
                hidden: true,
                disabled: false,
            },
            {
                id: "description",
                name: "Description",
                type: "text",
                value: _description,
                hidden: false,
                disabled: false,
            },
        ];
        createDynamicFormDialog(okButton, cancelButton, dialogTitle, fields).then(function (formInputs) {
            const postData = {
                detection_session_id: formInputs.data.detection_session,
                track_id: formInputs.data.track_id,
                is_suspicious: formInputs.data.is_suspicious,
                should_follow: formInputs.data.should_follow,
                description: formInputs.data.description,
            };
            postUpdateDetectionObjectDescription(postData).then((data) => {
                console.log(data.message);
            });
        });
    }

    function toggleModelVisibility(_labelName, _isVisible) {
        if (modelConfigs[_labelName] !== undefined) {
            visibleDetectionObjects[_labelName] = _isVisible;
            for (const sessionId in detectionObjects) {
                for (const detectionTrackId in detectionObjects[sessionId]) {
                    if (detectionObjects[sessionId][detectionTrackId].label === _labelName) {
                        detectionObjects[sessionId][detectionTrackId].model.visibility = _isVisible;
                    }
                }
            }
        }
    }

    function getAllDetectionNames() {
        let listOfDetectionTypes = [];
        for (const type in modelConfigs) {
            listOfDetectionTypes.push(type);
        }
        return listOfDetectionTypes;
    }
    function createAllDetectionTypesCheckbox(_name) {
        const table = document.getElementById("detectionObjectsTableCheckBox");
        const newTr = document.createElement("tr");
        // Set the inner HTML of the new <tr> with the name
        newTr.innerHTML = `
                <td class="tools-option-content">• ${_name}</td>
                <td>
                    <input id="det${_name.toLowerCase()}Checkbox" type="checkbox" class="vehicle-and-person-tracker-visual-option" onclick="vechicleAndPersonTrackerOptionClick(this)" value="${_name.toLowerCase()}" disabled="">
                </td>
            `;
        // Append the new <tr> to the table
        table.appendChild(newTr);
    }

    // Initialize
    initializeDetectedObjectOnMap();
}