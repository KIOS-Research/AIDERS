console.log("SCRIPT STARTED");

let crisisIncidentObjects = {};  // Store models by incident_id

let crisisTypeModelConfigs = {
    "Human_Trafficking": {
        shape: "cube",
    },
    "Smuggling": {
        shape: "cone",
    },
    "Natural_Disaster": {
        shape: "sphere",
    },
    "Terrorist_Attack": {
        shape: "cylinder",
    },
    "SuspiciousActivity": {
        shape: "cylinder"
    }
};

let crisisSeverityModelConfigs = {
    High: {
        color: "rgb(255, 0, 0)", // red
    },
    Low: {
        color: "rgb(34, 129, 34)", // green
    },
    Medium: {
        color: "rgb(207, 158, 21)", // yellow
    },
};

function getGeometryFromShape(shape) {
    switch (shape.toLowerCase()) {
        case "cube":
            return new THREE.BoxGeometry(20, 20, 20);
        case "cylinder":
            return new THREE.CylinderGeometry(5, 5, 1, 32);
        case "cone":
            return new THREE.ConeGeometry(15, 20, 32);
        case "sphere":
            return new THREE.SphereGeometry(10, 32, 32);
        default:
            console.warn(`Unknown shape "${shape}", defaulting to BoxGeometry.`);
            return new THREE.BoxGeometry(100, 100, 100);
    }
}

let crisisClassificationModels = {};

function initializeCrisisModels() {
    for (const type in crisisTypeModelConfigs) {
        for (const severity in crisisSeverityModelConfigs) {
            if (crisisTypeModelConfigs.hasOwnProperty(type)) {
                const config = crisisTypeModelConfigs[type];
                const geometry = getGeometryFromShape(config.shape);
                const material = new THREE.MeshPhongMaterial({
                    color: crisisSeverityModelConfigs[severity].color,
                    opacity: 0.2,
                    transparent: true,
                    side: THREE.DoubleSide,
                });
                const mesh = new THREE.Mesh(geometry, material);
                const options = {
                    obj: mesh,
                    adjustment: { x: 0, y: 0, z: 0 },
                    units: "meters",
                    rotation: { x: 90, y: 0, z: 0 },
                    anchor: "center",
                };

                if (!crisisClassificationModels[type]) {
                    crisisClassificationModels[type] = {};
                }

                crisisClassificationModels[type][severity] = tb.Object3D(options); // Save the model
            }
        }
    }
}

// This function adds the crisis incident to the map and stores the model by incident_id
function addCrisisIncidentToMap(_crisisClassificationData) {

    _crisisClassificationData.forEach((object) => {
        const {
            id,
            incident_id,
            incident_type,
            severity_level,
            latitude,
            longitude,
            timestamp,
            // action_type,
            sender,
            sent_utc,
            area_type,
            time_of_day,
            detections_summary,
            description,
            resolved,
            false_alarm,
            updated_at,
            updated_by,
        } = object;

        // console.log(object);
        // console.log("Adding crisis incident to map:", id, incident_id);
        

        // Check if the incident already exists in the map
        if (crisisIncidentObjects[incident_id]) {
            console.log("CRISIS DATA ALREADY EXISTS!");
            const modelInstance = crisisIncidentObjects[incident_id];
            // console.log(modelInstance.model);
            modelInstance.setScale(100.0);
            
            if (modelInstance && modelInstance.model && modelInstance.model.material) {
                // Update the model's material color based on the severity level
                const mesh = modelInstance.model;
                mesh.material.color.set(crisisSeverityModelConfigs[severity_level].color);
                // mesh.material.color.set("rgb(0, 162, 255)");

                mesh.material.needsUpdate = true;
                // Update the model's position
                modelInstance.setCoords([longitude, latitude]);

                // Update the tooltip if it exists
                // let tooltipElement = document.getElementById(modelInstance.userData.tooltipElementId);
                // console.log("Tooltip Element ID:", modelInstance.userData.tooltipElementId);
                // console.log(id);
                
                
 
                const tooltipHTML = `
                    <div id="${modelInstance.userData.tooltipElementId}" style="text-align:left; font-size: 11px;" class="font-weight-normal">
                        <strong>Incident ID:</strong> ${incident_id}<br/>
                        <strong>Severity:</strong> ${severity_level}<br/>
                        <strong>Latitude:</strong> ${latitude}<br/>
                        <strong>Longitude:</strong> ${longitude}<br/>
                        <strong>Area Type:</strong> ${area_type || "-"}<br/>
                        <strong>Time of Day:</strong> ${time_of_day || "-"}<br/>
                        ${renderDetectionsSummaryTable(detections_summary)}
                        <strong>Description:</strong> ${description || "-"}<br/>
                        <strong>Resolved:</strong> ${resolved ? "<i class='fa-solid fa-square-check text-success' style='font-size: 16px;'></i>" : "<i class='fa-solid fa-square-xmark text-danger' style='font-size: 16px;'></i>"}<br/>
                        <strong>False Alarm:</strong> ${false_alarm ? "<i class='fa-solid fa-square-check text-success' style='font-size: 16px;'></i>" : "<i class='fa-solid fa-square-xmark text-danger' style='font-size: 16px;'></i>"}<br/>
                        <strong>Sent UTC:</strong> ${sent_utc.substring(0, 19)}<br/>
                        <strong>Updated At:</strong> ${updated_at ? updated_at.split('.')[0] : "-"}<br/>
                        <strong>Updated By:</strong> ${updated_by || "-"}<br/>
                        <div class='text-center'>
                            <button onclick="showCrisisClassificationModal('${id}', '${incident_id}', '${description}', '${resolved}', '${false_alarm}')" class="btn-sm btn-primary" style="width: 100%">Edit Incident</button>
                        </div>
                    </div>
                `;

                modelInstance.addTooltip(tooltipHTML, true, 20);
                
                // delete the div for this alert in the DOM if it exists
                const alertDiv = document.getElementById("crisis-alert-" + incident_id);
                if (alertDiv) {
                    alertDiv.remove();
                }
                // console.log(modelInstance.userData.tooltipElementId);

                // TODO: finish the logic to update the tooltip or other properties when the incident_id already exists
            }
            return;
        }

        const baseModel = crisisClassificationModels?.[incident_type]?.[severity_level];

        if (!baseModel) {
            console.warn(`Unknown incident type or severity: ${incident_type}, ${severity_level}`);
            return;
        }

        // Duplicate the base model for the new incident
        const modelInstance = baseModel.duplicate();
        // Make sure each instance has its own material
        if (modelInstance.model && modelInstance.model.material) {
            modelInstance.model.material = modelInstance.model.material.clone();
        }
        
        
        modelInstance.setCoords([longitude, latitude]);
        modelInstance.label = true;
        modelInstance.visibility = true;

        let sentTimestamp = sent_utc.substring(0, 19)
        // sentTimestamp = sentTimestamp.toISOString().split('.')[0] + 'Z';
        // Tooltip logic
        const cleanUpdatedAt = updated_at ? updated_at.split('.')[0] : "-";
        const tooltipElementId = `crisis-tooltip-${incident_id}`;
        // <strong>Type:</strong> ${incident_type}<br/>
        // <strong>Sender:</strong> ${sender}<br/>
        const tooltipHTML = `
            <div id="${tooltipElementId}" style="text-align:left; font-size: 11px;" class="font-weight-normal">
                <strong>Incident ID:</strong> ${incident_id}<br/>
                <strong>Severity:</strong> ${severity_level}<br/>
                <strong>Latitude:</strong> ${latitude}<br/>
                <strong>Longitude:</strong> ${longitude}<br/>
                <strong>Area Type:</strong> ${area_type || "-"}<br/>
                <strong>Time of Day:</strong> ${time_of_day || "-"}<br/>
                ${renderDetectionsSummaryTable(detections_summary)}
                <strong>Description:</strong> ${description || "-"}<br/>
                <strong>Resolved:</strong> ${resolved ? "<i class='fa-solid fa-square-check text-success' style='font-size: 16px;'></i>" : "<i class='fa-solid fa-square-xmark text-danger' style='font-size: 16px;'></i>"}<br/>
                <strong>False Alarm:</strong> ${false_alarm ? "<i class='fa-solid fa-square-check text-success' style='font-size: 16px;'></i>" : "<i class='fa-solid fa-square-xmark text-danger' style='font-size: 16px;'></i>"}<br/>
                <strong>Sent UTC:</strong> ${sentTimestamp}<br/>
                <strong>Updated At:</strong> ${cleanUpdatedAt}<br/>
                <strong>Updated By:</strong> ${updated_by || "-"}<br/>
                <div class='text-center'>
                    <button onclick="showCrisisClassificationModal('${id}', '${incident_id}', '${description}', '${resolved}', '${false_alarm}')" class="btn-sm btn-primary" style="width: 100%">Edit Incident</button>
                </div>
            </div>
        `;
        modelInstance.addTooltip(tooltipHTML, true, 20);
        modelInstance.userData.tooltipElementId = tooltipElementId;

        // Add the model to the map
        tb.add(modelInstance);

        // Store the model instance by incident_id
        crisisIncidentObjects[incident_id] = modelInstance;
    });
}



// Helper function to render detections_summary as a table
function renderDetectionsSummaryTable(detections_summary) {
    if (!detections_summary) return "-";
    let summaryObj = detections_summary;
    if (typeof detections_summary === "string") {
        try {
            summaryObj = JSON.parse(detections_summary);
        } catch (e) {
            return detections_summary; // fallback to raw string
        }
    }
    if (typeof summaryObj !== "object" || Array.isArray(summaryObj)) return "-";
    let rows = Object.entries(summaryObj)
        .filter(([key, value]) => value !== 0) // Skip rows where value is 0
        .map(([key, value]) => `<tr><td>${key}</td><td>${value}</td></tr>`)
        .join("");
    return `<table class="table-bordered" style="width: 100%"><thead><tr><th colspan='2'>Detections Summary</th></tr></thead><tbody>${rows}</tbody></table>`;
}


// // Function to edit a model's description or properties
// function editCrisisIncident(incident_id, newProperties) {
//     const modelInstance = crisisIncidentObjects[incident_id];
//     if (modelInstance) {
//         // Modify the properties of the model instance
//         Object.assign(modelInstance.userData, newProperties); // Update userData with new properties
        
//         // Optionally, update tooltip or other properties here
//         const tooltip = document.getElementById(modelInstance.userData.tooltipElementId);
//         if (tooltip) {
//             tooltip.innerHTML = `
//                 <strong>Incident ID:</strong> ${incident_id}<br/>
//                 <strong>Sender:</strong> ${newProperties.sender || modelInstance.userData.sender}<br/>
//                 <strong>Type:</strong> ${newProperties.incident_type || modelInstance.userData.incident_type}<br/>
//                 <strong>Severity:</strong> ${newProperties.severity_level || modelInstance.userData.severity_level}<br/>
//                 <strong>Sent UTC:</strong> ${newProperties.sent_utc || modelInstance.userData.sent_utc}<br/>
//                 <strong>Action Type:</strong> ${newProperties.action_type || modelInstance.userData.action_type}<br/>
//                 <strong>Time:</strong> ${newProperties.timestamp || modelInstance.userData.timestamp}<br/>
//                  <button onclick="handleAddDescription('${incident_id}')">Add Description</button>
//                 `;
//         }
//     }
// }

// // Function to delete a model from the map
// function deleteCrisisIncident(incident_id) {
//     const modelInstance = crisisIncidentObjects[incident_id];
//     if (modelInstance) {
//         // Remove the model from the map (scene)
//         tb.remove(modelInstance);

//         // Remove the model from the stored objects
//         delete crisisIncidentObjects[incident_id];
//     }
// }

// Tooltip click handler to display the tooltip on the screen when an object is clicked
function handleTooltipClick(e) {
    const object = e.intersects[0]?.object;

    if (object && object.userData?.tooltipElementId) {
        const tooltip = document.getElementById(object.userData.tooltipElementId);
        if (tooltip) {
            tooltip.style.display = 'block';
            tooltip.style.position = 'absolute';
            tooltip.style.left = `${e.domEvent.clientX + 10}px`;
            tooltip.style.top = `${e.domEvent.clientY + 10}px`;
            tooltip.style.background = 'rgba(0,0,0,0.8)';
            tooltip.style.color = '#fff';
            tooltip.style.padding = '8px';
            tooltip.style.borderRadius = '5px';
            tooltip.style.zIndex = 9999;
            tooltip.style.textAlign = 'left';
        }
    }
}



//crisis classification
function showCrisisClassificationModal(id, incidentId, description, resolved, falseAlarm) {
    document.getElementById("editCrisisClassificationId").value = id;
    document.getElementById("editCrisisClassificationIncidentId").value = incidentId;
    // console.log(description);
    
    document.getElementById("editCrisisClassificationDescription").value = (description != "null") ? description : "";
    document.getElementById("editCrisisClassificationResolved").checked = (resolved == 1);
    document.getElementById("editCrisisClassificationFalseAlarm").checked = (falseAlarm == 1);

    document.getElementById("editCrisisClassificationModalTitle").innerText = `Edit Crisis Incident: ${incidentId} (${id})`;
    $("#editCrisisClassificationModal").show();
}


function submitCrisisClassificationModal() {
    console.log("Submitting Edit Crisis Classification Modal Form");
    // const csrf = document.getElementById("csrf").value;
    const id = document.getElementById("editCrisisClassificationId").value;
    const incidentId = document.getElementById("editCrisisClassificationIncidentId").value;
    const incidentDesc = document.getElementById("editCrisisClassificationDescription").value;
    const incidentFalseAlarm = document.getElementById("editCrisisClassificationFalseAlarm").checked;
    const incidentResolved = document.getElementById("editCrisisClassificationResolved").checked;


    // fetch(`/crisis_classification/update/${id}`, {
    //     method: 'POST',
    //     headers: {
    //         'Content-Type': 'application/json',
    //         ...(csrf && { 'X-CSRFToken': csrf }),
    //     },
    //     body: JSON.stringify({
    //         description: incidentDesc,
    //         resolved: incidentResolved,
    //         false_alarm: incidentFalseAlarm,
    //     }),
    //     // credentials: 'include', // Needed if using session authentication
    // })
    //     .then(response => response.json())
    //     .then(data => {
    //         if (data.message) {
    //             alert(data.message);
    //         } else {
    //             alert('Unexpected response');
    //         }
    //     })
    //     .catch(error => {
    //         console.error('Error:', error);
    //     });
      


    let reqData = JSON.stringify({
        "description": incidentDesc,
        "resolved": incidentResolved,
        "false_alarm": incidentFalseAlarm,
    });

    $.ajax({
        type: 'POST',
        url: `/crisis_classification/update/${id}`,
        data: reqData,
        headers: {
            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
        },
        success: function (response) {
            console.log(response)
            $("#editCrisisClassificationModal").hide();
        },
        error: function (error) {
            console.error("Error updating crisis classification:", error);
            // alert(`Error updating crisis classification: ${error.responseText}`);
        }
    });
}
