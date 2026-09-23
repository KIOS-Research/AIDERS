function sendMessageToPilot(toggleID, _droneName) {
    var pressed = $("#" + toggleID).is(":checked");
    if (pressed) {
        var pilotNotificationDialog = $('#pilot-not-dialog');
        let defer = $.Deferred();

        // Set the drone name in the selected-drone div
        $(".selected-drone").text(_droneName);

        // Set the drone name in the input field
        const inputSelectedDrone = document.getElementById('selected-drone-input');
        inputSelectedDrone.value = '';
        inputSelectedDrone.value = _droneName;

        const errorMessageElement = document.getElementsByClassName('errorMessage')[0];

        pilotNotificationDialog.dialog({
            autoOpen: false, // Dialog won't open automatically
            modal: true,
            height: 'auto',
            width: 600,
            title: "Pilot Notification", // Set the title dynamically
            buttons: {
                "Send": function () {
                    var selectedDrone = [$("#selected-drone-input").val()];
                    var sender = $("#username").val();
                    var pilotMessage = $("#message").val();

                    // Check if pilotMessage is empty
                    if (!pilotMessage) {
                        errorMessageElement.textContent = "Message cannot be empty.";
                        return; // Stop here if the message is empty
                    } else {
                        errorMessageElement.textContent = ""; // Clear any previous error messages
                    }

                    // Proceed with submitting the message
                    submitMessage(selectedDrone, pilotMessage, sender);

                    // Clear the message field
                    $("#message").val('');

                    $(this).dialog("close");

                    $("#" + toggleID).bootstrapToggle("off");
                },
                "Cancel": function () {
                    defer.resolve([false]);
                    $(this).dialog("close");
                    // Set toggle back to off
                    $("#" + toggleID).bootstrapToggle("off");
                }
            },
            close: function () {
                defer.resolve([false]);
                $("#" + toggleID).bootstrapToggle("off");
            }
        });
        pilotNotificationDialog.dialog("open");
        return defer.promise();
    }

}

function sendMessageToAll() {
    let selectedDones = get_selected_drones();

    if (selectedDones.length === 0) {
        showPopupForALittle("#noDroneBox", "", 2000);
    } else {
        var pilotNotificationDialog = $('#pilot-not-dialog');
        let defer = $.Deferred();

        // Set the drone name in the selected-drone div
        let droneNameList = '';
        selectedDones.forEach(function (drone) {
            droneNameList = droneNameList + drone.droneID + ',';
        });
        // Convert droneNameList to an array
        let droneIDsArray = droneNameList.split(',').filter(drone => drone.trim() !== '');

        const droneList = document.querySelector('.selected-drone');
        droneList.innerHTML = '';

        droneIDsArray.forEach(drone => {
            const div = document.createElement('div');

            // Create FontAwesome icon element
            const icon = document.createElement('i');
            icon.className = 'fa fa-check';

            // Add icon and text content to the div
            div.appendChild(icon);
            div.appendChild(document.createTextNode(` ${drone}`));

            droneList.appendChild(div);
        });

        // Remove the trailing comma from droneNameList
        droneNameList = droneNameList.slice(0, -1);

        // Set the drone name in the input field
        const inputSelectedDrone = document.getElementById('selected-drone-input');
        inputSelectedDrone.value = '';
        inputSelectedDrone.value = droneNameList;

        const errorMessageElement = document.getElementsByClassName('errorMessage')[0];

        pilotNotificationDialog.dialog({
            autoOpen: false, // Dialog won't open automatically
            modal: true,
            height: 'auto',
            width: 600,
            title: "Notify All", // Set the title dynamically
            buttons: {
                "Send": function () {
                    var selectedDrone = $("#selected-drone-input").val();
                    // explode selectedDrone into an array
                    var selectedDrones = selectedDrone.split(',').filter(drone => drone.trim() !== '');
                    var sender = $("#username").val();
                    var pilotMessage = $("#message").val();

                    // Check if pilotMessage is empty
                    if (!pilotMessage) {
                        errorMessageElement.textContent = "Message cannot be empty.";
                        return; // Stop here if the message is empty
                    } else {
                        errorMessageElement.textContent = ""; // Clear any previous error messages
                    }

                    // Proceed with submitting the message
                    submitMessage(selectedDrones, pilotMessage, sender);

                    // Clear the message field
                    $("#message").val('');

                    $(this).dialog("close");
                },
                "Cancel": function () {
                    defer.resolve([false]);
                    $(this).dialog("close");
                    // Set toggle back to off
                }
            },
            close: function () {
                defer.resolve([false]);
            }
        });
        pilotNotificationDialog.dialog("open");
        return defer.promise();
    }
}

function submitMessage(selectedDrones, pilotMessage, sender) {
    let postMessage = {
        selectedDrones: selectedDrones,
        sender: sender,
        message: pilotMessage,
    };
    console.log(`postMessage================ ${JSON.stringify(postMessage)}`)
    postPilotMessage(postMessage).then(function (_response) {

    });
}

function postPilotMessage(_data) {
    console.log(CURRENT_OP);

    const url = "sendPilotNotification/";
    let data = postRequest(url, _data);
    return data;
}