function sendMessageToDevice(toggleID, _deviceName) {
    
    var pressed = $("#" + toggleID).is(":checked");
    // console.log(pressed);
    if (pressed) {
        var deviceNotificationDialog = $('#device-not-dialog');
        let defer = $.Deferred();

        // Set the device name in the selected-device div
        $(".selected-device").text(_deviceName);

        // Set the device name in the input field
        const inputSelectedDevice = document.getElementById('selected-device-input');
        inputSelectedDevice.value = '';
        inputSelectedDevice.value = _deviceName;

        const errorMessageElement = document.getElementsByClassName('errorMessage')[0];

        deviceNotificationDialog.dialog({
            autoOpen: false, // Dialog won't open automatically
            modal: true,
            height: 'auto',
            width: 600,
            title: "Device Notification", // Set the title dynamically
            buttons: {
                "Send": function () {
                    // console.log("ishalla");
                    
                    var selectedDevice = [$("#selected-device-input").val()];
                    var sender = $("#username").val();
                    var deviceMessage = $("#device-message").val();

                    console.log(selectedDevice, sender, deviceMessage);
                    

                    // Check if deviceMessage is empty
                    if (!deviceMessage) {
                        errorMessageElement.textContent = "Message cannot be empty.";
                        return; // Stop here if the message is empty
                    } else {
                        errorMessageElement.textContent = ""; // Clear any previous error messages
                    }

                    // Proceed with submitting the message
                    submitDeviceMessage(selectedDevice, deviceMessage, sender);

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
        deviceNotificationDialog.dialog("open");
        return defer.promise();
    }

}


function submitDeviceMessage(selectedDevices, deviceMessage, sender) {
    let postMessage = {
        selectedDevices: selectedDevices,
        sender: sender,
        message: deviceMessage,
    };
    console.log(`postMessage================ ${JSON.stringify(postMessage)}`)
    postDeviceMessage(postMessage).then(function (_response) {

    });
}

function postDeviceMessage(_data) {
    console.log(CURRENT_OP);

    const url = "sendDeviceNotification/";
    let data = postRequest(url, _data);
    return data;
}