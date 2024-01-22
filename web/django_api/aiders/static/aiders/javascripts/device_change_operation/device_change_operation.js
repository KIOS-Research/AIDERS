{
    function edit_device_operation(name) {
        $.ajax({
            url: dutils.urls.resolve('device_modify_operation', { device_name: name }),
            cache: false,
            success: function (
                response //Get the build map periods from API
            ) {
                response = JSON.parse(response);
                selected = ' ';
                let list_of_operation_names = [];
                response.forEach(function (operation) {
                    console.log(operation);
                    list_of_operation_names.push(operation['fields']['operation_name']);
                    if ('Selected' in operation['fields']) {
                        selected = operation['fields']['operation_name'];
                    }
                });
                let okButton = 'Submit';
                let cancelButton = 'Cancel';
                let dialogTitle = 'Select Operation to Join.';
                create_operation_dialog(okButton, cancelButton, selected, list_of_operation_names, dialogTitle).then(function (selected_operation) {
                    $.ajax({
                        url: dutils.urls.resolve('device_modify_operation', { device_name: name }),
                        method: 'POST',
                        headers: {
                            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                        },
                        data: { operation_name: selected_operation },
                        cache: false,
                        success: function (response) {
                            if (response == name) {
                                location.reload();
                            }
                        },
                    });
                });
            },
        });
    }
    function start_device_session(name) {
        $.ajax({
            url: dutils.urls.resolve('device_new_session', { device_name: name }),
            cache: false,
            success: function (response) {},
        });
    }
    function stop_device_session(name) {
        $.ajax({
            url: dutils.urls.resolve('device_stop_session', { device_name: name }),
            cache: false,
            success: function (response) {},
        });
    }
}
