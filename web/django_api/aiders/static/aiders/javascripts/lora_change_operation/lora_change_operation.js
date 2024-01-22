function Edit_lora_operation(lora_name) {
    $.ajax({
        url: dutils.urls.resolve('balora_modify_operation', { lora_name: lora_name }),
        cache: false,
        success: function (
            response //Get the build map periods from API
        ) {
            response = JSON.parse(response);
            selected = ' ';
            let list_of_operation_names = [];
            response.forEach(function (operation) {
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
                    url: dutils.urls.resolve('balora_modify_operation', { lora_name: lora_name }),
                    method: 'POST',
                    headers: {
                        'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
                    },
                    data: { operation_name: selected_operation },
                    cache: false,
                    success: function (
                        response //Get the build map periods from API
                    ) {
                        if (response == 'lora_name') {
                            location.reload();
                        }
                    },
                });
            });
        },
    });
}
