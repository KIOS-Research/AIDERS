function edit_user_operation(username, operation_list) {
    operation_list = operation_list.split(' operation join');
    edit_all_groups = all_groups.split(',');
    edit_all_groups = subtract_two_object_arrays(edit_all_groups, operation_list);
    edit_operation_dialog(
        'save',
        'cancel',
        operation_list,
        edit_all_groups,
        'Select the operation so user ' + username + ' can have access.'
    ).then(function (selected_operation) {
        data = '';
        selected_operation.forEach((element) => {
            data = data + element + ',';
        });
        console.log(data);
        $.ajax({
            url: dutils.urls.resolve('manage_permissions_user', { user_name: username }),
            method: 'POST',
            headers: {
                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
            },
            data: { selected: data },
            cache: false,
            success: function (
                response //Get the build map periods from API
            ) {
                window.location.reload();
            },
        });
    });
}
function edit_operation_dialog(okButton, cancelButton, selected, not_selected, dialogTitle) {
    html_string = `<br><div class="container d-inline-block d-flex justify-content-center"  >
    <div class="row">
        <div class="col h-75" >
            <p class= "text-center my-0">ALL</p>
            <select name="all_list in" id="all_list" class="form-control formcls h-75" size="13" multiple="multiple">`;
    not_selected.forEach((element) => {
        if (element != '') {
            html_string = html_string + '<option value="' + element + '">' + element + '</option>';
        }
    });

    html_string =
        html_string +
        `</select>
        </div>
        <div class="d-grid gap-2 col-2 mx-auto pt-5" style="height: 150px;">
            <button type="button" id="all_list_rightAll" class="btn btn-primary btn-block"><i class="fa-solid fa-angles-right"></i></button>
            <button type="button" id="all_list_rightSelected" class="btn btn-success btn-block"><i class="fa-solid fa-angle-right"></i></i></button>
            <button type="button" id="all_list_leftSelected" class="btn btn-success btn-block"><i class="fa-solid fa-angle-left"></i></i></button>
            <button type="button" id="all_list_leftAll" class="btn btn-primary btn-block"><i class="fa-solid fa-angles-left"></i></button>
        </div>
        <div class="col h-75" style="height: 230px">
            <p class= "text-center my-0">ALLOWED</p>
            <select name="all_list out" id="all_list_to" class="form-control formcls h-75" size="13" multiple="multiple">`;
    selected.forEach((element) => {
        if (element != '') {
            html_string = html_string + '<option value="' + element + '">' + element + '</option>';
        }
    });
    html_string =
        html_string +
        `</select>
        </div>
    </div>
    </div>
    <script>
    $(document).ready(function() {

        $('#all_list').multiselect({
            search: {
    
                left: '<input type="text" name="q" autocomplete="off" class="form-control" placeholder="Search..." />',
    
                right: '<input type="text" name="q" autocomplete="off" class="form-control" placeholder="Search..." />',
    
                }
        });
    });
    </script>`;
    let defer = $.Deferred();
    $('#fade').css('display', 'inline-block');
    let dialog = $('<div></div>')
        .appendTo('body')
        .html(html_string)
        .dialog({
            autoOpen: false,
            height: 'auto',
            width: 600,
            modal: true,
            title: dialogTitle,
            buttons: [
                {
                    text: okButton,
                    click: function () {
                        data_list = [];
                        $('#all_list_to option').each(function () {
                            data_list.push(this.value);
                        });
                        defer.resolve(data_list);
                        $('#fade').css('display', 'none');
                        $(this).remove();
                    },
                },
                {
                    text: cancelButton,
                    click: function () {
                        defer.resolve(false);
                        $('#fade').css('display', 'none');
                        $(this).remove();
                    },
                },
            ],
            close: function () {
                $('#fade').css('display', 'none');
                $(this).remove();
            },
        });
    dialog.dialog('open');
    return defer.promise();
}
