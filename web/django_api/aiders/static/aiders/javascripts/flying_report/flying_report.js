{
    $('#id_start_date_time').datepicker({ minDate: 0 });
    $('#form_selection').change(function () {
        if ($('#form_selection').val() != 'custom') {
            $('#id_latitude').prop('readonly', true);
            $('#id_longitude').prop('readonly', true);
            available_drones.forEach((drone) => {
                if (drone.drone_name === $('#form_selection').val()) {
                    $('#id_latitude').val(drone.latitude);
                    $('#id_longitude').val(drone.longitude);
                }
            });
        } else {
            $('#id_latitude').prop('readonly', false);
            $('#id_longitude').prop('readonly', false);
        }
    });

    $('#form_selection').change();
    function default_data_form() {
        $('#id_altitude').val(120);
        $('#id_radius').val(1000);
        $('#id_buffer_altitude').val(50);
        $('#id_buffer_radius').val(50);
    }
    function clear_form() {
        $('#id_latitude').val('');
        $('#id_longitude').val('');
        $('#id_altitude').val('');
        $('#id_radius').val('');
        $('#id_buffer_altitude').val('');
        $('#id_buffer_radius').val('');
        $('#id_start_date_time').val('');
        $('#id_end_date_time').val('');
        $('#form_selection').val('custom');
    }
    $('#sidebarToggle').click(function () {
        document.querySelectorAll('.col3').forEach((element) => {
            if (element.style.display == 'block') {
                element.style.display = 'none';
            } else {
                element.style.display = 'block';
            }
        });
    });
}
