{
    function activate_water_sampler(droneID) {
        postElementId('Collect water ' + droneID, 'Click');
        $.ajax({
            type: 'POST',
            url: dutils.urls.resolve('water_collection_activated', { operation_name: CURRENT_OP }),
            data: { drone_id: droneID },
            headers: {
                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
            },
            success: function (response) {
                if (response === '200') {
                    create_popup_for_a_little(SUCCESS_ALERT, 'Water sampler valve is opening.', 3000);
                } else {
                    create_popup_for_a_little(WARNING_ALERT, 'There is a problem with water sampler valve.', 3000);
                }
            },
        });
    }
}
