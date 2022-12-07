{
    function activate_water_sampler(droneID) {
        $.ajax({
            type: "POST",
            url:  dutils.urls.resolve('water_collection_activated', {operation_name:CURRENT_OP}),
            data: {'drone_id': droneID},
            headers: {
                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value
            },
            success: function (response){
                if(response==='Sending message to drone.'){
                    create_popup_for_a_little(SUCCESS_ALERT,response,3000)
                }else{
                    create_popup_for_a_little(WARNING_ALERT,response,3000)
                }
            }
        })
    }
}