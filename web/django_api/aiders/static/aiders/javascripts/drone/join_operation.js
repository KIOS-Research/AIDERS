{
    $('.open-join-operation-modal').click(function (e)
    {
        // var CSRFtoken = $('input[name=csrfmiddlewaretoken]').val();
        //
        // // var data = document.getElementById("id_new_list").value;
        //
        // $.post($(this).attr("data-url"), {
        //     test: "teeestt",
        //     csrfmiddlewaretoken: CSRFtoken
        // });

         // "<a href=\"{% url 'new_operation' %}\"><button type=\"button\" class=\"btn btn-warning btn-sm join-operation\" style=\"font-size: 0.8em;\">Join</button></a>\n"
        let okBtn = "OK"
        let cancelBtn = "Cancel"
        let message = "You are about to join an operation.<br> Provide the Operation Code you wish to join"
        let dialogTitle = "Join Operation"
        let opName = ""
        let joinOpUrl = $(this).attr("data-url")
        let opNotFound = true

        create_operation_join_dialog(okBtn,cancelBtn,message,dialogTitle).then(function (willProceedAndOpName)
        {
            let canProceed = willProceedAndOpName[0]

            if (canProceed)
            {
                let opName = willProceedAndOpName[1]
                $.ajax({
                    headers: {'X-CSRFToken': document.getElementById('csrf').querySelector('input').value},
                    type: "POST",
                    url: joinOpUrl,
                    data: {
                        operation_name: opName,
                    },
                    success: function (data)
                    {

                    },
                    error: function (jqXHR, textStatus, errorThrown)
                    {
                        if(jqXHR.status === 404 || errorThrown === 'Not Found')
                        {
                            alert(`No active operation found with the name: ${opName}`)
                            // create_popup_for_a_little(WARNING_ALERT,`No active operation found with the name: ${opName}`,3000)
                        }

                    },
                })
            }
        });





    });
}