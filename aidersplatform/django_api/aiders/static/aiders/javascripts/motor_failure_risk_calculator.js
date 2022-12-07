{


    function toggleRiskCalculation(toggleID)
    {
        // let riskInfoBox = $("#droneHullBox")
        // riskInfoBox.toggle();
        let pressed = $(`#${toggleID}`).is(':checked');
        let hullBoxIntervl ;
        if (pressed)
        {
            let settings = {
                "url": API_URL_MOTOR_FAILURE_RISK,
                "method": "GET",
                "timeout": 0,
                "headers": {"Content-Type": "application/json"}
            };

            $.ajax(settings).done(function (response) {
                console.log(response);
                let allRisks = response

                createHullBoxes(allRisks)

                /*Update the content of the hull box every 5 seconds*/
                hullBoxIntervl = setInterval(function ()
                {
                    $.ajax(settings).done(function (updatedRisks)
                    {
                        updateHullBoxesContent(updatedRisks)
                    })

                },5000)



            });


        }
        else
        {
            let allDrones = get_all_drone_info_array()
            allDrones.forEach(el => removeEl('#droneHullBox_' + el['droneID']))
            clearInterval(hullBoxIntervl)
        }



        function createHullBoxes(allRisks)
        {
            allRisks.forEach( function (risk)
                {
                    let dateTime = risk['DateTime'].split(' ')
                    let date = dateTime[0]
                    let time = dateTime[1]
                    let divID ='droneHullBox_' + risk['DroneID']
                    let color = getHullBoxColor(get_drone_index(risk['DroneID']))

                    let droneSpanID = 'hullDrone_' + risk['DroneID']
                    let dateSpanID = 'hullDate_' + risk['DroneID']
                    let timeSpanID = 'hullTime_' + risk['DroneID']
                    let motorPfailSpanID = 'hullMotorFail_' + risk['DroneID']
                    let motorMttfSpanID = 'hullMotorMttf_' + risk['DroneID']

                    let batteryPfailSpanID = 'hullBatteryFail_' + risk['DroneID']
                    let batteryMttfSpanID = 'hullBatteryMttf_' + risk['DroneID']

                    let chipPfailSpanID = 'hullChipFail_' + risk['DroneID']
                    let chipMTTFSpanID = 'hullChipMttf_' + risk['DroneID']

                    let secondsSpanID = 'hullSeconds_' + risk['DroneID']
                    let batterySpanID = 'hullBattery_' + risk['DroneID']
                    $(`<div id=${divID} class="droneHullBox" style="background-color: ${color}">\n
                        <div> <b>Drone:  <span id=${droneSpanID}> ${risk['DroneID']} </span> </b> </div>\n
                        <div> <b>Date: <span id=${dateSpanID}>  ${date}   </span> </b> </div>\n
                        <div><b>Time: <span id=${timeSpanID}>  ${time}  </span> </b> </div>\n
                        <div><b>Seconds: <span id=${secondsSpanID}>  ${risk['Seconds']}  </span>  </b>  </div>\n
                        <div><b>Battery: <span id=${batterySpanID}>  ${get_drone_object(risk['DroneID']).droneInfo.currentBatteryLevel}%  </span> <br></br>  </b>  </div>\n
                        
                        <div><b>Motor Failure MTTF:  <span id=${motorMttfSpanID}>  ${(risk['motorMTTF']).toFixed(2)} years  </span> </b> </div>\n
                        <div><b>Motor P_Fail:  <span id=${motorPfailSpanID}>  ${(risk['motorPfail']).toFixed(4)}  </span> <br></br> </b> </div>\n
                        
                        <div><b>Battery Failure MTTF:  <span id=${batteryMttfSpanID}>  ${(risk['batteryMTTF']).toFixed(2)} years  </span> </b> </div>\n
                        <div><b>Battery P_Fail:  <span id=${batteryPfailSpanID}>  ${(risk['batteryPfail']).toFixed(4)}   </span> <br></br> </b> </div>\n
                        
                        <div><b>Chip Failure MTTF:  <span id=${chipMTTFSpanID}>  ${(risk['chipMTTF']).toFixed(2)} years  </span> </b> </div>\n
                        <div><b>Chip P_Fail:  <span id=${chipPfailSpanID}>  ${(risk['chipPfail']).toFixed(4)}  </span>  </b> </div>\n
                    </div> `).insertAfter($('#map')).draggable();
                }

            )
        }

        function updateHullBoxesContent(allRisks)
        {
            allRisks.forEach( function (risk)
                {
                    let dateTime = risk['DateTime'].split(' ')
                    let date = dateTime[0]
                    let time = dateTime[1]
                    const riskObj = {};
                    riskObj.droneID =  risk["DroneID"]
                    riskObj.date =  date
                    riskObj.time = time
                    riskObj.motorPFail = parseFloat(risk["motorPfail"]).toFixed(4);
                    riskObj.motorMTTF = parseFloat(risk["motorMTTF"]).toFixed(2);

                    riskObj.batteryPfail = parseFloat(risk["batteryPfail"]).toFixed(4);
                    riskObj.batteryMTTF = parseFloat(risk["batteryMTTF"]).toFixed(2);

                    riskObj.chipPfail = parseFloat(risk["chipPfail"]).toFixed(4);
                    riskObj.chipMTTF = parseFloat(risk["chipMTTF"]).toFixed(2);


                    riskObj.seconds = parseInt(risk["Seconds"])
                    riskObj.batteryLevel =  get_drone_object(riskObj.droneID).droneInfo.currentBatteryLevel
                    displayRiskData(riskObj)

                }

            )
        }
        function displayRiskData(riskObj)
        {
            let droneDisplay = document.getElementById('hullDrone_'+  riskObj.droneID)
            let dateDisplay = document.getElementById('hullDate_'+  riskObj.droneID);
            let timeDisplay = document.getElementById('hullTime_'+  riskObj.droneID);

            let motorPfailDisplay = document.getElementById('hullMotorFail_'+  riskObj.droneID);
            let motorMTFFdisplay = document.getElementById('hullMotorMttf_'+  riskObj.droneID);

            let batteryPfailDisplay = document.getElementById('hullBatteryFail_'+  riskObj.droneID);
            let batteryMTFFdisplay = document.getElementById('hullBatteryMttf_'+  riskObj.droneID);

            let chipPfailDisplay = document.getElementById('hullChipFail_'+  riskObj.droneID);
            let chipMTFFdisplay = document.getElementById('hullChipMttf_'+  riskObj.droneID);

            let secondsDisplay = document.getElementById('hullSeconds_' +  riskObj.droneID);
            let batteryDisplay = document.getElementById('hullBattery_' +  riskObj.droneID);

            droneDisplay.textContent = riskObj.droneID
            dateDisplay.textContent = riskObj.date
            timeDisplay.textContent = riskObj.time

            motorPfailDisplay.textContent = riskObj.motorPFail
            motorMTFFdisplay.textContent = riskObj.motorMTTF + ' years'

            batteryPfailDisplay.textContent = riskObj.batteryPfail
            batteryMTFFdisplay.textContent = riskObj.batteryMTTF + ' years'

            chipPfailDisplay.textContent = riskObj.chipPfail
            chipMTFFdisplay.textContent = riskObj.chipMTTF + ' years'


            secondsDisplay.textContent = riskObj.seconds
            batteryDisplay.textContent = riskObj.batteryLevel + '%'

            document.getElementById('droneHullBox_' + riskObj.droneID).style["backgroundColor"] =getHullBoxColor(get_drone_index(riskObj['DroneID']))
        }
      
    }



}