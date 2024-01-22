{

    let safeDronesResultsInterval;

    function toggleRiskCalculation(toggleID) {
        console.log("aaa");
        console.log(NET_IP);

        // let riskInfoBox = $("#droneHullBox")
        // riskInfoBox.toggle();
        let pressed = $(`#${toggleID}`).is(':checked');
        if (pressed) {
            let apiCall = {
                "url": "/safeDronesResults",
                "method": "GET",
                "timeout": 0,
                "headers": {"Content-Type": "application/json"}
            };

            createSafeDronesBoxes();

            $.ajax(apiCall).done(function (response) {
                console.log(response);
                // let allRisks = JSON.parse(response);

                /*Update the content of the hull box every 5 seconds*/
                safeDronesResultsInterval = setInterval(function () {
                    $.ajax(apiCall).done(function (response)
                    {
                        // let updatedRisks = JSON.parse(response);
                        updateSafeDronesBoxesContent(response);
                    })

                },5000)
            });


        }
        else {
            let allDrones = get_all_drone_info_array()
            allDrones.forEach(el => removeEl('#droneHullBox_' + el['droneID']))
            clearInterval(safeDronesResultsInterval)

            let apiCall = {
                "url": "/safeDronesStop",
                "method": "GET",
                "timeout": 0,
                "headers": { "Content-Type": "application/json" }
            };
            $.ajax(apiCall).done(function (response) {
                console.log(response);
            });            
        }



        function createSafeDronesBoxes() {
            // let counter = 0;
            let allDrones = get_all_drone_info_array();
            allDrones.forEach( function (drone) {
                let divID ='droneHullBox_' + drone['droneID']
                let color = getHullBoxColor(get_drone_index(drone['droneID']))

                let droneSpanID = 'hullDrone_' + drone['droneID']
                let dateSpanID = 'hullDate_' + drone['droneID']
                let timeSpanID = 'hullTime_' + drone['droneID']
                let motorPfailSpanID = 'hullMotorFail_' + drone['droneID']
                let motorMttfSpanID = 'hullMotorMttf_' + drone['droneID']

                let batteryPfailSpanID = 'hullBatteryFail_' + drone['droneID']
                let batteryMttfSpanID = 'hullBatteryMttf_' + drone['droneID']

                let chipPfailSpanID = 'hullChipFail_' + drone['droneID']
                let chipMTTFSpanID = 'hullChipMttf_' + drone['droneID']

                let gpsPfailSpanID = 'hullGpsFail_' + drone['droneID']
                let gpsMTTFSpanID = 'hullGpsMttf_' + drone['droneID']

                let secondsSpanID = 'hullSeconds_' + drone['droneID']
                let batterySpanID = 'hullBattery_' + drone['droneID']
                let satellitesSpanID = 'hullSatellites_' + drone['droneID']

                // console.log(get_drone_object(risk['drone_name']).droneInfo);
                $(`<div id=${divID} class="droneHullBox" style="background-color: ${color}">\n
                    <div><img class='safe-drones-logo' src="/static/aiders/imgs/SafeDrones_Logo.png"></div>
                    <div> <b>Drone:  <span id=${droneSpanID}>${drone['droneID']}</span> </b> </div>\n
                    <div> <b>Date: <span id=${dateSpanID}>-</span> </b> </div>\n
                    <div><b>Time: <span id=${timeSpanID}>-</span> </b> </div>\n
                    <div><b>Seconds: <span id=${secondsSpanID}>-</span>  </b>  </div>\n
                    <div><b>Battery: <span id=${batterySpanID}>-</span></b>  </div>\n
                    <div><b>Satellites: <span id=${satellitesSpanID}>-</span> <br></br>  </b>  </div>\n
                    
                    <div><b>Motor Failure MTTF:  <span id=${motorMttfSpanID}>-</span> </b> </div>\n
                    <div><b>Motor P_Fail:  <span id=${motorPfailSpanID}>-</span> <br></br> </b> </div>\n
                    
                    <div><b>Battery Failure MTTF:  <span id=${batteryMttfSpanID}> -</span> </b> </div>\n
                    <div><b>Battery P_Fail:  <span id=${batteryPfailSpanID}>-</span> <br></br> </b> </div>\n
                    
                    <div><b>Chip Failure MTTF:  <span id=${chipMTTFSpanID}>-</span> </b> </div>\n
                    <div><b>Chip P_Fail:  <span id=${chipPfailSpanID}>-</span>  <br></br> </b> </div>\n

                    <div><b>GPS Failure MTTF:  <span id=${gpsMTTFSpanID}>-</span> </b> </div>\n
                    <div><b>GPS P_Fail:  <span id=${gpsPfailSpanID}>-</span>  </b> </div>\n

                </div> `).insertAfter($('#map')).draggable();

                // let left = 300 + (counter*300);
                jQuery("#"+divID).css({ "left": "315px", "top": "170px" });
            });
        }

        function updateSafeDronesBoxesContent(allRisks) {
            allRisks.forEach( function (risk) {
                if (get_drone_object(risk['drone_name']) !== -1) {
                    let dateTime = risk['DateTime'].split('T')
                    let date = dateTime[0]
                    let time = dateTime[1]
                    const riskObj = {};
                    riskObj.drone_name =  risk["drone_name"]
                    riskObj.date =  date
                    riskObj.time = time
                    riskObj.motorPFail = risk["motorPfail"];
                    riskObj.motorMTTF = risk["motorMTTF"];
    
                    riskObj.batteryPfail = risk["batteryPfail"];
                    riskObj.batteryMTTF = risk["batteryMTTF"];
    
                    riskObj.chipPfail = risk["chipPfail"];
                    riskObj.chipMTTF = risk["chipMTTF"];
    
                    riskObj.gpsPfail = risk["gpsPfail"];
                    riskObj.gpsMTTF = risk["gpsMTTF"];
    
                    riskObj.seconds = parseInt(risk["Seconds"])
                    riskObj.batteryLevel = get_drone_object(riskObj.drone_name).droneInfo.currentBatteryLevel
                    riskObj.satellites = get_drone_object(riskObj.drone_name).droneInfo.satellites
                    displayRiskData(riskObj)
                }
                else {
                    // console.log('Drone ' + risk['drone_name'] + ' not found!');
                }                

            })
        }


        function displayRiskData(riskObj) {
            let droneDisplay = document.getElementById('hullDrone_'+  riskObj.drone_name);
            if(droneDisplay !== undefined) {
                let dateDisplay = document.getElementById('hullDate_'+  riskObj.drone_name);
                let timeDisplay = document.getElementById('hullTime_'+  riskObj.drone_name);
    
                let motorPfailDisplay = document.getElementById('hullMotorFail_'+  riskObj.drone_name);
                let motorMTFFdisplay = document.getElementById('hullMotorMttf_'+  riskObj.drone_name);
    
                let batteryPfailDisplay = document.getElementById('hullBatteryFail_'+  riskObj.drone_name);
                let batteryMTFFdisplay = document.getElementById('hullBatteryMttf_'+  riskObj.drone_name);
    
                let chipPfailDisplay = document.getElementById('hullChipFail_'+  riskObj.drone_name);
                let chipMTFFdisplay = document.getElementById('hullChipMttf_'+  riskObj.drone_name);
    
                let gpsPfailDisplay = document.getElementById('hullGpsFail_' + riskObj.drone_name);
                let gpsMTFFdisplay = document.getElementById('hullGpsMttf_' + riskObj.drone_name);
    
                let secondsDisplay = document.getElementById('hullSeconds_' +  riskObj.drone_name);
                let batteryDisplay = document.getElementById('hullBattery_' + riskObj.drone_name);
                let satellitesDisplay = document.getElementById('hullSatellites_' + riskObj.drone_name);
    
                droneDisplay.textContent = riskObj.drone_name
                dateDisplay.textContent = riskObj.date
                timeDisplay.textContent = riskObj.time
    
                motorPfailDisplay.textContent = riskObj.motorPFail
                motorMTFFdisplay.textContent = Math.round(riskObj.motorMTTF/60) + ' minutes'
    
                batteryPfailDisplay.textContent = riskObj.batteryPfail
                batteryMTFFdisplay.textContent = Math.round(riskObj.batteryMTTF/60) + ' minutes'

                chipPfailDisplay.textContent = riskObj.chipPfail
                chipMTFFdisplay.textContent = Math.round(riskObj.chipMTTF/60) + ' minutes'
    
                gpsPfailDisplay.textContent = riskObj.gpsPfail
                gpsMTFFdisplay.textContent = Math.round(riskObj.gpsMTTF/60) + ' minutes'
    
                secondsDisplay.textContent = riskObj.seconds
                batteryDisplay.textContent = riskObj.batteryLevel + '%'
                satellitesDisplay.textContent = riskObj.satellites
    
                document.getElementById('droneHullBox_' + riskObj.drone_name).style["backgroundColor"] =getHullBoxColor(get_drone_index(riskObj['drone_name']))

                // popups
                // if (riskObj.chipPfail > 0.5) {
                //     create_popup_for_a_little(FAILED_ALERT, "SafeDrones: the chip of drone " + riskObj.drone_name + " has a high probability to fail!", 5000);
                // }
                if (riskObj.gpsPfail > 0.6) {
                    if (riskObj.gpsPfail > 0.8) {
                        create_popup_for_a_little(FAILED_ALERT, "SafeDrones: CRITICAL GPS WARNING FOR " + riskObj.drone_name + ". PLEASE LAND IMMEDIATELY!", 5000);
                    }
                    else {
                        create_popup_for_a_little(FAILED_ALERT, "SafeDrones: the GPS of drone " + riskObj.drone_name + " has a high probability to fail!", 5000);
                    }
                }                
                if (riskObj.batteryPfail > 0.6) {
                    if (riskObj.batteryPfail > 0.8) {
                        create_popup_for_a_little(FAILED_ALERT, "SafeDrones: CRITICAL BATTERY WARNING FOR " + riskObj.drone_name + ". PLEASE LAND IMMEDIATELY!", 5000);
                    }
                    else {
                        create_popup_for_a_little(FAILED_ALERT, "SafeDrones: the battery of drone " + riskObj.drone_name + " has a high probability to fail!", 5000);
                    }
                }
            }
            else {
                console.log('Element hullDrone_' + riskObj.drone_name + ' not found!');
            }
        }
      
    }



}