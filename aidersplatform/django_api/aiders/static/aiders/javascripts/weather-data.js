{
    let weatherInterval
    function weatherClickInfoBox(toggleButtonID)
    {
        let clickInfoBox = $("#weatherDataBox")
        clickInfoBox.toggle();
        let pressed = $(`#${toggleButtonID}`).is(':checked');
        postElementId(toggleButtonID, pressed)
        if (pressed)
        {
            clickInfoBox.draggable();
            weatherInterval = setInterval( function() { presentWeatherData(); }, WEATHER_UPDATE_INTERVAL );
            function presentWeatherData()
            {

                let weatherObj = {};
                weatherObj.time = webSocketMessage['weather_station']["current_time"]
                weatherObj.windDir =  parseFloat(webSocketMessage['weather_station']["wind_direction"]).toFixed(2)
                weatherObj.windSpeed = parseFloat(webSocketMessage['weather_station']["wind_speed"]).toFixed(3);
                weatherObj.temp = parseFloat(webSocketMessage['weather_station']["temperature"]).toFixed(2);
                weatherObj.pressure = parseFloat(webSocketMessage['weather_station']["pressure"]).toFixed(2);
                weatherObj.humidity = parseFloat(webSocketMessage['weather_station']["humidity"]).toFixed(2);
                weatherObj.heading = parseFloat(webSocketMessage['weather_station']["heading"]).toFixed(3);
                displayWeatherData(weatherObj, 'weatherDataBox')
                weatherObj={}
            }
        }
        else
        {
            clearInterval(weatherInterval)
        }
    }


    function displayWeatherData(weatherObj, element)
    {
        let droneDisplay
        if(weatherObj.drone!== undefined){
            droneDisplay = document.getElementById(element+"drone")
        }
        let timeDisplay = document.getElementById(element+'time1')
        let windDirDisplay = document.getElementById(element+'wDir');
        let wSpeedDisplay = document.getElementById(element+'wSpeed');
        let tempDisplay = document.getElementById(element+'temp');
        let pressureDisplay = document.getElementById(element+'press');
        let humidityDisplay = document.getElementById(element+'humidity');
        let headingDisplay = document.getElementById(element+'heading');
        if(weatherObj.drone!== undefined){
            droneDisplay.textContent = weatherObj.drone
        }

        timeDisplay.textContent = weatherObj.time
        windDirDisplay.textContent = weatherObj.windDir + " \u00B0";
        wSpeedDisplay.textContent = weatherObj.windSpeed + " m/s";
        tempDisplay.textContent = weatherObj.temp +  " \u2103"
        pressureDisplay.textContent = weatherObj.pressure + " Pa"
        humidityDisplay.innerHTML = weatherObj.humidity + " g.m<sup>-</sup><sup>3</sup>"
        headingDisplay.textContent = weatherObj.heading + " \u00B0"
    }

}