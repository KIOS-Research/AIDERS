{
    let weatherInterval;
    function weatherClickInfoBox(toggleButtonID) {
        postElementId('Weather Info', $(`#${toggleButtonID}`).is(':checked'));
        if ($(`#${toggleButtonID}`).is(':checked')) {
            let div = document.createElement('div');
            div.id = 'weatherDataBox';
            div.classList.add('weatherDataBox');
            div.classList.add('overlay-popup');
            div.style.position = 'absolute';
            let offset = getNumberOfOverlayPanels() * 20;
            div.style.top = offset + 'px';
            div.style.left = offset + 'px';            
            document.getElementsByClassName('overlay-section')[0].appendChild(div);
            div.innerHTML = `<div> <b> Weather Station Platform</b></div>
                    <div><b>Time: </b> <span id='weatherDataBoxtime1'></span></div>
                    <div><b>Wind Direction: </b><span id='weatherDataBoxwDir'></span></div>
                    <div><b>Wind Speed: </b><span id='weatherDataBoxwSpeed'></span></div>
                    <div><b>Temperature: </b><span id='weatherDataBoxtemp'></span></div>
                    <div><b>Pressure: </b><span id='weatherDataBoxpress'></span></div>
                    <div><b>Humidity: </b><span id='weatherDataBoxhumidity'></span></div>
                    <div><b>Heading: </b><span id='weatherDataBoxheading'></span></div>`;
            jQuery(div).draggable();
            weatherInterval = setInterval(function () {
                presentWeatherData();
            }, WEATHER_UPDATE_INTERVAL);
            function presentWeatherData() {
                let weatherObj = {};
                weatherObj.time = g_websocketMessage['weather_station']['time'];
                weatherObj.windDir = parseFloat(g_websocketMessage['weather_station']['wind_direction']).toFixed(2);
                weatherObj.windSpeed = parseFloat(g_websocketMessage['weather_station']['wind_speed']).toFixed(3);
                weatherObj.temp = parseFloat(g_websocketMessage['weather_station']['temperature']).toFixed(2);
                weatherObj.pressure = parseFloat(g_websocketMessage['weather_station']['pressure']).toFixed(2);
                weatherObj.humidity = parseFloat(g_websocketMessage['weather_station']['humidity']).toFixed(2);
                weatherObj.heading = parseFloat(g_websocketMessage['weather_station']['heading']).toFixed(3);
                displayWeatherData(weatherObj, 'weatherDataBox', true);
                weatherObj = {};
            }
        } else {
            document.getElementById('weatherDataBox').remove();
            clearInterval(weatherInterval);
        }
    }

    function displayWeatherData(weatherObj, element, platform) {
        if (weatherObj.drone !== undefined) {
            document.getElementById(element + 'drone').textContent = weatherObj.drone;
        }
        document.getElementById(element + 'time1').textContent = weatherObj.time;
        document.getElementById(element + 'wDir').textContent = weatherObj.windDir + ' \u00B0';
        document.getElementById(element + 'wSpeed').textContent = weatherObj.windSpeed + ' m/s';
        document.getElementById(element + 'temp').textContent = weatherObj.temp + ' \u2103';
        document.getElementById(element + 'press').textContent = weatherObj.pressure + ' Pa';
        document.getElementById(element + 'humidity').innerHTML = weatherObj.humidity + ' g.m<sup>-</sup><sup>3</sup>';
        document.getElementById(element + 'heading').textContent = weatherObj.heading + ' \u00B0';
    }
}
