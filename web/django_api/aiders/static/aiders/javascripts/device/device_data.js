{
    function deviceClickInfoBox(toggleId, deviceId) {
        console.log(toggleId);
        postElementId('GPS ' + deviceId, document.getElementById(toggleId).checked);
        if (document.getElementById(toggleId).checked) {
            let div = document.createElement('div');
            div.id = 'deviceInfoBox' + deviceId;
            div.classList.add('deviceDataBox');
            div.classList.add('overlay-popup');
            div.style.position = 'absolute';
            let offset = getNumberOfOverlayPanels() * 20;
            div.style.top = offset + 'px';
            div.style.left = offset + 'px';            
            document.getElementsByClassName('overlay-section')[0].appendChild(div);
            div.innerHTML =
                `<div> <b> Device: ` +
                deviceId +
                `</b></div>
                        <div><b>Time: </b> <span id='` +
                deviceId +
                `_time'></span></div>
                        <div><b>Operator: </b><span id='` +
                deviceId +
                `_operator'></span></div>
                        <div><b>Latitude: </b><span id='` +
                deviceId +
                `_latitude'></span></div>
                        <div><b>Longitude: </b><span id='` +
                deviceId +
                `_longitude'></span></div>
                        <div><b>Altitude: </b><span id='` +
                deviceId +
                `_altitude'></span></div>
                        <div><b>Bearing: </b><span id='` +
                deviceId +
                `_heading'></span></div>
                        <div><b>Battery: </b><span id='` +
                deviceId +
                `_battery_percentage'></span></div>`;
            jQuery(div).draggable();
        } else {
            document.getElementById('deviceInfoBox' + deviceId).remove();
        }
    }
    function displayDeviceData(deviceId) {
        let timestamp = get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.time;
        
        let date = new Date(timestamp);
        let formattedTime = date.getHours().toString().padStart(2, '0') + ":" +
            date.getMinutes().toString().padStart(2, '0') + ":" +
            date.getSeconds().toString().padStart(2, '0');	

        document.getElementById(deviceId + '_time').textContent = formattedTime
        document.getElementById(deviceId + '_operator').innerHTML = get_all_device_info_array().find(
            (device) => device.deviceID === deviceId
        ).deviceInfo.operator;
        document.getElementById(deviceId + '_battery_percentage').innerHTML =
            get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.currentBatteryLevel + '%';
        document.getElementById(deviceId + '_latitude').innerHTML =
            parseFloat(get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.currentCoordinate[1]).toFixed(6) + '°';
        document.getElementById(deviceId + '_longitude').innerHTML =
            parseFloat(get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.currentCoordinate[0]).toFixed(6) + '°';
        document.getElementById(deviceId + '_altitude').innerHTML =
            parseFloat(get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.altitude).toFixed(1) + ' m';
        document.getElementById(deviceId + '_heading').innerHTML =
            parseFloat(get_all_device_info_array().find((device) => device.deviceID === deviceId).deviceInfo.heading).toFixed(1) + '°';
    }
}
