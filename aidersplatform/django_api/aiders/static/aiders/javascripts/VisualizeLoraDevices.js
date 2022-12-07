{
    let loraNames = []

    //This interval is responsible for maintaining a list that contains all online lora devices
    setInterval(function () {
        let lora_url = dutils.urls.resolve('lora_devices', {operation_name: CURRENT_OP})
        //Check from database the online lora devices at this moment
        $.get(lora_url, function (online_lora_devices) {
            // console.log("Online lora devices: ", online_lora_devices)

            //For each online lora device, check if we already have the lora device name
            //here locally in an array. If not, add it
            online_lora_devices.forEach((lora, index) => {
                if (loraNames.indexOf(lora['tagName']) === -1) {
                    loraNames.push(lora['tagName'])
                }
            })

        })
    }, 5000)


    //This interval is responsible for updating on the map, the location of each lora device
    setInterval(function () {
        loraNames.forEach((lora_name, index) => {
            let lora_info_url = dutils.urls.resolve('lora_live_info', {
                operation_name: CURRENT_OP,
                lora_device_name: lora_name
            })

            $.get(lora_info_url, function (live_info) {
                console.log("LIVE LORA INFO: ", live_info)
                let lngLat = new maplibregl.LngLat(live_info['lon'], live_info['lat'])
                let el = document.createElement('div');
                el.id = 'temp' + lora_name
                el.className = 'marker';
                let color
                color = "background-color:" + '#003cff' + ";"
                let test = '<span style=' + color + '><b>'
                el.innerHTML = test + (lora_name) + '</b></span>'


               clickMarker1 = new maplibregl
                .Marker(el)
                .setLngLat(lngLat)
                .addTo(map);



            })
        })

    }, 1000)
}