{

    let gpsInterval
    let lora_devices = []
    function startStopGPSReceiver(toggleButtonID)
    {
        let pressed = $(`#${toggleButtonID}`).is(':checked');

        if (pressed)
        {
            startReceivingLoraLocations()
        }
        else
        {
            stopReceivingLoraLocations()
        }
    }
    /*Periodically checks if there are any lora devices available that transmit their locations.
    * If there are, they will be displayed on the map as markers*/

    function startReceivingLoraLocations()
    {
        let tempPopupID = 'tempWaitPopup'
        create_popup(WARNING_ALERT,"Getting ready to start...", tempPopupID)

        let firstTime = true
        let loraMarkerLocations = []
        gpsInterval = setInterval(function ()
        {
            let reqSettings = {
                url :  API_URL_LORA_STATIONS,
                type:'GET',
                headers:{"Content-Type": "application/json"}
            }

            $.ajax(reqSettings).done(function (locations) {
                for (let i = 0; i < locations.length; i++)
                {
                    let lon1 = locations[i]["data"]["coordinates"]["lon"]
                    let lat1 = locations[i]["data"]["coordinates"]["lat"]
                    let tagName =  locations[i]["data"]["tagName"]
                    let lngLat = new maplibregl.LngLat(lon1,lat1)
                    let color = getColor(i);

                    if (!loraDeviceExists(tagName))
                    {
                        lora_devices.push(locations[i])
                    }
                    if (!markerExists(lon1,lat1, loraMarkerLocations))
                    {

                        if (firstTime)
                        {
                            let msg = "Successfully started receiving GPS locations!"
                            create_popup_for_a_little(SUCCESS_ALERT,msg,2000)
                            map.flyTo({center: [  lon1,lat1 ], zoom: 20});
                            firstTime = false
                            removeEl('#' + tempPopupID)
                        }
                        placeMarkerOnMap(lngLat,tagName,color);
                        loraMarkerLocations.push({'tagName':tagName,'location':{'lon':lon1,'lat':lat1}})
                    }

                }
            })
        },2000)
    }

    function stopReceivingLoraLocations()
    {
        clearInterval(gpsInterval)
        let msg = "Stopped receiving GPS locations!"
        create_popup_for_a_little(WARNING_ALERT,msg,2000)
    }
    function placeMarkerOnMap(lngLat,tagName, color)
    {
        let el = document.createElement('div');
        el.id = 'temp' + tagName
        el.className = 'marker';

        color = "background-color:" +  color + ";"
        let test = '<span style=' + color + '><b>'
        el.innerHTML = test + (tagName ) + '</b></span>'
        loraMarker = new maplibregl
            .Marker(el)
            .setLngLat(lngLat)
            .addTo(map)
    }

    function markerExists(lon, lat, loraLocations)
    {
        for (let i = 0; i < loraLocations.length; i++)
        {
            if (lon === loraLocations[i]['location']['lon'] && lat === loraLocations[i]['location']['lat'])
            {
                return true
            }
        }
        return false
    }

    function getLoraDeviceIfExists(str)
    {
        for (let i = 0; i < lora_devices.length; i++)
        {
            if (str.includes(lora_devices[i]['data']['tagName']))
            {
                return lora_devices[i]
            }
        }

        return -1
    }

    function getLoraDevices()
    {
        return lora_devices
    }
    function loraDeviceExists(tagName)
    {
        for (let i = 0; i < lora_devices.length; i++)
        {
            if (tagName === lora_devices[i]["data"]["tagName"])
            {
                return true
            }
        }
        return false
    }
}