{
    let isTogglePressed = false

    let clickMarker = new maplibregl.Marker({
        'color': '#314ccd'
    });

    let highestElMarker = new maplibregl.Marker({
        'color': '#f54242'
    });

    let circleLayerID = 'circleLayer'
    let circleSourceID = 'circleSource'
    let clickMarkerID = 'clickMarker'
    let highestElMarkerID = 'highestElMarker'
    map.on('click', function (e)
    {
        let isLocked = $('#lockUnlockIcn').hasClass('fa-lock')
        if (isLocked) return //User wants the yellow circle layer to stay "locked" on the map. Nothing else to do


        if (map.getLayer(circleLayerID)) {
            map.removeLayer(circleLayerID);
        }
        if (map.getSource(circleSourceID)) {
            map.removeSource(circleSourceID);
        }
        highestElMarker.remove();
        if (!isTogglePressed ) //Don't do anything if the toggle button is disabled
        {
            clickMarker.remove();
            // highestElMarker.remove();
            return;
        }





        let bldsDisplay = document.getElementById('blds');
        let rdsDisplay = document.getElementById('rds');



        let radiusInMeters = 1000

        clickMarker.setLngLat(e.lngLat).addTo(map);
        let lng = e.lngLat.lng;

        let lat = e.lngLat.lat;
        let distPerPix = getDistPerPixel()


        let pixelOffset = (radiusInMeters/1000)/distPerPix //To find out how many pixels does 1 km equal to.
        add_circl_layer(circleSourceID,circleLayerID,radiusInMeters,lat,lng)
        let bbox = [
            [e.point.x - pixelOffset, e.point.y - pixelOffset],
            [e.point.x + pixelOffset, e.point.y + pixelOffset]
        ];

        let featuresBldngs = map.queryRenderedFeatures(bbox, {
            layers: [layerBuildings.id]
        });
        let featuresRoads = map.queryRenderedFeatures(bbox, {
            layers: [layerRoads.id]
        });

        updateElementContent(layerRoads.id,featuresRoads.length, rdsDisplay)
        updateElementContent(layerBuildings.id,featuresBldngs.length, bldsDisplay)


        // Run through the selected features and set a filter
        // to match features with unique FIPS codes to activate
        // the `counties-highlighted` layer.
        // let filter = features.reduce(
        //     function (memo, feature)
        //     {
        //         memo.push(feature.properties.objectid);
        //         return memo;
        //     },
        //     ['in', 'objectid']
        // );
        // console.log("FILTER: ")
        // console.log(filter);
        // map.setFilter(layerBuildings.id, filter);
        //
        // let filterRoads = featuresRoads.reduce(
        //     function (memo, feature)
        //     {
        //         memo.push(feature.properties.id);
        //         return memo;
        //     },
        //     ['in', 'id']
        // );
        // map.setFilter(layerRoads.id, filterRoads);


        // console.log("PIXEL OFFSET: " + pixelOffset)
        presentAreaInformation(radiusInMeters, lng, lat, highestElMarkerID);

    });

    /*
    * Toggles the visibility of the information box and the yellow big circle that appears
    * */
    function toggleAreaInfoBoxAndLayer(toggleButtonID)
    {
        let clickInfoBox = $("#clickInfoBox")
        clickInfoBox.toggle();
        isTogglePressed = $("#" + toggleButtonID).is(':checked');
        postElementId(toggleButtonID, isTogglePressed)
        if (isTogglePressed)
        {
            clickInfoBox.draggable();
            highestElMarker._element.hidden = false
            clickMarker._element.hidden = false
            if (map.getLayer(circleLayerID)) {
                map.setLayoutProperty(circleLayerID, 'visibility', 'visible');
            }
        }
        else if(!isClickInfoToggleButtonPressed)
        {
            if (map.getLayer(circleLayerID)) {
                map.setLayoutProperty(circleLayerID, 'visibility', 'none');
            }
            // highestElMarker.remove();
            // clickMarker.remove();
            highestElMarker._element.hidden = true
            clickMarker._element.hidden = true
        }
    }


    /*
    * In case locking is activated, the click area layer (the yellow big circle) stays "locked"
    * on the map. That means, if user tries to click elsewhere on the map, the big yellow circle layer
    * will not appear again on the second location
    * */
    function lockUnlockIcon(iconElem)
    {
        // console.log("CLICKED LOCKED ICON")
        // console.log("THIS IS: ",event)
        // console.log("FIND I: ", $(this).find('i'))
        // $(this).find('i').addClass('fa-unlock');
        //         console.log("FIND I: ", $(this).find('i'))
        let locked = iconElem.classList.contains('fa-lock')
        let unlocked = iconElem.classList.contains('fa-unlock')
        if (locked)
        {
            $(iconElem).removeClass('fa-lock').addClass(' fa-unlock');
        }
        else
        {
             $(iconElem).removeClass('fa-unlock').addClass(' fa-lock');
        }
        // $(".fa-lock").click(function(){
        //       // alert("Icon clicked");
        //     console.log("CLICKED")
        //       $(this).removeClass('fa-lock').addClass(' fa-unlock');
        //
        //       // $(this).find('i').toggleClass('fa-address-book');
        //     });
    }

    function updateElementContent(layer_id, textContent, element)
    {
        if (layerExists(layer_id) && isLayerVisible(layer_id))
        {
            element.textContent =  textContent
        }
        else
        {
            element.textContent = "Load Layer First"
        }
    }
    function getDistPerPixel()
    {
        const clientWidth = window.innerWidth;
        const distance = getDistance();
        const onePixel_km = distance / clientWidth;
        // console.log(`1px is ~${onePixel.toFixed(3)} km or ${Math.ceil((onePixel) * 1000)} meters`);
        // console.log(`1 PX IS: ${onePixel_km} km`)
        return onePixel_km
    }

    function getDistance() {
        const bounds = map.getBounds();
        const topLeft = turf.point([bounds._ne.lng, bounds._ne.lat]);
        const topRight = turf.point([bounds._sw.lng, bounds._ne.lat]);
        const bottomLeft = turf.point([bounds._ne.lng, bounds._sw.lat]);
        const bottomRight = turf.point([bounds._sw.lng, bounds._sw.lat]);

        const middleLeft = turf.midpoint(topLeft, bottomLeft);
        const middleRight = turf.midpoint(topRight, bottomRight);
        const distance = turf.distance(middleLeft, middleRight, {'units':'kilometers'});
        return distance;
    }
    function presentAreaInformation(radius, lng, lat, highestElMarkerID)
    {
        let eleDisplay = document.getElementById('ele');
        let radDisplay = document.getElementById('rad');
        let lngDisplay = document.getElementById('lng');
        let latDisplay = document.getElementById('lat');
1

        if (typeof mapboxgl === "undefined" )
        {
            lngDisplay.textContent = lng.toFixed(6) + " \u00B0";
            latDisplay.textContent = lat.toFixed(6) + " \u00B0";
            eleDisplay.textContent = 'Not Available';
            radDisplay.textContent = ` ${radius} m`
        }
        else
        {
            if (mapboxgl.accessToken !== null)
            {
                  // Make the API request
            // add radius to the request
            let query = 'https://api.mapbox.com/v4/mapbox.mapbox-terrain-v2/tilequery/' + lng + ',' + lat + '.json?layers=contour&limit=50&radius=' + radius + '&access_token=' +  mapboxgl.accessToken;
            $.ajax({
                method: 'GET',
                url: query,
            }).done(function (data)
            {
                // Get all the returned features in radius
                let allFeatures = data.features;
                //console.log(allFeatures);
                // Find the maxumim elevation and store the location
                let max = -1000, imax = -1;
                for (let i = 0; i < allFeatures.length; i++)
                {
                    if (allFeatures[i].properties.ele > max)
                    {
                        max = allFeatures[i].properties.ele;
                        imax = i;
                    }
                }
                let highestElevation = max;
                // Display the largest elevation value

                let maxll = new maplibregl.LngLat(allFeatures[imax].geometry.coordinates[0], allFeatures[imax].geometry.coordinates[1]);
                // markerHighest.setLngLat(maxll).addTo(map);

                // Display the longitude and latitude values
                lngDisplay.textContent = lng.toFixed(6) + " \u00B0";
                latDisplay.textContent = lat.toFixed(6) + " \u00B0";
                eleDisplay.textContent = highestElevation + ' m';
                radDisplay.textContent = ` ${radius} m`

                highestElMarker = create_marker( `${highestElevation}m`,maxll,'#f54242',highestElMarkerID, highestElMarker)
            });
            }

        }

    }

    function create_marker(text, pos, col, marker_id)
    {
        let el = document.createElement('div');
        el.id = marker_id
        el.className = 'marker';
        let color = "background-color:" +  col + ";"
        let test = '<span style=' + color + '><b>'
        el.innerHTML = test + (text ) + '</b></span>'
        let tempMarker = new maplibregl
            .Marker(el)
            .setLngLat(pos)
            .addTo(map)
        // console.log(tempMarker)

        return tempMarker;
    }


    function add_popup_on_marker(pos, htmlString, fontSizePX)
    {
        const popup = new maplibregl.Popup()
            .setLngLat(pos)
            .setHTML(htmlString)
            .addTo(map);

        const popupElem = popup.getElement();
        popupElem.style.fontSize = fontSizePX + "px";
    }

}
