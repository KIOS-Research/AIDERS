

// get the coverage points from the back-end and render them on the map
function showOperationCoverage(fromDate, fromTime, toDate, toTime, getDrones, getDevices, getBaloras, getPoints) {
    $("#coverageShowBtn").prop("disabled", true);
    $("#coverageLoader").show();
    
    clearOperationCoverage();
    
    console.log("getting operation coverage");

    $.ajax({
        url: "/coverage_points/" + CURRENT_OP,
        type: 'POST',
        headers: {
            'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
        },        
        data: JSON.stringify({ fromDate: fromDate, fromTime: fromTime, toDate: toDate, toTime: toTime, getDrones: getDrones, getDevices: getDevices, getBaloras: getBaloras, getPoints: getPoints }),
        contentType: 'application/json; charset=utf-8',
        dataType: 'json',
        async: true,
        success: function (response) {
            
            // console.log(response.dronePolygons);
            // console.log(response.droneData);

            renderPoints('drone', response.droneAllTelemetryPoints, "#F00", 2.5, 'lat', 'lon');
            renderPoints('device', response.deviceAllTelemetryPoints, "#D33", 2.5, 'latitude', 'longitude');
            renderPoints('balora', response.baloraAllTelemetryPoints, "#D33", 2.5, 'latitude', 'longitude');

            renderPolygons('drone', response.dronePolygons, response.droneData, "#FF0000", 0.2);
            renderPolygons('device', response.devicePolygons, response.deviceData, "#FF00FF", 0.2);
            renderPolygons('balora', response.baloraPolygons, response.baloraData, "#f58d05", 0.2);

            $("#coverageShowBtn").prop("disabled", false);
            $("#coverageLoader").hide();

            create_popup_for_a_little(SUCCESS_ALERT, "Operation coverage plotted.", 3000);
        }
    });

}


function renderPolygons(clientType, allPolygons, allData, color, opacity) {

    allPolygons.forEach(function (currentPolygon, i) {
        
        let polygonPoints = [];
        let firstPoint = [0, 0]
        let polygon = JSON.parse(currentPolygon);
        
        if (polygon.length < 3) {
            polygon = polygon[0];
        }
        // if (polygon.length < 3) {
        //     polygon = polygon[0];
        // }
        // console.log("polygon: " + polygon.length);
        polygon.forEach(function (point, j) {
            // console.log(point);

            if (firstPoint[0] == 0 && firstPoint[1] == 0) {
                firstPoint = [point[1], point[0]];
            }
            polygonPoints.push([point[1], point[0]]);
        });
        polygonPoints.push(firstPoint);   // add the first point to the end of the path to close the polygon

        let rand = Math.random(100000);

        let shape = {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [polygonPoints]
            }
        };

        map.addSource(clientType + "-coverage-" + i + "-" + rand, {
            "type": "geojson",
            "data": shape
        });

        map.addLayer({
            "id": clientType + "-coverage-" + i + "-" + rand,
            "type": "fill",
            "source": clientType + "-coverage-" + i + "-" + rand,
            "paint": {
                "fill-color": color,
                "fill-opacity": opacity
            }
        });

        // add an outline layer
        map.addLayer({
            "id": clientType + "-coverage-outline-" + i + "-" + rand,
            "type": "line",
            "source": clientType + "-coverage-" + i + "-" + rand,
            "paint": {
                "line-color": "#555", // Set the outline color here
                "line-width": 2
            }
        });

        // add a popup on click
        if (polygonPoints.length > 0) {
            // calculate the centroid of the polygon
            let xSum = 0, ySum = 0;
            polygonPoints.forEach(function (point) {
                xSum += parseFloat(point[0]);
                ySum += parseFloat(point[1]);
            });
            let centroid = [xSum / polygonPoints.length, ySum / polygonPoints.length];
    
            // console.log("xSum: " + xSum);
            // console.log("ySum: " + ySum);
            // console.log("centroid: " + centroid);
    
            // create the popup
            var date = new Date(allData[i][1]);
            var formattedTime = date.toLocaleString();
            let popup = new maplibregl.Popup()
                .setLngLat(centroid)
                .setHTML(allData[i][0] + "<br />" + formattedTime)
    
            // add the 'click' event listener
            map.on('click', clientType + "-coverage-" + i + "-" + rand, function (e) {
                popup.addTo(map);
            });            
        }
        else {
            console.log("polygonPoints.length = 0");
        }

    });

}


function renderPoints(clientType, allPoints, color, size, latName, lonName) {

    let points = [];
    var tempJson = JSON.parse(allPoints);
    tempJson.forEach(function (point, i) {
        points.push([point[lonName], point[latName]]);
    });

    let rand = Math.random(100000);

    map.addSource(clientType + '-points-' + rand, {
        "type": "geojson",
        "data": {
            "type": "Feature",
            "geometry": {
                "type": "MultiPoint",
                "coordinates": points
            }
        }
    });
    map.addLayer({
        "id": clientType + '-points-' + rand,
        "type": "circle",
        "source": clientType + '-points-' + rand,
        "paint": {
            "circle-radius": size,
            "circle-color": color
        }
    });
}    


function clearOperationCoverage() {
    console.log("clearing operation coverage");
    map.getStyle().layers.forEach(function (layer) {
        if (layer.id.indexOf('drone-coverage-') === 0 || layer.id.indexOf('device-coverage-') === 0 || layer.id.indexOf('balora-coverage-') === 0) {
            map.removeLayer(layer.id);
        }
        if (layer.id.indexOf('drone-points-') === 0 || layer.id.indexOf('device-points-') === 0 || layer.id.indexOf('balora-points-') === 0) {
            map.removeLayer(layer.id);
        }        
    });        
}

