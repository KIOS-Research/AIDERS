// get the coverage points from the back-end and render them on the map
function showOperationCoverage(fromDate, fromTime, toDate, toTime, getDrones, getDevices, getBaloras, getPoints) {
    $("#coverageShowBtn").prop("disabled", true);
    $("#coverageLoader").show();

    clearOperationCoverage();

    console.log("getting operation coverage");

    let postData = {
        fromDate: fromDate,
        fromTime: fromTime,
        toDate: toDate,
        toTime: toTime,
        getDrones: getDrones,
        getDevices: getDevices,
        getBaloras: getBaloras,
        getPoints: getPoints,
    };
    postGetCoveragePoints(postData).then(function (_response) {
        console.log("Coverage response received:", _response);
        console.log("Drone polygons count:", _response.dronePolygons?.length || 0);
        console.log("Device polygons count:", _response.devicePolygons?.length || 0);
        console.log("Balora polygons count:", _response.baloraPolygons?.length || 0);

        renderPoints("drone", _response.droneAllTelemetryPoints, "#F00", 2.5, "lat", "lon");
        renderPoints("device", _response.deviceAllTelemetryPoints, "#D33", 2.5, "latitude", "longitude");
        renderPoints("balora", _response.baloraAllTelemetryPoints, "#D33", 2.5, "latitude", "longitude");

        renderPolygons("drone", _response.dronePolygons, _response.droneData, "#FF0000", 0.2);
        renderPolygons("device", _response.devicePolygons, _response.deviceData, "#FF00FF", 0.2);
        renderPolygons("balora", _response.baloraPolygons, _response.baloraData, "#f58d05", 0.2);

        $("#coverageShowBtn").prop("disabled", false);
        $("#coverageLoader").hide();

        create_popup_for_a_little(SUCCESS_ALERT, "Operation coverage plotted.", 3000);
    });
}

function renderPolygons(clientType, allPolygons, allData, color, opacity) {
    console.log(`Rendering ${allPolygons.length} ${clientType} coverage areas`);
    
    allPolygons.forEach(function (currentPolygon, i) {
        let polygon = JSON.parse(currentPolygon);
        console.log(`Coverage area ${i}:`, polygon);

        // The polygon from backend is already in [lon, lat] format which is correct for MapLibre
        // Just ensure it's properly closed (first point = last point)
        if (polygon.length > 0 && 
            (polygon[0][0] !== polygon[polygon.length - 1][0] || 
             polygon[0][1] !== polygon[polygon.length - 1][1])) {
            polygon.push(polygon[0]); // Close the polygon if not already closed
        }

        let rand = Math.random() * 100000;

        let shape = {
            type: "Feature",
            geometry: {
                type: "Polygon",
                coordinates: [polygon], // Use the polygon coordinates directly
            },
        };

        console.log(`Adding coverage area ${i} with ${polygon.length} points:`, shape);

        map.addSource(clientType + "-coverage-" + i + "-" + rand, {
            type: "geojson",
            data: shape,
        });

        map.addLayer({
            id: clientType + "-coverage-" + i + "-" + rand,
            type: "fill",
            source: clientType + "-coverage-" + i + "-" + rand,
            paint: {
                "fill-color": color,
                "fill-opacity": opacity,
            },
        });

        // add an outline layer
        map.addLayer({
            id: clientType + "-coverage-outline-" + i + "-" + rand,
            type: "line",
            source: clientType + "-coverage-" + i + "-" + rand,
            paint: {
                "line-color": "#555", // Set the outline color here
                "line-width": 2,
            },
        });

        // add a popup on click
        if (polygon.length > 0) {
            // calculate the centroid of the polygon
            let xSum = 0,
                ySum = 0;
            polygon.forEach(function (point) {
                xSum += parseFloat(point[0]); // longitude
                ySum += parseFloat(point[1]); // latitude
            });
            let centroid = [xSum / polygon.length, ySum / polygon.length];

            console.log(`Coverage area ${i} centroid:`, centroid);

            // Calculate approximate area (rough estimate for display purposes)
            let area = calculatePolygonArea(polygon);
            
            // create the popup
            var date = new Date(allData[i][1]);
            var formattedTime = date.toLocaleString();
            let popup = new maplibregl.Popup().setLngLat(centroid).setHTML(
                `<strong>${allData[i][0]}</strong><br />` +
                `Start Time: ${formattedTime}<br />` +
                `Coverage Points: ${polygon.length - 1}<br />` + // -1 because polygon is closed
                `Approx. Area: ${area.toFixed(2)} km²`
            );

            // add the 'click' event listener
            map.on("click", clientType + "-coverage-" + i + "-" + rand, function (e) {
                popup.addTo(map);
            });
        } else {
            console.log("polygon.length = 0");
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

    map.addSource(clientType + "-points-" + rand, {
        type: "geojson",
        data: {
            type: "Feature",
            geometry: {
                type: "MultiPoint",
                coordinates: points,
            },
        },
    });
    map.addLayer({
        id: clientType + "-points-" + rand,
        type: "circle",
        source: clientType + "-points-" + rand,
        paint: {
            "circle-radius": size,
            "circle-color": color,
        },
    });
}

function clearOperationCoverage() {
    console.log("clearing operation coverage");
    map.getStyle().layers.forEach(function (layer) {
        if (
            layer.id.indexOf("drone-coverage-") === 0 ||
            layer.id.indexOf("device-coverage-") === 0 ||
            layer.id.indexOf("balora-coverage-") === 0
        ) {
            map.removeLayer(layer.id);
        }
        if (
            layer.id.indexOf("drone-points-") === 0 ||
            layer.id.indexOf("device-points-") === 0 ||
            layer.id.indexOf("balora-points-") === 0
        ) {
            map.removeLayer(layer.id);
        }
    });
    
    // Clear all sources as well
    Object.keys(map.getStyle().sources).forEach(function(sourceId) {
        if (
            sourceId.indexOf("drone-coverage-") === 0 ||
            sourceId.indexOf("device-coverage-") === 0 ||
            sourceId.indexOf("balora-coverage-") === 0 ||
            sourceId.indexOf("drone-points-") === 0 ||
            sourceId.indexOf("device-points-") === 0 ||
            sourceId.indexOf("balora-points-") === 0
        ) {
            try {
                map.removeSource(sourceId);
            } catch (e) {
                console.log("Error removing source:", sourceId, e);
            }
        }
    });
}

// Utility function to calculate approximate area of a polygon in km²
function calculatePolygonArea(coordinates) {
    if (coordinates.length < 3) return 0;
    
    let area = 0;
    const earthRadius = 6371000; // Earth's radius in meters
    
    for (let i = 0; i < coordinates.length - 1; i++) {
        const lon1 = coordinates[i][0] * Math.PI / 180;
        const lat1 = coordinates[i][1] * Math.PI / 180;
        const lon2 = coordinates[(i + 1) % (coordinates.length - 1)][0] * Math.PI / 180;
        const lat2 = coordinates[(i + 1) % (coordinates.length - 1)][1] * Math.PI / 180;
        
        area += (lon2 - lon1) * (2 + Math.sin(lat1) + Math.sin(lat2));
    }
    
    area = Math.abs(area) * earthRadius * earthRadius / 2;
    return area / 1000000; // Convert to km²
}
