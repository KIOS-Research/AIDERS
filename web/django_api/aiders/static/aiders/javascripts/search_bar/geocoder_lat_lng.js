/* given a query in the form "lng, lat" or "lat, lng" returns the matching
 * geographic coordinate(s) as search results in carmen geojson format,
 * https://github.com/mapbox/carmen/blob/master/carmen-geojson.md
 */
// var coordinatesGeocoder = function (query) {
// // match anything which looks like a decimal degrees coordinate pair
//     console.log("OVER_OVER")
//     var matches = query.match(
//         /^[ ]*(?:Lat: )?(-?\d+\.?\d*)[, ]+(?:Lng: )?(-?\d+\.?\d*)[ ]*$/i
//     );
//     if (!matches) {
//         return null;
//     }
//
//     function coordinateFeature(lng, lat) {
//         return {
//             center: [lng, lat],
//             geometry: {
//                 type: 'Point',
//                 coordinates: [lng, lat]
//             },
//             place_name: 'Lat: ' + lat + ' Lng: ' + lng,
//             place_type: ['coordinate'],
//             properties: {},
//             type: 'Feature'
//         };
//     }
//
//     var coord1 = Number(matches[1]);
//     var coord2 = Number(matches[2]);
//     var geocodes = [];
//
//     if (coord1 < -90 || coord1 > 90) {
// // must be lng, lat
//         geocodes.push(coordinateFeature(coord1, coord2));
//     }
//
//     if (coord2 < -90 || coord2 > 90) {
// // must be lat, lng
//         geocodes.push(coordinateFeature(coord2, coord1));
//     }
//
//     if (geocodes.length === 0) {
// // else could be either lng, lat or lat, lng
//         geocodes.push(coordinateFeature(coord1, coord2));
//         geocodes.push(coordinateFeature(coord2, coord1));
//     }
//
//     return geocodes;
// };

var coordinatesGeocoder = function (query) {
    // match anything which looks like a decimal degrees coordinate pair
    let isLatLng = query.match(/^[ ]*(?:Lat: )?(-?\d+\.?\d*)[, ]+(?:Lng: )?(-?\d+\.?\d*)[ ]*$/i);

    let drone = is_query_contains_drone_id(query);
    if (isLatLng) {
        var coord1 = Number(isLatLng[1]);
        var coord2 = Number(isLatLng[2]);
        var geocodes = [];

        if (coord1 < -90 || coord1 > 90) {
            // must be lng, lat
            geocodes.push(coordinateFeature(coord1, coord2));
        }

        if (coord2 < -90 || coord2 > 90) {
            // must be lat, lng
            geocodes.push(coordinateFeature(coord2, coord1));
        }

        if (geocodes.length === 0) {
            // else could be either lng, lat or lat, lng
            geocodes.push(coordinateFeature(coord1, coord2));
            geocodes.push(coordinateFeature(coord2, coord1));
        }

        return geocodes;
    } else if (drone !== -1) {
        console.log(drone);
        let lat = Number(drone.droneInfo.currentCoordinate[1]);
        let lng = Number(drone.droneInfo.currentCoordinate[0]);
        geocodes = [];

        if (lng < -90 || lng > 90) {
            // must be lat, lng
            geocodes.push(droneCoordinateFeature(lng, lat, drone.droneID));
        }

        if (geocodes.length === 0) {
            // else could be either lng, lat or lat, lng
            //             geocodes.push(coordinateFeature(lat, lng));
            geocodes.push(droneCoordinateFeature(lng, lat, drone.droneID));
        }

        return geocodes;
        // console.log("DRONE : ", drone)
        // console.log("LAT : ", drone.droneInfo.currentCoordinate[0])
        // console.log("LNG : ", drone.droneInfo.currentCoordinate[1])
        // geocodes.push(droneCoordinateFeature(drone.droneInfo.currentCoordinate[1],drone.droneInfo.currentCoordinate[0]))
    }

    return null;

    function coordinateFeature(lng, lat) {
        return {
            center: [lng, lat],
            geometry: {
                type: 'Point',
                coordinates: [lng, lat],
            },
            place_name: 'Lat: ' + lat + ' Lng: ' + lng,
            place_type: ['coordinate'],
            properties: {},
            type: 'Feature',
        };
    }

    function droneCoordinateFeature(lng, lat, droneID) {
        return {
            center: [lng, lat],
            geometry: {
                type: 'Point',
                coordinates: [lng, lat],
            },
            place_name: droneID + ' - Lat: ' + lat + ' Lng: ' + lng,
            place_type: ['coordinate'],
            properties: {},
            type: 'Feature',
        };
    }

    function loraCoordinateFeature(lng, lat, tagName) {
        return {
            center: [lng, lat],
            geometry: {
                type: 'Point',
                coordinates: [lng, lat],
            },
            place_name: tagName + '_Lora' + ' - Lat: ' + lat + ' Lng: ' + lng,
            place_type: ['coordinate'],
            properties: {},
            type: 'Feature',
        };
    }
};
