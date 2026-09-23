// Initialize waypoint-related state
let waypointCounter = 1;
const waypoints = []; // Stores waypoint data (coordinates + name)
const waypointMarkers = []; // Stores marker objects added on the map
let mapClickHandler = null; // Reference to the current map click event handler

// Triggered when user chooses to define a new area
function defineArea() {    
    showDefineAreaSelectionMessage();
    placeWaypointsOnMap();    
}

// Clears all placed waypoints and resets state
function clearDefineAreaWaypoints() {
    $("#define-area-selection-message").hide();
    $("#sidebar-overlay").hide();

    // Remove all markers from the map
    waypointMarkers.forEach(marker => marker.remove());
    waypointMarkers.length = 0;
    waypoints.length = 0;
    waypointCounter = 1;

    // Remove map click handler if it exists
    if (mapClickHandler) {
        map.off('click', mapClickHandler);
        mapClickHandler = null;
    }

    console.log("All waypoints cleared and click listener removed.");
}

// Show UI message for defining area
function showDefineAreaSelectionMessage() {
    $("#define-area-selection-message").show();
    $("#sidebar-overlay").show();
}

// Handles adding waypoints on map click
function placeWaypointsOnMap() {
    // Remove previous listener if exists
    if (mapClickHandler) map.off('click', mapClickHandler);

    // Define and store the new click handler
    mapClickHandler = function (e) {
        const lngLat = e.lngLat;

        // Create waypoint object and push to array
        const waypoint = {
            name: `Waypoint ${waypointCounter}`,
            coords: [lngLat.lng, lngLat.lat]
        };
        waypoints.push(waypoint);
        waypointCounter++;

        // Add marker to map
        const marker = new maplibregl.Marker()
            .setLngLat(waypoint.coords)
            .setPopup(new maplibregl.Popup().setText(waypoint.name))
            .addTo(map);

        waypointMarkers.push(marker);

        // console.log(`Waypoint ${waypointCounter - 1}:`);
        // console.log(`Latitude: ${lngLat.lat}, Longitude: ${lngLat.lng}`);
        // console.log('All waypoints:', waypoints);
    };

    // Attach new click handler to map
    map.on('click', mapClickHandler);    
}

// Opens dialog for saving a user-defined area
function openUserDefinedAreaDialog() {
    if (waypointMarkers.length === 0) {
        showPopupForALittle("#noWaypointFounded", "", 2000); // Show warning if no waypoints
        return; // Do not open dialog
    }

    const defer = $.Deferred(); // Declare defer
    var userDefinedAreaDialog = $('#user-defined-area-dialog');

    userDefinedAreaDialog.dialog({
        autoOpen: false, // Dialog won't open automatically
        modal: true,
        height: 'auto',
        width: 600,
        title: "User Defined Area", // Set the title dynamically
        buttons: {
            "Save": function () {
                // Collect form input values
                const userId = $("#userId").val();
                const operationId = $("#operationId").val();
                const name = $("#name").val().trim();
                const description = $("#description").val().trim();
                const color = $("#color").val();
                
                // Validate input
                if (!name || !description) {
                    $(".errorMessage").text("Please fill in all required fields.");
                    return;
                }

                // Format waypoint data as coordinates
                const coordinates = waypoints.map(wp => ({
                    latitude: wp.coords[1],
                    longitude: wp.coords[0]
                }));

                // Construct data to be saved
                const areaData = {
                    name,
                    description,
                    color,
                    altitude: 0,
                    coordinates
                };

                const payload = {
                    algorithm_name: "USER_DEFINED_AREA",
                    title: name,
                    input: null,
                    output: areaData,
                    canBeLoadedOnMap: true,
                    operation_id: parseInt(operationId),
                    user_id: parseInt(userId)
                };

                // Get CSRF token and send POST request
                const csrfToken = document.getElementById("csrf").querySelector("input").value;

                fetch(`/save-user-defined-area/`, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "X-CSRFToken": csrfToken
                    },
                    body: JSON.stringify(payload)
                })
                .then(response => {
                    if (!response.ok) throw new Error("Error saving to database");
                    return response.json();
                })
                .then(data => {
                    console.log("Saved successfully", data);
                    $(this).dialog("close");
                    clearDefineAreaWaypoints();
                    showPopupForALittle('#successBox', "The defined area has been successfully saved.", 3000);

                    // display the newly saved area on the map
                    // const randomPk = Math.floor(Math.random() * 10000) + 1; // Generate a random PK for the new area
                    const newAreaData = {
                        fields: {
                            algorithm_name: "USER_DEFINED_AREA",
                            output: {
                                name: name,
                                description: description,
                                color: color,
                                coordinates: areaData.coordinates
                            }
                        },
                        pk: data.pk // Assuming the response contains the primary key of the saved area
                    };
                    loadUserDefinedAreas(newAreaData);

                })
                .catch(err => {
                    console.error("Save failed", err);
                });
            },
            "Cancel": function () {
                defer.resolve([false]);
                $(this).dialog("close");
            }
        },
        close: function () {
            defer.resolve([false]);
        }
    });
    userDefinedAreaDialog.dialog("open");
    return defer.promise();
}

function loadUserDefinedAreas(algorithm) {
    console.log(algorithm);
    
    // Exit if there is no output data
    if (!algorithm.fields.output) return;

    const output = algorithm.fields.output;
    const coords = output.coordinates.map(coord => [coord.longitude, coord.latitude]);

    // Ensure the polygon is closed (first and last coordinates must match)
    if (
        coords.length > 2 &&
        (coords[0][0] !== coords[coords.length - 1][0] ||
         coords[0][1] !== coords[coords.length - 1][1])
    ) {
        coords.push(coords[0]);
    }

    // Create a GeoJSON Feature for the polygon
    const geojson = {
        type: "Feature",
        geometry: {
            type: "Polygon",
            coordinates: [coords]
        },
        properties: {
            name: output.name,
            description: output.description,
            color: output.color
        }
    };

    const polygonId = `user-defined-area-${algorithm.pk}`;

    // Remove previous layer and source with the same ID (if they exist)
    if (map.getLayer(polygonId)) map.removeLayer(polygonId);
    if (map.getLayer(`${polygonId}-outline`)) map.removeLayer(`${polygonId}-outline`);
    if (map.getSource(polygonId)) map.removeSource(polygonId);

     // Add the GeoJSON source to the map
    map.addSource(polygonId, {
        type: "geojson",
        data: geojson
    });

    // Add a filled polygon layer to the map
    map.addLayer({
        id: polygonId,
        type: "fill",
        source: polygonId,
        paint: {
            "fill-color": output.color || "#088",
            "fill-opacity": 0.4
        }
    });

    // Add an outline layer for the polygon
    map.addLayer({
        id: `${polygonId}-outline`,
        type: "line",
        source: polygonId,
        paint: {
            "line-color": output.color || "#000",
            "line-width": 2
        }
    });

    // Show popup on polygon click
    map.on("click", polygonId, (e) => {
        new maplibregl.Popup()
            .setLngLat(e.lngLat)
            .setHTML(`<strong>${output.name}</strong><br>${output.description}`)
            .addTo(map);
    });

    // Change cursor to pointer on hover
    map.on("mouseenter", polygonId, () => {
        map.getCanvas().style.cursor = 'pointer';
    });
    // Reset cursor when not hovering
    map.on("mouseleave", polygonId, () => {
        map.getCanvas().style.cursor = '';
    });

    // Return coordinates for use in fitBounds
    return coords;
}

function clearUserDefinedArea() {
    const selectedAlgorithms = JSON.parse(sessionStorage.getItem(SessionProfileKeys.SELECTED_ALGORITHMS)) || [];

    selectedAlgorithms.forEach(algorithm => {
        if (algorithm.fields.algorithm_name === 'USER_DEFINED_AREA') {
            const polygonId = `user-defined-area-${algorithm.pk}`;
            
            // Αφαίρεση του fill layer
            if (map.getLayer(polygonId)) {
                map.removeLayer(polygonId);
            }

            // Αφαίρεση του outline layer
            if (map.getLayer(`${polygonId}-outline`)) {
                map.removeLayer(`${polygonId}-outline`);
            }

            // Αφαίρεση της πηγής (source)
            if (map.getSource(polygonId)) {
                map.removeSource(polygonId);
            }
        }
    });

    console.log("User-defined areas cleared from map.");
}

