/**
 * ADS-B Aircraft WebSocket Client
 * 
 * This module handles real-time streaming of ADS-B aircraft data through a WebSocket connection.
 * It maintains a persistent connection to the Go WebSocket server and updates aircraft
 * positions and information on the map and side panel.
 * 
 * Features:
 * - Real-time aircraft tracking (updates within last 60 seconds)
 * - Map markers with popups showing aircraft information
 * - Side panel integration with aircraft list
 * - Automatic connection management with reconnection logic
 * 
 * Dependencies:
 * - maplibregl (for map markers and popups)
 * - Global variables: OPERATION_ID, TOKEN, NGINX_PORT
 */

(function() {
    'use strict';

    const DEBUG_ADSB = false;

    // Configuration
    const CONFIG = {
        wsInterval: 1000, // WebSocket polling interval in milliseconds
        reconnectDelay: 2000, // Delay before attempting to reconnect in milliseconds
        wsEndpoint: '/ws/getAdsbAircraft',
    };

    // State management
    let allAdsbAircraft = {};
    let allAdsbAircraftMarkers = {};
    let allAdsbAircraftTrails = {}; // Store trail coordinates for each aircraft
    let adsbWebSocket = null;
    let adsbWsInterval = null;

    const AIRCRAFT_ICON = '/static/aiders/imgs/plane-icon.png';

    const TRAIL_DURATION_MINUTES = 30; // Show last 30 minutes of trail

    /**
     * Initialize the ADS-B aircraft WebSocket connection
     */
    function initAdsbAircraftWebSocket() {
        const wsAddress = `ws://${window.location.hostname}:${NGINX_PORT}${CONFIG.wsEndpoint}`;
        console.log('Connecting to ADS-B Aircraft WebSocket:', wsAddress);

        try {
            adsbWebSocket = new WebSocket(`${wsAddress}?token=${encodeURIComponent(TOKEN)}`);

            adsbWebSocket.addEventListener('open', handleWebSocketOpen);
            adsbWebSocket.addEventListener('close', handleWebSocketClose);
            adsbWebSocket.addEventListener('error', handleWebSocketError);
            adsbWebSocket.addEventListener('message', handleWebSocketMessage);

        } catch (error) {
            console.error('Error creating ADS-B Aircraft WebSocket:', error);
            scheduleReconnect();
        }
    }

    /**
     * Handle WebSocket connection opened
     */
    function handleWebSocketOpen(event) {
        console.log('ADS-B Aircraft WebSocket connection established.');
        
        // Start polling for aircraft data
        if (adsbWsInterval) {
            clearInterval(adsbWsInterval);
        }
        adsbWsInterval = setInterval(sendAdsbAircraftRequest, CONFIG.wsInterval);
    }

    /**
     * Handle WebSocket connection closed
     */
    function handleWebSocketClose(event) {
        console.log('ADS-B Aircraft WebSocket connection closed. Reconnecting...');
        
        if (adsbWsInterval) {
            clearInterval(adsbWsInterval);
            adsbWsInterval = null;
        }
        
        scheduleReconnect();
    }

    /**
     * Handle WebSocket error
     */
    function handleWebSocketError(error) {
        console.error('ADS-B Aircraft WebSocket error:', error);
    }

    /**
     * Handle incoming WebSocket message
     */
    function handleWebSocketMessage(event) {
        try {
            const data = JSON.parse(event.data);
            
            if (data.error_msg && data.error_msg.length > 0) {
                console.warn('ADS-B Aircraft WebSocket error:', data.error_msg);
            }
            
            if (data.adsb_aircraft) {
                handleAdsbAircraftUpdate(data.adsb_aircraft);
            }
        } catch (error) {
            console.error('Error parsing ADS-B Aircraft WebSocket message:', error);
        }
    }

    /**
     * Send request for ADS-B aircraft data
     */
    function sendAdsbAircraftRequest() {
        if (adsbWebSocket && adsbWebSocket.readyState === WebSocket.OPEN) {
            const request = {
                operation_id: parseInt(OPERATION_ID)
            };
            adsbWebSocket.send(JSON.stringify(request));
        }
    }

    /**
     * Schedule a reconnection attempt
     */
    function scheduleReconnect() {
        setTimeout(() => {
            console.log('Attempting to reconnect ADS-B Aircraft WebSocket...');
            initAdsbAircraftWebSocket();
        }, CONFIG.reconnectDelay);
    }

    /**
     * Update ADS-B aircraft on map and panel based on received data
     */
    function handleAdsbAircraftUpdate(aircraft) {
        if (DEBUG_ADSB) {
            console.log('ADS-B Aircraft update:', aircraft);
        }
        
        // Hide section if no aircraft
        if (Object.keys(allAdsbAircraft).length === 0 && aircraft.length === 0) {
            const menuElement = document.getElementById("adsb-aircraft-menu");
            if (menuElement) {
                menuElement.style.display = "none";
            }
            return;
        }

        // Process incoming aircraft
        if (aircraft && Array.isArray(aircraft)) {
            aircraft.forEach(ac => {
                const acKey = ac.icao24;
                
                // Add new aircraft or update existing
                if (!allAdsbAircraft[acKey]) {
                    allAdsbAircraft[acKey] = ac;
                    placeAdsbAircraftOnMap(ac);
                } else {
                    // Update existing aircraft
                    updateAdsbAircraftOnMap(ac);
                    allAdsbAircraft[acKey] = ac;
                }
            });
        }

        // Remove aircraft that are no longer in the update
        Object.keys(allAdsbAircraft).forEach(icao24 => {
            if (!aircraft.some(ac => ac.icao24 === icao24)) {
                console.log(`Aircraft ${icao24} not found in update, removing from map and panel.`);
                removeAdsbAircraft(icao24);
            }
        });

        // Update UI
        updateAdsbAircraftUI();
    }

    /**
     * Place ADS-B aircraft marker on the map
     */
    function placeAdsbAircraftOnMap(ac) {
        if (!ac.lat || !ac.lon) {
            console.warn(`Aircraft ${ac.icao24} has no coordinates, skipping map placement.`);
            return;
        }

        // Convert altitude from feet to meters for display
        const altitudeM = ac.altitude_ft ? (ac.altitude_ft * 0.3048).toFixed(0) : 'N/A';
        // Convert speed from knots to km/h
        const speedKmh = ac.ground_speed_kts ? (ac.ground_speed_kts * 1.852).toFixed(0) : 'N/A';
        
        // Create popup content
        const popupContent = `
            <div style="font-family: Arial, sans-serif; min-width: 220px;">
                <h4 style="margin: 0 0 10px 0; color: #333;">ADS-B Aircraft</h4>
                <table style="width: 100%; font-size: 12px;">
                    <tr><td><b>ICAO24:</b></td><td>${ac.icao24}</td></tr>
                    ${ac.callsign ? `<tr><td><b>Callsign:</b></td><td>${ac.callsign}</td></tr>` : ''}
                    ${ac.altitude_ft !== null ? `<tr><td><b>Altitude:</b></td><td>${ac.altitude_ft} ft (${altitudeM} m)</td></tr>` : ''}
                    ${ac.ground_speed_kts !== null ? `<tr><td><b>Speed:</b></td><td>${ac.ground_speed_kts} kts (${speedKmh} km/h)</td></tr>` : ''}
                    ${ac.track_deg !== null ? `<tr><td><b>Heading:</b></td><td>${ac.track_deg.toFixed(0)}°</td></tr>` : ''}
                    ${ac.vertical_rate_fpm !== null ? `<tr><td><b>V/S:</b></td><td>${ac.vertical_rate_fpm} ft/min</td></tr>` : ''}
                    ${ac.squawk ? `<tr><td><b>Squawk:</b></td><td>${ac.squawk}</td></tr>` : ''}
                    ${ac.emergency ? `<tr><td><b>Status:</b></td><td style="color: red;"><b>EMERGENCY</b></td></tr>` : ''}
                    ${ac.on_ground ? `<tr><td><b>Status:</b></td><td>On Ground</td></tr>` : ''}
                    <tr><td><b>Position:</b></td><td>${ac.lat.toFixed(6)}, ${ac.lon.toFixed(6)}</td></tr>
                    <tr><td><b>Last Seen:</b></td><td>${new Date(ac.last_seen).toLocaleTimeString()}</td></tr>
                </table>
            </div>
        `;

        // Create popup
        const popup = new maplibregl.Popup({ offset: 25 })
            .setHTML(popupContent);

        // Create marker element
        const el = document.createElement('div');
        el.className = 'adsb-aircraft-marker';
        el.style.width = '32px';
        el.style.minHeight = '32px';
        el.style.display = 'flex';
        el.style.flexDirection = 'column';
        el.style.alignItems = 'center';
        
        // Create callsign label
        if (ac.callsign) {
            const callsignLabel = document.createElement('div');
            callsignLabel.style.fontSize = '10px';
            callsignLabel.style.fontWeight = 'bold';
            callsignLabel.style.color = '#000';
            callsignLabel.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
            callsignLabel.style.padding = '1px 4px';
            callsignLabel.style.borderRadius = '3px';
            callsignLabel.style.marginBottom = '2px';
            callsignLabel.style.whiteSpace = 'nowrap';
            callsignLabel.textContent = ac.callsign.trim();
            el.appendChild(callsignLabel);
        }
        
        // Create inner element for the icon
        const innerEl = document.createElement('div');
        innerEl.className = 'aircraft-icon';
        innerEl.style.cursor = 'pointer';
        innerEl.style.width = '32px';
        innerEl.style.height = '32px';
        innerEl.style.transition = 'transform 0.5s ease-out';
        
        // Rotate aircraft icon based on heading (track_deg is 0-359, 0=North, clockwise)
        if (ac.track_deg !== null) {
            // Airplane icon points up by default, so rotation matches track_deg directly
            innerEl.style.transform = `rotate(${ac.track_deg}deg)`;
        }
        
        // Use airplane image icon
        const iconImg = document.createElement('img');
        iconImg.src = AIRCRAFT_ICON;
        iconImg.style.width = '100%';
        iconImg.style.height = '100%';
        iconImg.style.objectFit = 'contain';
        innerEl.appendChild(iconImg);
        
        innerEl.setAttribute('data-rotation', ac.track_deg || 0);
        
        // Color code based on altitude (blue for high, green for medium, yellow for low)
        if (ac.altitude_ft !== null) {
            if (ac.altitude_ft > 30000) {
                innerEl.style.color = '#0066FF'; // Blue - high altitude
            } else if (ac.altitude_ft > 10000) {
                innerEl.style.color = '#00CC66'; // Green - medium altitude
            } else {
                innerEl.style.color = '#FFB800'; // Yellow - low altitude
            }
        }
        
        // Emergency aircraft in red
        if (ac.emergency) {
            innerEl.style.color = '#FF0000';
            innerEl.style.animation = 'blink 1s linear infinite';
        }
        
        el.appendChild(innerEl);

        // Create marker
        const marker = new maplibregl.Marker({
            element: el, 
            anchor: 'center',
            offset: [0, ac.callsign ? -12 : 0], // Offset down by half the callsign label height
            rotationAlignment: 'map'
        })
            .setLngLat([ac.lon, ac.lat])
            .setPopup(popup)
            .addTo(map);

        // Store marker
        allAdsbAircraftMarkers[ac.icao24] = marker;
        
        // Fetch initial trail data from server (only once when aircraft first appears)
        fetchAircraftTrail(ac.icao24);

        console.log(`Aircraft ${ac.icao24} placed on map.`);
    }

    /**
     * Update existing ADS-B aircraft marker on the map
     */
    function updateAdsbAircraftOnMap(ac) {
        const marker = allAdsbAircraftMarkers[ac.icao24];
        
        if (marker && ac.lat && ac.lon) {
            // Update position
            marker.setLngLat([ac.lon, ac.lat]);
            
            // Update rotation based on heading with smooth transition
            const el = marker.getElement();
            const innerEl = el.querySelector('.aircraft-icon');
            if (innerEl && ac.track_deg !== null) {
                const currentRotation = parseFloat(innerEl.getAttribute('data-rotation') || 0);
                let newRotation = ac.track_deg;
                
                // Handle rotation wrapping (359° -> 1° should rotate 2° not 358°)
                let diff = newRotation - currentRotation;
                if (diff > 180) diff -= 360;
                if (diff < -180) diff += 360;
                
                const finalRotation = currentRotation + diff;
                innerEl.style.transform = `rotate(${finalRotation}deg)`;
                innerEl.setAttribute('data-rotation', newRotation);
            }
            
            // Update trail
            updateAircraftTrail(ac);
            
            // Convert altitude and speed
            const altitudeM = ac.altitude_ft ? (ac.altitude_ft * 0.3048).toFixed(0) : 'N/A';
            const speedKmh = ac.ground_speed_kts ? (ac.ground_speed_kts * 1.852).toFixed(0) : 'N/A';
            
            // Update popup content
            const popupContent = `
                <div style="font-family: Arial, sans-serif; min-width: 220px;">
                    <h4 style="margin: 0 0 10px 0; color: #333;">ADS-B Aircraft</h4>
                    <table style="width: 100%; font-size: 12px;">
                        <tr><td><b>ICAO24:</b></td><td>${ac.icao24}</td></tr>
                        ${ac.callsign ? `<tr><td><b>Callsign:</b></td><td>${ac.callsign}</td></tr>` : ''}
                        ${ac.altitude_ft !== null ? `<tr><td><b>Altitude:</b></td><td>${ac.altitude_ft} ft (${altitudeM} m)</td></tr>` : ''}
                        ${ac.ground_speed_kts !== null ? `<tr><td><b>Speed:</b></td><td>${ac.ground_speed_kts} kts (${speedKmh} km/h)</td></tr>` : ''}
                        ${ac.track_deg !== null ? `<tr><td><b>Heading:</b></td><td>${ac.track_deg.toFixed(0)}°</td></tr>` : ''}
                        ${ac.vertical_rate_fpm !== null ? `<tr><td><b>V/S:</b></td><td>${ac.vertical_rate_fpm} ft/min</td></tr>` : ''}
                        ${ac.squawk ? `<tr><td><b>Squawk:</b></td><td>${ac.squawk}</td></tr>` : ''}
                        ${ac.emergency ? `<tr><td><b>Status:</b></td><td style="color: red;"><b>EMERGENCY</b></td></tr>` : ''}
                        ${ac.on_ground ? `<tr><td><b>Status:</b></td><td>On Ground</td></tr>` : ''}
                        <tr><td><b>Position:</b></td><td>${ac.lat.toFixed(6)}, ${ac.lon.toFixed(6)}</td></tr>
                        <tr><td><b>Last Seen:</b></td><td>${new Date(ac.last_seen).toLocaleTimeString()}</td></tr>
                    </table>
                </div>
            `;
            
            marker.getPopup().setHTML(popupContent);
        }
    }

    /**
     * Fetch trail data for a specific aircraft (called once when aircraft first appears)
     */
    function fetchAircraftTrail(icao24) {
        const now = new Date();
        const since = new Date(now.getTime() - TRAIL_DURATION_MINUTES * 60 * 1000);
        
        // Use CURRENT_OP (operation name) to construct the correct URL path
        const url = `/api/operations/${CURRENT_OP}/adsb/aircraft/${icao24}/trail?since=${since.toISOString()}`;
        
        fetch(url, {
            headers: {
                'Authorization': `Bearer ${TOKEN}`,
                'Content-Type': 'application/json'
            }
        })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            return response.json();
        })
        .then(data => {
            if (data.trail && data.trail.length > 0) {
                updateAircraftTrailLayer(icao24, data.trail);
            } else if (DEBUG_ADSB) {
                console.log(`No trail data available for aircraft ${icao24}`);
            }
        })
        .catch(error => {
            if (DEBUG_ADSB) {
                console.warn(`Error fetching trail for aircraft ${icao24}:`, error);
            }
        });
    }

    /**
     * Update the trail layer on the map for a specific aircraft
     */
    function updateAircraftTrailLayer(icao24, trailData) {
        const sourceId = `adsb-trail-${icao24}`;
        const layerId = `adsb-trail-layer-${icao24}`;
        
        // Convert trail data to GeoJSON LineString
        const coordinates = trailData.map(point => [point.longitude, point.latitude]);
        
        if (coordinates.length < 2) {
            return; // Need at least 2 points for a line
        }
        
        const geojson = {
            type: 'Feature',
            geometry: {
                type: 'LineString',
                coordinates: coordinates
            },
            properties: {
                icao24: icao24
            }
        };
        
        // Check if source exists
        if (map.getSource(sourceId)) {
            // Update existing source
            map.getSource(sourceId).setData(geojson);
        } else {
            // Add new source and layer
            map.addSource(sourceId, {
                type: 'geojson',
                data: geojson
            });
            
            map.addLayer({
                id: layerId,
                type: 'line',
                source: sourceId,
                layout: {
                    'line-join': 'round',
                    'line-cap': 'round'
                },
                paint: {
                    'line-color': '#0080FF',
                    'line-width': 2,
                    'line-opacity': 0.7
                }
            });
        }
        
        // Store trail data
        allAdsbAircraftTrails[icao24] = trailData;
    }

    /**
     * Update aircraft trail in real-time (add current position)
     */
    function updateAircraftTrail(ac) {
        if (!ac.lat || !ac.lon) return;
        
        const icao24 = ac.icao24;
        const sourceId = `adsb-trail-${icao24}`;
        
        // Add current position to trail
        if (!allAdsbAircraftTrails[icao24]) {
            allAdsbAircraftTrails[icao24] = [];
        }
        
        // Add new position
        allAdsbAircraftTrails[icao24].push({
            latitude: ac.lat,
            longitude: ac.lon,
            timestamp: new Date().toISOString()
        });
        
        // Keep only recent positions (limit trail length)
        const cutoffTime = new Date(Date.now() - TRAIL_DURATION_MINUTES * 60 * 1000);
        allAdsbAircraftTrails[icao24] = allAdsbAircraftTrails[icao24].filter(point => 
            new Date(point.timestamp) > cutoffTime
        );
        
        // Update trail layer if we have enough points
        if (allAdsbAircraftTrails[icao24].length >= 2) {
            updateAircraftTrailLayer(icao24, allAdsbAircraftTrails[icao24]);
        }
    }

    /**
     * Remove ADS-B aircraft from map and panel
     */
    function removeAdsbAircraft(icao24) {
        // Remove from map
        const marker = allAdsbAircraftMarkers[icao24];
        if (marker) {
            marker.remove();
            delete allAdsbAircraftMarkers[icao24];
        }
        
        // Remove trail layer
        const sourceId = `adsb-trail-${icao24}`;
        const layerId = `adsb-trail-layer-${icao24}`;
        if (map.getLayer(layerId)) {
            map.removeLayer(layerId);
        }
        if (map.getSource(sourceId)) {
            map.removeSource(sourceId);
        }
        delete allAdsbAircraftTrails[icao24];

        // Remove from panel
        const panelEntry = document.getElementById(`adsb-aircraft-${icao24}`);
        if (panelEntry) {
            panelEntry.remove();
        }

        // Remove from state
        delete allAdsbAircraft[icao24];
    }

    /**
     * Update ADS-B aircraft UI (counter and visibility)
     */
    function updateAdsbAircraftUI() {
        const menuElement = document.getElementById("adsb-aircraft-menu");
        const counterElement = document.getElementById("connected-adsb-aircraft");
        
        const aircraftCount = Object.keys(allAdsbAircraft).length;
        
        if (aircraftCount > 0) {
            if (counterElement) {
                counterElement.innerHTML = aircraftCount;
            }
            if (menuElement) {
                menuElement.style.display = "block";
            }
        } else {
            if (menuElement) {
                menuElement.style.display = "none";
            }
        }
    }

    /**
     * Zoom to a specific aircraft on the map
     */
    function zoomToAdsbAircraft(icao24) {
        const ac = allAdsbAircraft[icao24];
        const marker = allAdsbAircraftMarkers[icao24];
        
        if (ac && marker && ac.lat && ac.lon) {
            map.flyTo({
                center: [ac.lon, ac.lat],
                zoom: 12,
                duration: 1500
            });
            
            // Open the popup
            marker.togglePopup();
        }
    }

    /**
     * Cleanup function for disconnection
     */
    function cleanup() {
        if (adsbWsInterval) {
            clearInterval(adsbWsInterval);
            adsbWsInterval = null;
        }
        
        if (adsbWebSocket) {
            adsbWebSocket.close();
            adsbWebSocket = null;
        }

        // Remove all aircraft
        Object.keys(allAdsbAircraft).forEach(icao24 => {
            removeAdsbAircraft(icao24);
        });
    }

    // Make zoomToAdsbAircraft globally accessible
    window.zoomToAdsbAircraft = zoomToAdsbAircraft;

    // Initialize on map load
    if (typeof map !== 'undefined') {
        map.on('load', function() {
            console.log('Initializing ADS-B Aircraft WebSocket...');
            initAdsbAircraftWebSocket();
        });
    } else {
        console.warn('Map object not found, delaying ADS-B Aircraft WebSocket initialization');
        setTimeout(() => {
            if (typeof map !== 'undefined') {
                map.on('load', function() {
                    console.log('Initializing ADS-B Aircraft WebSocket...');
                    initAdsbAircraftWebSocket();
                });
            }
        }, 1000);
    }

    // Cleanup on page unload
    window.addEventListener('beforeunload', cleanup);

})();
