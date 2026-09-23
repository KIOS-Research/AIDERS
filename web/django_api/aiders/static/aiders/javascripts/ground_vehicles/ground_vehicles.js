/**
 * Ground Vehicles WebSocket Client
 * 
 * This module handles real-time streaming of ground vehicle data through a WebSocket connection.
 * It maintains a persistent connection to the Go WebSocket server and updates ground vehicle
 * positions and information on the map and side panel.
 * 
 * Features:
 * - Real-time ground vehicle tracking (updates within last 30 seconds)
 * - Map markers with popups showing vehicle information
 * - Side panel integration with vehicle list
 * - Automatic connection management with reconnection logic
 * 
 * Dependencies:
 * - maplibregl (for map markers and popups)
 * - Global variables: OPERATION_ID, TOKEN, NGINX_PORT
 */

(function() {
    'use strict';

    const DEBUG_VEHICLES = false;

    // Configuration
    const CONFIG = {
        wsInterval: 1000, // WebSocket polling interval in milliseconds
        reconnectDelay: 2000, // Delay before attempting to reconnect in milliseconds
        wsEndpoint: '/ws/getGroundVehicles',
    };

    // State management
    let allGroundVehicles = {};
    let allGroundVehicleMarkers = {};
    let groundVehicleWebSocket = null;
    let groundVehicleWsInterval = null;

    const ICONS = {
        CCD: '/static/aiders/imgs/ccd-car-logo.png',
        FIRE: '/static/aiders/imgs/firetruck-icon.png',
        POLICE: '/static/aiders/imgs/policecar-icon.png',
        AMBULANCE: '/static/aiders/imgs/ambulance-icon.png',
    };

    /**
     * Initialize the ground vehicles WebSocket connection
     */
    function initGroundVehiclesWebSocket() {
        const wsAddress = `ws://${window.location.hostname}:${NGINX_PORT}${CONFIG.wsEndpoint}`;
        console.log('Connecting to Ground Vehicles WebSocket:', wsAddress);

        try {
            groundVehicleWebSocket = new WebSocket(`${wsAddress}?token=${encodeURIComponent(TOKEN)}`);

            groundVehicleWebSocket.addEventListener('open', handleWebSocketOpen);
            groundVehicleWebSocket.addEventListener('close', handleWebSocketClose);
            groundVehicleWebSocket.addEventListener('error', handleWebSocketError);
            groundVehicleWebSocket.addEventListener('message', handleWebSocketMessage);

        } catch (error) {
            console.error('Error creating Ground Vehicles WebSocket:', error);
            scheduleReconnect();
        }
    }

    /**
     * Handle WebSocket connection opened
     */
    function handleWebSocketOpen(event) {
        console.log('Ground Vehicles WebSocket connection established.');
        
        // Start polling for ground vehicle data
        if (groundVehicleWsInterval) {
            clearInterval(groundVehicleWsInterval);
        }
        groundVehicleWsInterval = setInterval(sendGroundVehicleRequest, CONFIG.wsInterval);
    }

    /**
     * Handle WebSocket connection closed
     */
    function handleWebSocketClose(event) {
        console.log('Ground Vehicles WebSocket connection closed. Reconnecting...');
        
        if (groundVehicleWsInterval) {
            clearInterval(groundVehicleWsInterval);
            groundVehicleWsInterval = null;
        }
        
        scheduleReconnect();
    }

    /**
     * Handle WebSocket error
     */
    function handleWebSocketError(error) {
        console.error('Ground Vehicles WebSocket error:', error);
    }

    /**
     * Handle incoming WebSocket message
     */
    function handleWebSocketMessage(event) {
        try {
            const data = JSON.parse(event.data);
            
            if (data.error_msg && data.error_msg.length > 0) {
                console.warn('Ground Vehicles WebSocket error:', data.error_msg);
            }
            
            if (data.ground_vehicles) {
                handleGroundVehiclesUpdate(data.ground_vehicles);
            }
        } catch (error) {
            console.error('Error parsing Ground Vehicles WebSocket message:', error);
        }
    }

    /**
     * Send request for ground vehicle data
     */
    function sendGroundVehicleRequest() {
        if (groundVehicleWebSocket && groundVehicleWebSocket.readyState === WebSocket.OPEN) {
            const request = {
                operation_id: parseInt(OPERATION_ID)
            };
            groundVehicleWebSocket.send(JSON.stringify(request));
        }
    }

    /**
     * Schedule a reconnection attempt
     */
    function scheduleReconnect() {
        setTimeout(() => {
            console.log('Attempting to reconnect Ground Vehicles WebSocket...');
            initGroundVehiclesWebSocket();
        }, CONFIG.reconnectDelay);
    }

    /**
     * Update ground vehicles on map and panel based on received data
     */
    function handleGroundVehiclesUpdate(groundVehicles) {
        if(DEBUG_VEHICLES) {
            console.log(groundVehicles);
        }
        
        // Hide section if no vehicles
        if (Object.keys(allGroundVehicles).length === 0 && groundVehicles.length === 0) {
            const menuElement = document.getElementById("ground-vehicles-menu");
            if (menuElement) {
                menuElement.style.display = "none";
            }
            return;
        }

        // Process incoming vehicles
        if (groundVehicles && Array.isArray(groundVehicles)) {
            groundVehicles.forEach(vehicle => {
                const vehicleKey = vehicle.name;
                
                // Add new vehicle or update existing
                if (!allGroundVehicles[vehicleKey]) {
                    allGroundVehicles[vehicleKey] = vehicle;
                    placeGroundVehicleOnMap(vehicle);
                    // placeGroundVehicleOnPanel(vehicle);
                } else {
                    // Update existing vehicle
                    updateGroundVehicleOnMap(vehicle);
                    allGroundVehicles[vehicleKey] = vehicle;
                }
            });
        }

        // Remove vehicles that are no longer in the update
        Object.keys(allGroundVehicles).forEach(vehicleName => {
            if (!groundVehicles.some(vehicle => vehicle.name === vehicleName)) {
                console.log(`Ground vehicle ${vehicleName} not found in update, removing from map and panel.`);
                removeGroundVehicle(vehicleName);
            }
        });

        // Update UI
        updateGroundVehiclesUI();
    }

    /**
     * Place ground vehicle marker on the map
     */
    function placeGroundVehicleOnMap(vehicle) {
        if (!vehicle.lat || !vehicle.lon) {
            console.warn(`Ground vehicle ${vehicle.name} has no coordinates, skipping map placement.`);
            return;
        }

        // Create popup content
        const popupContent = `
            <div style="font-family: Arial, sans-serif; min-width: 200px;">
                <h4 style="margin: 0 0 10px 0; color: #333;">${vehicle.name}</h4>
                <table style="width: 100%; font-size: 12px;">
                    <tr><td><b>Model:</b></td><td>${vehicle.model}</td></tr>
                    ${vehicle.service ? `<tr><td><b>Service:</b></td><td>${vehicle.service}</td></tr>` : ''}
                    <tr><td><b>Status:</b></td><td>${vehicle.status}</td></tr>
                    <tr><td><b>Speed:</b></td><td>${vehicle.speed.toFixed(1)} km/h</td></tr>
                    <tr><td><b>Heading:</b></td><td>${vehicle.heading.toFixed(0)}°</td></tr>
                    <tr><td><b>Position:</b></td><td>${vehicle.lat.toFixed(6)}, ${vehicle.lon.toFixed(6)}</td></tr>
                </table>
            </div>
        `;

        // Create popup
        const popup = new maplibregl.Popup({ offset: 25 })
            .setHTML(popupContent);

        // Create marker element
        const el = document.createElement('div');
        el.className = 'ground-vehicle-marker';
        el.style.width = '30px';
        el.style.height = '30px';
        
        // Create inner element for the icon
        const innerEl = document.createElement('div');
        innerEl.style.cursor = 'pointer';
        innerEl.style.width = '100%';
        innerEl.style.height = '100%';
        innerEl.style.backgroundImage = `url(${ICONS[vehicle.service] || '/static/aiders/imgs/ccd-car-logo.png'})`;
        innerEl.style.backgroundSize = 'contain';
        innerEl.style.backgroundRepeat = 'no-repeat';
        innerEl.style.backgroundPosition = 'center';
        innerEl.style.filter = 'drop-shadow(0 2px 4px rgba(0,0,0,0.6))';
        
        el.appendChild(innerEl);

        // Create marker with map rotation
        const marker = new maplibregl.Marker({
            element: el, 
            anchor: 'center',
            rotationAlignment: 'map',
            rotation: vehicle.heading
        })
            .setLngLat([vehicle.lon, vehicle.lat])
            .setPopup(popup)
            .addTo(map);

        // Store marker
        allGroundVehicleMarkers[vehicle.name] = marker;

        console.log(`Ground vehicle ${vehicle.name} placed on map.`);
    }

    /**
     * Update existing ground vehicle marker on the map
     */
    function updateGroundVehicleOnMap(vehicle) {
        const marker = allGroundVehicleMarkers[vehicle.name];
        
        if (marker && vehicle.lat && vehicle.lon) {
            // Update position
            marker.setLngLat([vehicle.lon, vehicle.lat]);
            
            // Update marker rotation using MapLibre's rotation method
            marker.setRotation(vehicle.heading);
            
            // Update popup content
            const popupContent = `
                <div style="font-family: Arial, sans-serif; min-width: 200px;">
                    <h4 style="margin: 0 0 10px 0; color: #333;">${vehicle.name}</h4>
                    <table style="width: 100%; font-size: 12px;">
                        <tr><td><b>Model:</b></td><td>${vehicle.model}</td></tr>
                        ${vehicle.service ? `<tr><td><b>Service:</b></td><td>${vehicle.service}</td></tr>` : ''}
                        <tr><td><b>Status:</b></td><td>${vehicle.status}</td></tr>
                        <tr><td><b>Speed:</b></td><td>${vehicle.speed.toFixed(1)} km/h</td></tr>
                        <tr><td><b>Heading:</b></td><td>${vehicle.heading.toFixed(0)}°</td></tr>
                        <tr><td><b>Position:</b></td><td>${vehicle.lat.toFixed(6)}, ${vehicle.lon.toFixed(6)}</td></tr>
                    </table>
                </div>
            `;
            
            marker.getPopup().setHTML(popupContent);
        }
    }

    /**
     * Place ground vehicle entry in the side panel
     */
    function placeGroundVehicleOnPanel(vehicle) {
        const ul = document.getElementById('ground-vehicle-selection-list');
        if (!ul) {
            console.warn('Ground vehicle selection list not found in panel');
            return;
        }

        const li = document.createElement('li');
        li.id = `ground-vehicle-${vehicle.name}`;
        li.style.marginBottom = '10px';
        li.style.padding = '10px';
        li.style.backgroundColor = 'rgba(255, 255, 255, 0.1)';
        li.style.borderRadius = '4px';
        li.style.cursor = 'pointer';

        li.innerHTML = `
            <div style="color: #ffffff; font-size: 13px;">
                <div style="font-weight: bold; margin-bottom: 5px;">
                    <a href="#" onclick="zoomToGroundVehicle('${vehicle.name}')" style="color: #17b8be; text-decoration: none;">
                        ${vehicle.name}
                    </a>
                </div>
                <div style="font-size: 11px; opacity: 0.8;">
                    ${vehicle.model} ${vehicle.service ? `(${vehicle.service})` : ''}
                </div>
                <div style="font-size: 11px; margin-top: 3px;">
                    <span style="color: #4CAF50;">●</span> ${vehicle.status} | ${vehicle.speed.toFixed(0)} km/h
                </div>
            </div>
        `;

        ul.appendChild(li);
    }

    /**
     * Remove ground vehicle from map and panel
     */
    function removeGroundVehicle(vehicleName) {
        // Remove from map
        const marker = allGroundVehicleMarkers[vehicleName];
        if (marker) {
            marker.remove();
            delete allGroundVehicleMarkers[vehicleName];
        }

        // Remove from panel
        const panelEntry = document.getElementById(`ground-vehicle-${vehicleName}`);
        if (panelEntry) {
            panelEntry.remove();
        }

        // Remove from state
        delete allGroundVehicles[vehicleName];
    }

    /**
     * Update ground vehicles UI (counter and visibility)
     */
    function updateGroundVehiclesUI() {
        const menuElement = document.getElementById("ground-vehicles-menu");
        const counterElement = document.getElementById("connected-ground-vehicles");
        
        const vehicleCount = Object.keys(allGroundVehicles).length;
        
        if (vehicleCount > 0) {
            if (counterElement) {
                counterElement.innerHTML = vehicleCount;
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
     * Zoom to a specific ground vehicle on the map
     */
    function zoomToGroundVehicle(vehicleName) {
        const vehicle = allGroundVehicles[vehicleName];
        const marker = allGroundVehicleMarkers[vehicleName];
        
        if (vehicle && marker && vehicle.lat && vehicle.lon) {
            map.flyTo({
                center: [vehicle.lon, vehicle.lat],
                zoom: 16,
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
        if (groundVehicleWsInterval) {
            clearInterval(groundVehicleWsInterval);
            groundVehicleWsInterval = null;
        }
        
        if (groundVehicleWebSocket) {
            groundVehicleWebSocket.close();
            groundVehicleWebSocket = null;
        }

        // Remove all vehicles
        Object.keys(allGroundVehicles).forEach(vehicleName => {
            removeGroundVehicle(vehicleName);
        });
    }

    // Make zoomToGroundVehicle globally accessible
    window.zoomToGroundVehicle = zoomToGroundVehicle;

    // Initialize on map load
    if (typeof map !== 'undefined') {
        map.on('load', function() {
            console.log('Initializing Ground Vehicles WebSocket...');
            initGroundVehiclesWebSocket();
        });
    } else {
        console.warn('Map object not found, delaying Ground Vehicles WebSocket initialization');
        setTimeout(() => {
            if (typeof map !== 'undefined') {
                map.on('load', function() {
                    console.log('Initializing Ground Vehicles WebSocket...');
                    initGroundVehiclesWebSocket();
                });
            }
        }, 1000);
    }

    // Cleanup on page unload
    window.addEventListener('beforeunload', cleanup);

})();
