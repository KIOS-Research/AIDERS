/**
 * Drone Remote IDs WebSocket Client
 * 
 * This module handles real-time streaming of drone Remote ID data through a WebSocket connection.
 * It maintains a persistent connection to the Go WebSocket server and updates drone RID
 * positions and information on the map and side panel.
 * 
 * Features:
 * - Real-time drone Remote ID tracking (updates within last 60 seconds)
 * - Map markers with popups showing device information
 * - Side panel integration with device list
 * - Automatic connection management with reconnection logic
 * 
 * Dependencies:
 * - maplibregl (for map markers and popups)
 * - Global variables: OPERATION_ID, TOKEN, NGINX_PORT
 */

(function() {
    'use strict';

    const DEBUG_DRIDS = false;

    // Configuration
    const CONFIG = {
        wsInterval: 1000, // WebSocket polling interval in milliseconds
        reconnectDelay: 2000, // Delay before attempting to reconnect in milliseconds
        wsEndpoint: '/ws/getDroneRids',
    };

    // State management
    let allDroneRids = {};
    let allDroneRidMarkers = {};
    let droneRidWebSocket = null;
    let droneRidWsInterval = null;

    const DRONE_ICON = '/static/aiders/imgs/drone-icon.png';

    /**
     * Initialize the drone RIDs WebSocket connection
     */
    function initDroneRidsWebSocket() {
        const wsAddress = `ws://${window.location.hostname}:${NGINX_PORT}${CONFIG.wsEndpoint}`;
        console.log('Connecting to Drone RIDs WebSocket:', wsAddress);

        try {
            droneRidWebSocket = new WebSocket(`${wsAddress}?token=${encodeURIComponent(TOKEN)}`);

            droneRidWebSocket.addEventListener('open', handleWebSocketOpen);
            droneRidWebSocket.addEventListener('close', handleWebSocketClose);
            droneRidWebSocket.addEventListener('error', handleWebSocketError);
            droneRidWebSocket.addEventListener('message', handleWebSocketMessage);

        } catch (error) {
            console.error('Error creating Drone RIDs WebSocket:', error);
            scheduleReconnect();
        }
    }

    /**
     * Handle WebSocket connection opened
     */
    function handleWebSocketOpen(event) {
        console.log('Drone RIDs WebSocket connection established.');
        
        // Start polling for drone RID data
        if (droneRidWsInterval) {
            clearInterval(droneRidWsInterval);
        }
        droneRidWsInterval = setInterval(sendDroneRidRequest, CONFIG.wsInterval);
    }

    /**
     * Handle WebSocket connection closed
     */
    function handleWebSocketClose(event) {
        console.log('Drone RIDs WebSocket connection closed. Reconnecting...');
        
        if (droneRidWsInterval) {
            clearInterval(droneRidWsInterval);
            droneRidWsInterval = null;
        }
        
        scheduleReconnect();
    }

    /**
     * Handle WebSocket error
     */
    function handleWebSocketError(error) {
        console.error('Drone RIDs WebSocket error:', error);
    }

    /**
     * Handle incoming WebSocket message
     */
    function handleWebSocketMessage(event) {
        try {
            const data = JSON.parse(event.data);
            
            if (data.error_msg && data.error_msg.length > 0) {
                console.warn('Drone RIDs WebSocket error:', data.error_msg);
            }
            
            if (data.drone_rids) {
                handleDroneRidsUpdate(data.drone_rids);
            }
        } catch (error) {
            console.error('Error parsing Drone RIDs WebSocket message:', error);
        }
    }

    /**
     * Send request for drone RID data
     */
    function sendDroneRidRequest() {
        if (droneRidWebSocket && droneRidWebSocket.readyState === WebSocket.OPEN) {
            const request = {
                operation_id: parseInt(OPERATION_ID)
            };
            droneRidWebSocket.send(JSON.stringify(request));
        }
    }

    /**
     * Schedule a reconnection attempt
     */
    function scheduleReconnect() {
        setTimeout(() => {
            console.log('Attempting to reconnect Drone RIDs WebSocket...');
            initDroneRidsWebSocket();
        }, CONFIG.reconnectDelay);
    }

    /**
     * Update drone RIDs on map and panel based on received data
     */
    function handleDroneRidsUpdate(droneRids) {
        if (DEBUG_DRIDS) {
            console.log('Drone RIDs update:', droneRids);
        }
        
        // Hide section if no devices
        if (Object.keys(allDroneRids).length === 0 && droneRids.length === 0) {
            const menuElement = document.getElementById("drone-rids-menu");
            if (menuElement) {
                menuElement.style.display = "none";
            }
            return;
        }

        // Process incoming devices
        if (droneRids && Array.isArray(droneRids)) {
            droneRids.forEach(rid => {
                const ridKey = rid.address;
                
                // Add new device or update existing
                if (!allDroneRids[ridKey]) {
                    allDroneRids[ridKey] = rid;
                    placeDroneRidOnMap(rid);
                    // placeDroneRidOnPanel(rid);
                } else {
                    // Update existing device
                    updateDroneRidOnMap(rid);
                    allDroneRids[ridKey] = rid;
                }
            });
        }

        // Remove devices that are no longer in the update
        Object.keys(allDroneRids).forEach(address => {
            if (!droneRids.some(rid => rid.address === address)) {
                console.log(`Drone RID ${address} not found in update, removing from map and panel.`);
                removeDroneRid(address);
            }
        });

        // Update UI
        updateDroneRidsUI();
    }

    /**
     * Place drone RID marker on the map
     */
    function placeDroneRidOnMap(rid) {
        if (!rid.lat || !rid.lon) {
            console.warn(`Drone RID ${rid.address} has no coordinates, skipping map placement.`);
            return;
        }

        // Create popup content
        const popupContent = `
            <div style="font-family: Arial, sans-serif; min-width: 220px;">
                <h4 style="margin: 0 0 10px 0; color: #333;">Drone Remote ID</h4>
                <table style="width: 100%; font-size: 12px;">
                    <tr><td><b>Address:</b></td><td>${rid.address}</td></tr>
                    ${rid.basic_id ? `<tr><td><b>Serial:</b></td><td>${rid.basic_id}</td></tr>` : ''}
                    ${rid.operator_id ? `<tr><td><b>Operator:</b></td><td>${rid.operator_id}</td></tr>` : ''}
                    ${rid.name && rid.name !== 'Unknown' ? `<tr><td><b>Name:</b></td><td>${rid.name}</td></tr>` : ''}
                    ${rid.altitude_m !== null ? `<tr><td><b>Altitude:</b></td><td>${rid.altitude_m.toFixed(1)} m</td></tr>` : ''}
                    ${rid.speed_m_s !== null ? `<tr><td><b>Speed:</b></td><td>${(rid.speed_m_s * 3.6).toFixed(1)} km/h</td></tr>` : ''}
                    ${rid.rssi !== null ? `<tr><td><b>Signal:</b></td><td>${rid.rssi} dBm</td></tr>` : ''}
                    <tr><td><b>Position:</b></td><td>${rid.lat.toFixed(6)}, ${rid.lon.toFixed(6)}</td></tr>
                    <tr><td><b>Last Seen:</b></td><td>${new Date(rid.last_seen).toLocaleTimeString()}</td></tr>
                </table>
            </div>
        `;

        // Create popup
        const popup = new maplibregl.Popup({ offset: 25 })
            .setHTML(popupContent);

        // Create marker element
        const el = document.createElement('div');
        el.className = 'drone-rid-marker';
        el.style.width = '32px';
        el.style.height = '32px';
        
        // Create inner element for the icon
        const innerEl = document.createElement('div');
        innerEl.style.cursor = 'pointer';
        innerEl.style.width = '100%';
        innerEl.style.height = '100%';
        innerEl.style.backgroundImage = `url(${DRONE_ICON})`;
        innerEl.style.backgroundSize = 'contain';
        innerEl.style.backgroundRepeat = 'no-repeat';
        innerEl.style.backgroundPosition = 'center';
        innerEl.style.filter = 'drop-shadow(0 2px 4px rgba(0,0,0,0.6))';
        innerEl.style.opacity = '0.9';
        
        el.appendChild(innerEl);

        // Create marker
        const marker = new maplibregl.Marker({
            element: el, 
            anchor: 'center'
        })
            .setLngLat([rid.lon, rid.lat])
            .setPopup(popup)
            .addTo(map);

        // Store marker
        allDroneRidMarkers[rid.address] = marker;

        console.log(`Drone RID ${rid.address} placed on map.`);
    }

    /**
     * Update existing drone RID marker on the map
     */
    function updateDroneRidOnMap(rid) {
        const marker = allDroneRidMarkers[rid.address];
        
        if (marker && rid.lat && rid.lon) {
            // Update position
            marker.setLngLat([rid.lon, rid.lat]);
            
            // Update popup content
            const popupContent = `
                <div style="font-family: Arial, sans-serif; min-width: 220px;">
                    <h4 style="margin: 0 0 10px 0; color: #333;">Drone Remote ID</h4>
                    <table style="width: 100%; font-size: 12px;">
                        <tr><td><b>Address:</b></td><td>${rid.address}</td></tr>
                        ${rid.basic_id ? `<tr><td><b>Serial:</b></td><td>${rid.basic_id}</td></tr>` : ''}
                        ${rid.operator_id ? `<tr><td><b>Operator:</b></td><td>${rid.operator_id}</td></tr>` : ''}
                        ${rid.name && rid.name !== 'Unknown' ? `<tr><td><b>Name:</b></td><td>${rid.name}</td></tr>` : ''}
                        ${rid.altitude_m !== null ? `<tr><td><b>Altitude:</b></td><td>${rid.altitude_m.toFixed(1)} m</td></tr>` : ''}
                        ${rid.speed_m_s !== null ? `<tr><td><b>Speed:</b></td><td>${(rid.speed_m_s * 3.6).toFixed(1)} km/h</td></tr>` : ''}
                        ${rid.rssi !== null ? `<tr><td><b>Signal:</b></td><td>${rid.rssi} dBm</td></tr>` : ''}
                        <tr><td><b>Position:</b></td><td>${rid.lat.toFixed(6)}, ${rid.lon.toFixed(6)}</td></tr>
                        <tr><td><b>Last Seen:</b></td><td>${new Date(rid.last_seen).toLocaleTimeString()}</td></tr>
                    </table>
                </div>
            `;
            
            marker.getPopup().setHTML(popupContent);
        }
    }

    /**
     * Place drone RID entry in the side panel
     */
    function placeDroneRidOnPanel(rid) {
        const ul = document.getElementById('drone-rid-selection-list');
        if (!ul) {
            console.warn('Drone RID selection list not found in panel');
            return;
        }

        const li = document.createElement('li');
        li.id = `drone-rid-${rid.address}`;
        li.style.marginBottom = '10px';
        li.style.padding = '10px';
        li.style.backgroundColor = 'rgba(255, 255, 255, 0.1)';
        li.style.borderRadius = '4px';
        li.style.cursor = 'pointer';

        const displayName = rid.basic_id || rid.operator_id || rid.address;
        const altitude = rid.altitude_m !== null ? `${rid.altitude_m.toFixed(0)}m` : 'N/A';
        const signal = rid.rssi !== null ? `${rid.rssi}dBm` : 'N/A';

        li.innerHTML = `
            <div style="color: #ffffff; font-size: 13px;">
                <div style="font-weight: bold; margin-bottom: 5px;">
                    <a href="#" onclick="zoomToDroneRid('${rid.address}')" style="color: #17b8be; text-decoration: none;">
                        ${displayName}
                    </a>
                </div>
                <div style="font-size: 11px; opacity: 0.8;">
                    ${rid.address}
                </div>
                <div style="font-size: 11px; margin-top: 3px;">
                    <span style="color: #4CAF50;">●</span> Alt: ${altitude} | Signal: ${signal}
                </div>
            </div>
        `;

        ul.appendChild(li);
    }

    /**
     * Remove drone RID from map and panel
     */
    function removeDroneRid(address) {
        // Remove from map
        const marker = allDroneRidMarkers[address];
        if (marker) {
            marker.remove();
            delete allDroneRidMarkers[address];
        }

        // Remove from panel
        const panelEntry = document.getElementById(`drone-rid-${address}`);
        if (panelEntry) {
            panelEntry.remove();
        }

        // Remove from state
        delete allDroneRids[address];
    }

    /**
     * Update drone RIDs UI (counter and visibility)
     */
    function updateDroneRidsUI() {
        const menuElement = document.getElementById("drone-rids-menu");
        const counterElement = document.getElementById("connected-drone-rids");
        
        const ridCount = Object.keys(allDroneRids).length;
        
        if (ridCount > 0) {
            if (counterElement) {
                counterElement.innerHTML = ridCount;
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
     * Zoom to a specific drone RID on the map
     */
    function zoomToDroneRid(address) {
        const rid = allDroneRids[address];
        const marker = allDroneRidMarkers[address];
        
        if (rid && marker && rid.lat && rid.lon) {
            map.flyTo({
                center: [rid.lon, rid.lat],
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
        if (droneRidWsInterval) {
            clearInterval(droneRidWsInterval);
            droneRidWsInterval = null;
        }
        
        if (droneRidWebSocket) {
            droneRidWebSocket.close();
            droneRidWebSocket = null;
        }

        // Remove all devices
        Object.keys(allDroneRids).forEach(address => {
            removeDroneRid(address);
        });
    }

    // Make zoomToDroneRid globally accessible
    window.zoomToDroneRid = zoomToDroneRid;

    // Initialize on map load
    if (typeof map !== 'undefined') {
        map.on('load', function() {
            console.log('Initializing Drone RIDs WebSocket...');
            initDroneRidsWebSocket();
        });
    } else {
        console.warn('Map object not found, delaying Drone RIDs WebSocket initialization');
        setTimeout(() => {
            if (typeof map !== 'undefined') {
                map.on('load', function() {
                    console.log('Initializing Drone RIDs WebSocket...');
                    initDroneRidsWebSocket();
                });
            }
        }, 1000);
    }

    // Cleanup on page unload
    window.addEventListener('beforeunload', cleanup);

})();
