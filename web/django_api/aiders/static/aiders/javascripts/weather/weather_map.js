// Weather Map Integration for AIDERS Platform
// Displays weather data from WeatherAPI.com on the map with markers and popups

{
    let allWeatherData = {};
    let allWeatherPopups = {};
    let weatherUpdateInterval;
    let weatherExpirationCheckInterval;

    function initializeWeatherDisplay() {
        // Load initial weather data
        fetchWeatherDataForMap();
        
        // Set up periodic updates (every minute)
        weatherUpdateInterval = setInterval(fetchWeatherDataForMap, 60000);
        
        // Set up periodic expiration checks (every 5 minutes)
        weatherExpirationCheckInterval = setInterval(removeExpiredWeatherData, 300000);
        
        console.log('Weather display system initialized');
    }

    function fetchWeatherDataForMap() {
        // Fetch weather data from Django API
        console.log('Fetching weather data from API...');
        
        // Get CSRF token from the page
        let csrfToken = '';
        const csrfTokenElement = document.querySelector('[name=csrfmiddlewaretoken]');
        if (csrfTokenElement) {
            csrfToken = csrfTokenElement.value;
        } else {
            // Try to get from cookie as fallback
            csrfToken = getCookie('csrftoken');
        }
        
        fetch('/api/weather/map-data/', {
            method: 'GET',
            headers: {
                'X-CSRFToken': csrfToken,
                'Content-Type': 'application/json',
            },
            credentials: 'same-origin'  // Include cookies for authentication
        })
            .then(response => {
                console.log('API Response status:', response.status);
                if (!response.ok) {
                    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
                }
                return response.json();
            })
            .then(data => {
                console.log('Weather API response:', data);
                if (data.status === 'success' && data.weather_data) {
                    handleWeatherUpdate(data.weather_data);
                } else if (data.status === 'error') {
                    console.error('API Error:', data.message);
                }
            })
            .catch(error => {
                console.error('Error fetching weather data:', error);
            });
    }

    // Helper function to get cookie value
    function getCookie(name) {
        let cookieValue = null;
        if (document.cookie && document.cookie !== '') {
            const cookies = document.cookie.split(';');
            for (let i = 0; i < cookies.length; i++) {
                const cookie = cookies[i].trim();
                if (cookie.substring(0, name.length + 1) === (name + '=')) {
                    cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
                    break;
                }
            }
        }
        return cookieValue;
    }

    function isWeatherDataExpired(weather) {
        // Check if weather data is older than 1 hour
        const weatherTime = new Date(weather.timestamp);
        const oneHourAgo = new Date(Date.now() - 60 * 60 * 1000); // 1 hour in milliseconds
        return weatherTime < oneHourAgo;
    }

    function removeExpiredWeatherData() {
        // Remove any weather data that's older than 1 hour
        Object.keys(allWeatherData).forEach(cityKey => {
            const weather = allWeatherData[cityKey];
            if (weather && isWeatherDataExpired(weather)) {
                console.log(`Removing expired weather data for ${weather.city} (last updated: ${weather.timestamp})`);
                removeWeatherMarker(cityKey);
            }
        });
    }

    function handleWeatherUpdate(weatherData) {
        // First remove any expired data from the client side
        removeExpiredWeatherData();
        
        // Remove old markers that are no longer in the data
        Object.keys(allWeatherData).forEach(cityKey => {
            if (!weatherData.some(weather => `${weather.city}-${weather.country}` === cityKey)) {
                removeWeatherMarker(cityKey);
            }
        });

        // Add or update weather markers
        weatherData.forEach(weather => {
            const cityKey = `${weather.city}-${weather.country}`;
            
            // Double-check that the incoming data isn't already expired
            if (isWeatherDataExpired(weather)) {
                console.warn(`Received expired weather data for ${weather.city}, skipping`);
                return;
            }
            
            if (!allWeatherData[cityKey]) {
                // New weather location - add marker
                addWeatherMarker(weather, cityKey);
            } else {
                // Update existing weather data
                allWeatherData[cityKey] = weather;
                updateWeatherPopup(cityKey);
            }
        });

        console.log(`Updated weather data for ${weatherData.length} cities`);
    }

    function addWeatherMarker(weather, cityKey) {
        // Store weather data
        allWeatherData[cityKey] = weather;

        // Use coordinates from the weather data (provided by WeatherAPI.com)
        if (weather.latitude && weather.longitude) {
            placeWeatherMarkerOnMap(weather, cityKey, {
                lat: weather.latitude,
                lng: weather.longitude
            });
        } else {
            console.warn(`No coordinates available for ${weather.city}`);
        }
    }

    function placeWeatherMarkerOnMap(weather, cityKey, coords) {
        // Create popup content with weather details
        const popupContent = createWeatherPopupContent(weather);
        
        // Create a popup for the weather data
        const popup = new maplibregl.Popup({ 
            offset: [0, -15],
            maxWidth: '300px',
            closeButton: true,
            closeOnClick: false
        }).setHTML(popupContent);

        // Store the popup for later updates
        allWeatherPopups[cityKey] = popup;

        // Add weather data as a map source and layer instead of using markers
        const sourceId = `weather-${cityKey}`;
        const layerId = `weather-layer-${cityKey}`;
        
        // Create GeoJSON point for the weather location
        const weatherGeoJSON = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [coords.lng, coords.lat]
                },
                "properties": {
                    "city": weather.city,
                    "temperature": weather.temperature,
                    "condition": weather.weather_condition,
                    "icon": getWeatherIcon(weather.weather_condition),
                    "wind_speed": weather.wind_speed || 0,
                    "wind_direction": weather.wind_direction || 0,
                    "wind_dir_text": weather.wind_dir_text || 'N/A'
                }
            }]
        };

        // Add source to map
        map.addSource(sourceId, {
            "type": "geojson",
            "data": weatherGeoJSON
        });

        // Add circle layer for the weather marker background
        map.addLayer({
            "id": `${layerId}-circle`,
            "type": "circle",
            "source": sourceId,
            "paint": {
                "circle-radius": 28,
                "circle-color": getWeatherBgColor(weather.temperature),
                "circle-stroke-color": "#ffffff",
                "circle-stroke-width": 1,
                "circle-opacity": 0.9,
                "circle-stroke-opacity": 1
            }
        });

        // Add inner circle for depth effect
        // map.addLayer({
        //     "id": `${layerId}-inner-circle`,
        //     "type": "circle",
        //     "source": sourceId,
        //     "paint": {
        //         "circle-radius": 22,
        //         "circle-color": getWeatherBgColor(weather.temperature),
        //         "circle-opacity": 0.7
        //     }
        // });

        // Add symbol layer for the weather icon
        map.addLayer({
            "id": `${layerId}-symbol`,
            "type": "symbol",
            "source": sourceId,
            "layout": {
                "text-field": ["get", "icon"],
                "text-size": 18,
                "text-allow-overlap": true,
                "text-ignore-placement": true,
                "text-offset": [-0.6, -0.3]
            },
            "paint": {
                "text-color": "#ffffff",
                "text-halo-color": "rgba(0,0,0,0.3)",
                "text-halo-width": 1
            }
        });

        // Add temperature text layer
        map.addLayer({
            "id": `${layerId}-temp`,
            "type": "symbol",
            "source": sourceId,
            "layout": {
                "text-field": ["concat", ["to-string", ["round", ["get", "temperature"]]], "°"],
                "text-size": 11,
                "text-offset": [1, -0.6],
                "text-anchor": "center",
                "text-allow-overlap": true,
                "text-ignore-placement": true,
                "text-font": ["Open Sans Bold", "Arial Unicode MS Bold"]
            },
            "paint": {
                "text-color": "#ffffff",
                "text-halo-color": "rgba(0,0,0,0.5)",
                "text-halo-width": 1.5
            }
        });


        // Add wind speed text layer  
        map.addLayer({
            "id": `${layerId}-wind-speed`,
            "type": "symbol",
            "source": sourceId,
            "layout": {
                "text-field": ["concat", ["to-string", ["round", ["get", "wind_speed"]]], " kph"],
                "text-size": 11,
                "text-offset": [0, 1],
                "text-anchor": "center",
                "text-allow-overlap": true,
                "text-ignore-placement": true,
                "text-font": ["Open Sans Bold", "Arial Unicode MS Bold"]
            },
            "paint": {
                "text-color": "#ffffff",
                "text-halo-color": "rgba(0,0,0,0.5)",
                "text-halo-width": 1.5
            }
        });
        
        
        // Add wind direction arrow layer
        map.addLayer({
            "id": `${layerId}-wind`,
            "type": "symbol",
            "source": sourceId,
            "layout": {
                "text-field": "↑",
                "text-size": 20,
                // "text-size": getWindArrowSize(weather.wind_speed),
                "text-rotate": ["get", "wind_direction"],
                "text-offset": [0, -1.9],
                "text-anchor": "center",
                "text-allow-overlap": true,
                "text-ignore-placement": true
            },
            "paint": {
                "text-color": getWindSpeedColor(weather.wind_speed),
                "text-halo-color": "rgba(0,0,0,0.7)",
                "text-halo-width": 1
            }
        });



        // Add click events to main circle layer to show popup
        map.on('click', `${layerId}-circle`, (e) => {
            const coordinates = e.features[0].geometry.coordinates.slice();
            
            // Create fresh popup with current weather data to ensure it's always up-to-date
            const currentWeather = allWeatherData[cityKey];
            if (currentWeather) {
                const freshPopupContent = createWeatherPopupContent(currentWeather);
                const freshPopup = new maplibregl.Popup({ 
                    offset: [0, -15],
                    maxWidth: '300px',
                    closeButton: true,
                    closeOnClick: false
                }).setHTML(freshPopupContent);
                
                freshPopup.setLngLat(coordinates).addTo(map);
                
                // Update stored popup reference
                if (allWeatherPopups[cityKey]) {
                    allWeatherPopups[cityKey].remove();
                }
                allWeatherPopups[cityKey] = freshPopup;
            }
        });

        // Add hover effects to main circle layer
        map.on('mouseenter', `${layerId}-circle`, () => {
            map.getCanvas().style.cursor = 'pointer';
            // Add hover effect by increasing opacity
            map.setPaintProperty(`${layerId}-circle`, 'circle-opacity', 1.0);
        });

        map.on('mouseleave', `${layerId}-circle`, () => {
            map.getCanvas().style.cursor = '';
            // Reset opacity
            map.setPaintProperty(`${layerId}-circle`, 'circle-opacity', 0.9);
        });

        console.log(`Weather layer placed for ${weather.city} at [${coords.lng}, ${coords.lat}]`);
    }

    function createWeatherPopupContent(weather) {
        const timestamp = new Date(weather.timestamp);
        const now = new Date();
        const ageInMinutes = Math.floor((now - timestamp) / (1000 * 60));
        
        // Determine freshness indicator
        let freshnessIndicator = '';
        let freshnessColor = '';
        if (ageInMinutes < 5) {
            freshnessIndicator = '🟢 Very Fresh';
            freshnessColor = '#10b981';
        } else if (ageInMinutes < 15) {
            freshnessIndicator = '🟡 Fresh';
            freshnessColor = '#f59e0b';
        } else if (ageInMinutes < 30) {
            freshnessIndicator = '🟠 Moderately Fresh';
            freshnessColor = '#f97316';
        } else {
            freshnessIndicator = '🔴 Getting Stale';
            freshnessColor = '#ef4444';
        }
        
        const timeAgo = ageInMinutes < 60 
            ? `${ageInMinutes} minutes ago`
            : `${Math.floor(ageInMinutes / 60)} hour(s) ago`;
        
        return `
            <div class="weather-popup">
                <div class="weather-popup-header">
                    <h4 style="margin: 0; color: #333;">
                        ${getWeatherIcon(weather.weather_condition)} 
                        ${weather.city}
                    </h4>
                    <p style="margin: 0; font-size: 12px; color: #666;">
                        ${weather.region ? weather.region + ', ' : ''}${weather.country}
                    </p>
                </div>
                
                <div class="weather-popup-content" style="margin-top: 10px;">
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;">
                        <span style="font-size: 18px; font-weight: bold;">
                            ${Math.round(weather.temperature)}°C
                        </span>
                        <span style="color: #666; font-size: 14px;">
                            Feels like ${Math.round(weather.feels_like)}°C
                        </span>
                    </div>
                    
                    <div style="color: #444; font-weight: 500; margin-bottom: 8px;">
                        ${weather.weather_condition}
                    </div>
                    
                    <!-- Enhanced Wind Information Section -->
                    <div style="background: linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 100%); padding: 8px; border-radius: 6px; margin-bottom: 8px;">
                        <div style="display: flex; align-items: center; gap: 8px; font-weight: 600; color: #0369a1;">
                            <span style="font-size: 14px;">💨</span>
                            <span>Wind: ${weather.wind_speed || 0} kph ${weather.wind_dir_text || 'N/A'}</span>
                        </div>
                        <div style="font-size: 11px; color: #0284c7; margin-top: 2px;">
                            Direction: ${weather.wind_direction || 0}° • ${getWindDescription(weather.wind_speed || 0)}
                        </div>
                    </div>
                    
                    <div class="weather-details" style="font-size: 12px; color: #666;">
                        <div><strong>Humidity:</strong> ${weather.humidity}%</div>
                        <div><strong>Pressure:</strong> ${weather.pressure} mb</div>
                        <div><strong>Visibility:</strong> ${weather.visibility} km</div>
                        ${weather.uv_index ? `<div><strong>UV Index:</strong> ${weather.uv_index}</div>` : ''}
                        ${weather.cloud_cover !== null ? `<div><strong>Cloud Cover:</strong> ${weather.cloud_cover}%</div>` : ''}
                    </div>
                    
                    <div style="margin-top: 8px; font-size: 11px; border-top: 1px solid #eee; padding-top: 5px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <span style="color: #888;">Updated: ${timeAgo}</span>
                            <span style="color: ${freshnessColor}; font-weight: 600;">${freshnessIndicator}</span>
                        </div>
                        <div style="color: #999; font-size: 10px; margin-top: 2px;">${timestamp.toUTCString()}</div>
                    </div>
                </div>
            </div>
        `;
    }

    function getWeatherIcon(condition) {
        const conditionLower = condition.toLowerCase();
        
        if (conditionLower.includes('sunny') || conditionLower.includes('clear')) return '☀️';
        if (conditionLower.includes('partly cloudy')) return '⛅';
        if (conditionLower.includes('cloudy') || conditionLower.includes('overcast')) return '☁️';
        if (conditionLower.includes('rain') || conditionLower.includes('drizzle')) return '🌧️';
        if (conditionLower.includes('thunder') || conditionLower.includes('storm')) return '⛈️';
        if (conditionLower.includes('snow')) return '❄️';
        if (conditionLower.includes('fog') || conditionLower.includes('mist')) return '🌫️';
        if (conditionLower.includes('wind')) return '💨';
        
        return '🌤️'; // Default icon
    }

    function getWindArrowSize(windSpeed) {
        // Scale arrow size based on wind speed
        if (windSpeed < 5) return 10;      // Light air
        if (windSpeed < 15) return 12;     // Light breeze
        if (windSpeed < 25) return 14;     // Moderate breeze
        if (windSpeed < 40) return 16;     // Strong breeze
        if (windSpeed < 60) return 18;     // Gale
        return 20;                         // Storm/Hurricane
    }

    function getWindSpeedColor(windSpeed) {
        // Color code wind speed for easy identification
        if (windSpeed < 5) return '#94a3b8';       // Light - gray
        if (windSpeed < 15) return '#22d3ee';      // Light breeze - cyan
        if (windSpeed < 25) return '#10b981';      // Moderate - green
        if (windSpeed < 40) return '#f59e0b';      // Strong - amber
        if (windSpeed < 60) return '#ef4444';      // Gale - red
        return '#dc2626';                          // Storm - dark red
    }

    function getWindDescription(windSpeed) {
        // Beaufort scale descriptions
        if (windSpeed < 1) return 'Calm';
        if (windSpeed < 6) return 'Light air';
        if (windSpeed < 12) return 'Light breeze';
        if (windSpeed < 20) return 'Gentle breeze';
        if (windSpeed < 29) return 'Moderate breeze';
        if (windSpeed < 39) return 'Fresh breeze';
        if (windSpeed < 50) return 'Strong breeze';
        if (windSpeed < 62) return 'Near gale';
        if (windSpeed < 75) return 'Gale';
        if (windSpeed < 89) return 'Strong gale';
        if (windSpeed < 103) return 'Storm';
        return 'Hurricane';
    }

    function getWeatherBgColor(temperature) {
        if (temperature < -10) return '#0f172a';    // Arctic - very dark blue
        if (temperature < 0) return '#1e40af';      // Freezing - dark blue
        if (temperature < 5) return '#3b82f6';      // Very cold - blue
        if (temperature < 10) return '#06b6d4';     // Cold - cyan
        if (temperature < 15) return '#10b981';     // Cool - emerald
        if (temperature < 20) return '#84cc16';     // Mild - lime
        if (temperature < 25) return '#eab308';     // Warm - yellow
        if (temperature < 30) return '#f97316';     // Hot - orange
        if (temperature < 35) return '#ef4444';     // Very hot - red
        return '#dc2626';                           // Extreme heat - dark red
    }

    function updateWeatherPopup(cityKey) {
        const weather = allWeatherData[cityKey];
        
        if (weather) {
            // Update the popup content with new weather data
            if (allWeatherPopups[cityKey]) {
                const updatedPopupContent = createWeatherPopupContent(weather);
                allWeatherPopups[cityKey].setHTML(updatedPopupContent);
            }
            
            // For layer-based weather markers, we need to update the layer data
            const sourceId = `weather-${cityKey}`;
            const layerId = `weather-layer-${cityKey}`;
            
            if (map.getSource(sourceId)) {
                // Create updated GeoJSON data with new weather information
                const coords = {
                    lat: weather.latitude,
                    lng: weather.longitude
                };
                
                const updatedGeoJSON = {
                    "type": "FeatureCollection",
                    "features": [{
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": [coords.lng, coords.lat]
                        },
                        "properties": {
                            "city": weather.city,
                            "temperature": weather.temperature,
                            "condition": weather.weather_condition,
                            "icon": getWeatherIcon(weather.weather_condition),
                            "wind_speed": weather.wind_speed || 0,
                            "wind_direction": weather.wind_direction || 0,
                            "wind_dir_text": weather.wind_dir_text || 'N/A'
                        }
                    }]
                };
                
                // Update the source data
                map.getSource(sourceId).setData(updatedGeoJSON);
                
                // Update background color
                const bgColor = getWeatherBgColor(weather.temperature);
                if (map.getLayer(`${layerId}-circle`)) {
                    map.setPaintProperty(`${layerId}-circle`, 'circle-color', bgColor);
                }
                
                // Update wind arrow color based on wind speed
                if (map.getLayer(`${layerId}-wind`)) {
                    const windColor = getWindSpeedColor(weather.wind_speed || 0);
                    map.setPaintProperty(`${layerId}-wind`, 'text-color', windColor);
                }
            }
        }
    }

    function removeWeatherMarker(cityKey) {
        if (allWeatherData[cityKey]) {
            // Remove the layers using the correct layer IDs
            const layerId = `weather-layer-${cityKey}`;
            const sourceId = `weather-${cityKey}`;
            
            // Check if layers exist before removing
            if (map.getLayer(`${layerId}-circle`)) map.removeLayer(`${layerId}-circle`);
            if (map.getLayer(`${layerId}-symbol`)) map.removeLayer(`${layerId}-symbol`);
            if (map.getLayer(`${layerId}-temp`)) map.removeLayer(`${layerId}-temp`);
            if (map.getLayer(`${layerId}-wind`)) map.removeLayer(`${layerId}-wind`);
            if (map.getLayer(`${layerId}-wind-speed`)) map.removeLayer(`${layerId}-wind-speed`);
            
            // Remove the source
            if (map.getSource(sourceId)) map.removeSource(sourceId);
            
            // Clean up stored popup
            if (allWeatherPopups[cityKey]) {
                allWeatherPopups[cityKey].remove();
                delete allWeatherPopups[cityKey];
            }
            
            delete allWeatherData[cityKey];
            console.log(`Removed weather marker for ${cityKey}`);
        }
    }

    async function getCityCoordinates(city, country) {
        // First try with a simple geocoding service or use your existing geocoding
        // For now, I'll implement a basic coordinate lookup
        try {
            const query = `${city}, ${country}`;
            const response = await fetch(`https://api.mapbox.com/geocoding/v5/mapbox.places/${encodeURIComponent(query)}.json?access_token=${MapBoxToken}&types=place`);
            const data = await response.json();
            
            if (data.features && data.features.length > 0) {
                const [lng, lat] = data.features[0].center;
                return { lat, lng };
            }
        } catch (error) {
            console.error('Geocoding error:', error);
        }
        
        // Fallback to some default coordinates or city database
        return getDefaultCityCoordinates(city, country);
    }

    function getDefaultCityCoordinates(city, country) {
        // Basic city coordinate lookup - you can expand this
        const cityCoords = {
            'London,GB': { lat: 51.5074, lng: -0.1278 },
            'New York,US': { lat: 40.7128, lng: -74.0060 },
            'Paris,France': { lat: 48.8566, lng: 2.3522 },
            'Tokyo,Japan': { lat: 35.6762, lng: 139.6503 },
            'Berlin,Germany': { lat: 52.5200, lng: 13.4050 },
            'Madrid,Spain': { lat: 40.4168, lng: -3.7038 },
            'Rome,Italy': { lat: 41.9028, lng: 12.4964 },
            'Athens,Greece': { lat: 37.9838, lng: 23.7275 },
            'Nicosia,Cyprus': { lat: 35.1856, lng: 33.3823 },
        };
        
        const key = `${city},${country}`;
        return cityCoords[key] || null;
    }

    // Toggle weather display on/off
    function toggleWeatherDisplay(show) {
        Object.keys(allWeatherData).forEach(cityKey => {
            const layerId = `weather-layer-${cityKey}`;
            
            // Toggle layer visibility
            const visibility = show ? 'visible' : 'none';
            
            if (map.getLayer(`${layerId}-circle`)) {
                map.setLayoutProperty(`${layerId}-circle`, 'visibility', visibility);
            }
            if (map.getLayer(`${layerId}-symbol`)) {
                map.setLayoutProperty(`${layerId}-symbol`, 'visibility', visibility);
            }
            if (map.getLayer(`${layerId}-temp`)) {
                map.setLayoutProperty(`${layerId}-temp`, 'visibility', visibility);
            }
            if (map.getLayer(`${layerId}-wind`)) {
                map.setLayoutProperty(`${layerId}-wind`, 'visibility', visibility);
            }
            if (map.getLayer(`${layerId}-wind-speed`)) {
                map.setLayoutProperty(`${layerId}-wind-speed`, 'visibility', visibility);
            }
        });
    }

    // Function called from the platform toggle
    function toggleWeatherMapDisplay(isChecked) {
        toggleWeatherDisplay(isChecked);
        console.log('Weather map display:', isChecked ? 'enabled' : 'disabled');
    }

    // Function to refresh weather data manually
    function refreshWeatherData() {
        console.log('Refreshing weather data...');
        fetchWeatherDataForMap();
    }

    // Update weather location count
    function updateWeatherLocationCount() {
        const count = Object.keys(allWeatherData).length;
        const countElement = document.getElementById('weatherLocationCount');
        if (countElement) {
            countElement.textContent = count;
        }
    }

    // Modified handleWeatherUpdate to update count
    function handleWeatherUpdate(weatherData) {
        // Remove old markers that are no longer in the data
        Object.keys(allWeatherData).forEach(cityKey => {
            if (!weatherData.some(weather => `${weather.city}-${weather.country}` === cityKey)) {
                removeWeatherMarker(cityKey);
            }
        });

        // Add or update weather markers
        weatherData.forEach(weather => {
            const cityKey = `${weather.city}-${weather.country}`;
            
            // Skip weather data without coordinates (older records)
            if (!weather.latitude || !weather.longitude) {
                console.warn(`Skipping ${weather.city} - no coordinates available`);
                return;
            }
            
            if (!allWeatherData[cityKey]) {
                // New weather location - add marker
                addWeatherMarker(weather, cityKey);
            } else {
                // Update existing weather data
                allWeatherData[cityKey] = weather;
                updateWeatherPopup(cityKey);
            }
        });

        // Update the count display
        updateWeatherLocationCount();

        console.log(`Updated weather data for ${weatherData.length} cities`);
    }

    // Initialize when map is loaded
    if (typeof map !== 'undefined') {
        map.on('load', () => {
            initializeWeatherDisplay();
        });
    } else {
        // Wait for map to be available
        const checkMap = setInterval(() => {
            if (typeof map !== 'undefined') {
                clearInterval(checkMap);
                map.on('load', () => {
                    initializeWeatherDisplay();
                });
            }
        }, 100);
    }

    // Expose functions globally if needed
    window.weatherMapUtils = {
        toggleWeatherDisplay,
        fetchWeatherDataForMap,
        initializeWeatherDisplay,
        toggleWeatherMapDisplay,
        refreshWeatherData
    };

    // Make functions available globally for HTML onclick handlers
    window.toggleWeatherMapDisplay = toggleWeatherMapDisplay;
    window.refreshWeatherData = refreshWeatherData;
}
