# ADS-B Aircraft Tracking for AIDERS Platform

This document describes the ADS-B (Automatic Dependent Surveillance-Broadcast) aircraft tracking system integrated into the AIDERS platform.

## Overview

The ADS-B system provides real-time aircraft tracking capabilities by displaying live aircraft positions on the MapLibre map. It fetches aircraft data from the OpenSky Network API and presents it in an intuitive interface.

## Features

- **Real-time aircraft tracking**: Display live aircraft positions with 2-second updates
- **Interactive aircraft markers**: Click on aircraft to see detailed information
- **Automatic bounds detection**: Only shows aircraft visible in the current map view
- **Fallback demo data**: Provides demo aircraft when API is unavailable
- **Layer control**: Toggle aircraft visibility from the Map Layers panel
- **Tools integration**: Control tracking from the Tools sidebar panel

## Components

### Frontend (JavaScript)
- **File**: `static/aiders/javascripts/adsb/adsb_aircraft.js`
- **Functionality**: Handles map display, user interaction, and data fetching
- **Dependencies**: MapLibre GL JS, existing AIDERS platform utilities

### Backend (Django)
- **File**: `aiders/views_adsb.py`
- **Endpoints**: 
  - `/api/operations/<operation_name>/adsb/aircraft` - Get aircraft data
  - `/api/operations/<operation_name>/adsb/aircraft/<icao24>` - Get specific aircraft info

### Styling
- **File**: `static/aiders/css/adsb/adsb_aircraft.css`
- **Features**: Aircraft popup styling, control panel appearance, responsive design

### Management Commands
- **File**: `aiders/management/commands/test_adsb.py`
- **Usage**: Test ADS-B functionality and generate demo data

## API Integration

### OpenSky Network API
- **Base URL**: `https://opensky-network.org/api/states/all`
- **Method**: GET requests with bounding box parameters
- **Rate Limiting**: Data cached for 30 seconds to avoid API limits
- **Fallback**: Demo data generation when API is unavailable

### Data Format
Aircraft objects contain:
```javascript
{
    icao24: "string",           // Unique aircraft identifier
    callsign: "string",         // Flight callsign
    origin_country: "string",   // Country of registration
    latitude: number,           // Current latitude
    longitude: number,          // Current longitude
    altitude: number,           // Barometric altitude (meters)
    velocity: number,           // Ground speed (m/s)
    heading: number,            // Track angle (degrees)
    vertical_rate: number,      // Vertical velocity (m/s)
    last_contact: number,       // Unix timestamp of last contact
    on_ground: boolean,         // Ground status
    timestamp: "ISO string"     // Data timestamp
}
```

## User Interface

### Tools Panel Controls
Located in the sidebar Tools section:
- **Toggle Switch**: Enable/disable aircraft tracking
- **Aircraft Counter**: Shows number of visible aircraft
- **Refresh Button**: Force refresh of aircraft data

### Map Layers Panel
Located in the Map Layers section:
- **ADS-B Aircraft**: Toggle aircraft layer visibility

### Aircraft Popups
Clicking on an aircraft marker shows:
- Callsign and ICAO24 identifier
- Country of origin
- Current position (coordinates)
- Altitude and ground speed
- Heading and vertical rate
- Last contact time

## Installation & Setup

### 1. Files Added
The following files have been added to the project:
```
web/django_api/aiders/static/aiders/javascripts/adsb/adsb_aircraft.js
web/django_api/aiders/static/aiders/css/adsb/adsb_aircraft.css
web/django_api/aiders/views_adsb.py
web/django_api/aiders/management/commands/test_adsb.py
```

### 2. Files Modified
- `platform.html`: Added controls and script includes
- `urls.py`: Added ADS-B API endpoints

### 3. Dependencies
- `requests==2.24.0` (already included in requirements.txt)
- Django caching framework (for API rate limiting)

## Usage

### Enabling Aircraft Tracking
1. Navigate to the AIDERS platform
2. Open the Tools panel in the sidebar
3. Find "ADS-B Aircraft Tracking" option
4. Toggle the switch to enable tracking
5. Aircraft will appear on the map with airplane icons

### Viewing Aircraft Information
1. Click on any aircraft icon on the map
2. A popup will display detailed aircraft information
3. The popup shows real-time data including position, altitude, speed, and heading


## Configuration

### Update Intervals
- Aircraft data updates every 2 seconds
- API responses cached for 30 seconds
- Demo data includes 3-8 simulated aircraft

### Map Bounds
- Only aircraft within the current map viewport are fetched
- Bounds automatically calculated from map view
- Reduces API calls and improves performance

### Demo Mode
When the OpenSky API is unavailable:
- Automatically falls back to demo data
- Generates 6 simulated aircraft around map center
- Demo aircraft have realistic flight parameters

## Troubleshooting

### Common Issues
1. **No aircraft visible**: Check internet connection, API may be down
2. **CORS errors**: Use Django API endpoint instead of direct OpenSky calls
3. **Performance issues**: Ensure caching is working properly
4. **Demo data only**: OpenSky API may be rate-limited or unavailable

### Debug Steps
1. Check browser console for JavaScript errors
2. Verify API endpoints are accessible
3. Test with management command: `python manage.py test_adsb --demo`
4. Check Django logs for API request errors

### API Limitations
- OpenSky Network API has rate limits (4000/day)
- Some aircraft may not broadcast ADS-B data
- Military aircraft often excluded from public feeds
- Data accuracy depends on receiver coverage

## Future Enhancements

Potential improvements:
- Aircraft trail/path visualization
- Aircraft type and airline information
- Filtering by altitude, speed, or country
- Historical aircraft data playback
- Integration with flight planning tools
- Aircraft weather overlay correlation
- Custom aircraft icons by type
- Sound notifications for nearby aircraft

## Security Considerations

- API requests go through Django backend (no direct client-to-OpenSky calls)
- CSRF protection on all API endpoints
- No sensitive flight data stored locally
- Caching prevents excessive API requests
- User permissions respected for tracking controls

## Performance Notes

- Minimal impact on map performance
- Aircraft markers efficiently updated using MapLibre sources
- Data fetching optimized with bounds checking
- Caching reduces server load
- Demo mode ensures functionality even offline
