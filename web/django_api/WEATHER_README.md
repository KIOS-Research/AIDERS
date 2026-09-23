# Weather API Integration with WeatherAPI.com

This Django application includes weather data collection functionality using the WeatherAPI.com API.

## Features

- 🌤️ Fetch current weather conditions for multiple cities
- 📊 Store weather data in database with detailed information
- ⚙️ Web interface for API configuration
- 🔄 Automatic periodic updates
- 📱 Responsive weather data display
- 🛠️ Management command for manual/automated fetching

## Setup

### 1. Get WeatherAPI.com API Key

1. Visit [WeatherAPI.com](https://www.weatherapi.com)
2. Sign up for a free account
3. Get your API key from the dashboard
4. Free accounts include 1 million API calls per month

### 2. Configure in Django Admin

1. Navigate to the Weather menu in your Django application
2. Click on "Weather Configuration"
3. Enter your API key
4. Add cities (comma-separated, e.g., "London, New York, Paris")
5. Set update interval (in minutes)
6. Enable automatic updates

### 3. Test the Integration

Run the test script to verify your API key works:

```bash
cd /path/to/your/django_api
python test_weather_api.py
```

### 4. Manual Weather Data Fetch

You can manually fetch weather data using the management command:

```bash
# Fetch once and exit
python manage.py fetch_weather --once

# Run continuously (for production)
python manage.py fetch_weather

# Override update interval
python manage.py fetch_weather --interval 30
```

### 5. Automatic Container Startup

The weather fetching service is automatically started when your Django container starts. You can control this with the `ENABLE_WEATHER_FETCH` environment variable:

```bash
# In your .env file or docker-compose.yml
ENABLE_WEATHER_FETCH=1  # Enable automatic weather fetching (default)
ENABLE_WEATHER_FETCH=0 # Disable automatic weather fetching
```

### Container Startup Sequence
When your Django container starts, here's what happens:

1. **Database Setup**: Migrations run, admin user is ensured
2. **Django Starts**: Either debug mode (`runserver`) or production mode (`gunicorn`)
3. **Custom Features Start**: Your existing custom features start
4. **Weather Service Starts**: If `ENABLE_WEATHER_FETCH=true` (default), the weather service starts in background
5. **Container Stays Alive**: The container continues running with all services

The weather service runs independently and will continue fetching weather data according to your configuration.

## Database Models

### WeatherAPI
Stores weather data with the following fields:
- `city`: City name
- `country`: Country name
- `region`: State/region
- `temperature`: Temperature in Celsius
- `feels_like`: Feels like temperature
- `humidity`: Humidity percentage
- `pressure`: Pressure in millibars
- `wind_speed`: Wind speed in kph
- `wind_direction`: Wind direction in degrees
- `wind_dir_text`: Wind direction text (N, NE, etc.)
- `weather_condition`: Weather condition description
- `weather_icon`: Weather icon URL
- `visibility`: Visibility in km
- `uv_index`: UV index
- `cloud_cover`: Cloud cover percentage
- `gust_kph`: Wind gust speed in kph
- `timestamp`: When data was stored
- `api_timestamp`: When data was recorded by API

### WeatherConfig
Stores API configuration:
- `api_key`: WeatherAPI.com API key
- `cities`: Comma-separated list of cities
- `update_interval`: Update interval in minutes
- `is_active`: Enable/disable automatic updates

## API Endpoints

- `/weather/config/` - Weather configuration page
- `/weather/data/` - Weather data display page
- `/weather/update/` - Manual weather update trigger

## Production Deployment

### Docker Container Automatic Startup
Your weather fetching service is configured to start automatically when your Django container starts. The service runs in the background using the simple approach.

**How it works:**
1. When your container starts, the `entrypoint.web.sh` script runs
2. After Django starts, the weather service starts in the background
3. The service will continuously fetch weather data based on your configuration
4. If the service crashes, you can restart it manually or restart the container

### Environment Variable Control
You can control the weather service with the `ENABLE_WEATHER_FETCH` environment variable:

```bash
# In your .env file
ENABLE_WEATHER_FETCH=true   # Enable automatic weather fetching (default)
ENABLE_WEATHER_FETCH=false  # Disable automatic weather fetching
```

### Monitoring and Management
Use the provided monitoring script to manage the weather service:

```bash
# Check if weather service is running
./weather_monitor.sh status

# View weather service logs
./weather_monitor.sh logs

# Restart weather service
./weather_monitor.sh restart

# Test weather API
./weather_monitor.sh test

# View current configuration
./weather_monitor.sh config
```

## WeatherAPI.com Data Format

The API returns comprehensive weather data:
- Current conditions
- Temperature (actual and feels like)
- Wind speed, direction, and gusts
- Humidity and pressure
- Visibility and cloud cover
- UV index
- Weather condition with icon

## Troubleshooting

### Common Issues

1. **API Key Invalid**: Verify your API key is correct and active
2. **City Not Found**: Check city name spelling, try "City, Country" format
3. **Rate Limit**: Free accounts have 1M calls/month limit
4. **Network Issues**: Check internet connectivity and firewall settings

### Debugging

Enable Django debug mode and check the console output when running:
```bash
python manage.py fetch_weather --once
```

The function will print detailed error messages for troubleshooting.

## License

This weather integration is part of the AIDERS platform.
