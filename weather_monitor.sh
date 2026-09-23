#!/bin/bash
# Weather Service Monitor Script
# This script helps you monitor and control the weather fetching service

CONTAINER_NAME="web"

show_help() {
    echo "Weather Service Monitor"
    echo "======================"
    echo
    echo "Usage: $0 [COMMAND]"
    echo
    echo "Commands:"
    echo "  status    - Show weather service status"
    echo "  logs      - Show weather service logs"
    echo "  restart   - Restart weather service"
    echo "  test      - Test weather API with sample city"
    echo "  config    - Show current weather configuration"
    echo "  enable    - Enable automatic weather fetching"
    echo "  disable   - Disable automatic weather fetching"
    echo "  help      - Show this help message"
    echo
}

check_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "❌ Container '$CONTAINER_NAME' is not running"
        exit 1
    fi
}

case "$1" in
    "status")
        check_container
        echo "🔍 Checking weather service status..."
        docker exec $CONTAINER_NAME ps aux | grep "fetch_weather" | grep -v grep || echo "❌ Weather service not found"
        ;;
    
    "logs")
        check_container
        echo "📋 Weather service logs:"
        echo "Checking container logs for weather-related output..."
        docker logs $CONTAINER_NAME 2>&1 | grep -i weather | tail -20 || echo "No weather logs found in container output"
        ;;
    
    "restart")
        check_container
        echo "🔄 Restarting weather service..."
        docker exec $CONTAINER_NAME pkill -f "fetch_weather" 2>/dev/null || true
        sleep 2
        docker exec $CONTAINER_NAME bash -c "cd /app && python manage.py fetch_weather &"
        echo "✅ Weather service restarted"
        ;;
    
    "test")
        check_container
        echo "🧪 Testing weather API..."
        docker exec $CONTAINER_NAME python /app/manage.py fetch_weather --once
        ;;
    
    "config")
        check_container
        echo "⚙️  Current weather configuration:"
        docker exec $CONTAINER_NAME python /app/manage.py shell -c "
from aiders.models import WeatherConfig
config = WeatherConfig.objects.first()
if config:
    print(f'API Key: {config.api_key[:10]}...')
    print(f'Cities: {config.cities}')
    print(f'Update Interval: {config.update_interval} minutes')
    print(f'Active: {config.is_active}')
else:
    print('No configuration found. Please set up via web interface.')
"
        ;;
    
    "enable")
        echo "✅ Enabling weather fetching service..."
        echo "Add ENABLE_WEATHER_FETCH=true to your .env file and restart the container"
        ;;
    
    "disable")
        check_container
        echo "⏹️  Disabling weather fetching service..."
        docker exec $CONTAINER_NAME pkill -f "fetch_weather" 2>/dev/null || true
        echo "Weather service stopped. Add ENABLE_WEATHER_FETCH=false to your .env file to disable on restart"
        ;;
    
    "help"|""|*)
        show_help
        ;;
esac
