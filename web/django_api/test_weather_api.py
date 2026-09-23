#!/usr/bin/env python3
"""
Test script for WeatherAPI.com integration
Run this to test if the API integration is working correctly
"""

import requests
import json
from datetime import datetime

def test_weather_api(api_key, city="London"):
    """Test WeatherAPI.com API call"""
    try:
        url = "http://api.weatherapi.com/v1/current.json"
        params = {
            'key': api_key,
            'q': city,
            'aqi': 'no'
        }
        
        print(f"Testing WeatherAPI.com with city: {city}")
        print(f"URL: {url}")
        print(f"Parameters: {params}")
        
        response = requests.get(url, params=params, timeout=10)
        response.raise_for_status()
        
        data = response.json()
        
        print(f"\n✅ API call successful!")
        print(f"📍 Location: {data['location']['name']}, {data['location']['region']}, {data['location']['country']}")
        print(f"🌡️  Temperature: {data['current']['temp_c']}°C (feels like {data['current']['feelslike_c']}°C)")
        print(f"☁️  Condition: {data['current']['condition']['text']}")
        print(f"💨 Wind: {data['current']['wind_kph']} kph {data['current']['wind_dir']}")
        print(f"💧 Humidity: {data['current']['humidity']}%")
        print(f"📊 Pressure: {data['current']['pressure_mb']} mb")
        print(f"👁️  Visibility: {data['current']['vis_km']} km")
        print(f"☀️  UV Index: {data['current']['uv']}")
        print(f"☁️  Cloud Cover: {data['current']['cloud']}%")
        print(f"📅 Last Updated: {data['current']['last_updated']}")
        
        return True
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Network error: {e}")
        return False
    except KeyError as e:
        print(f"❌ API response format error. Missing key: {e}")
        print(f"Full response: {json.dumps(data, indent=2)}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

def main():
    print("🌤️  WeatherAPI.com Integration Test")
    print("=" * 40)
    
    # You need to replace this with your actual API key
    api_key = input("Enter your WeatherAPI.com API key: ").strip()
    
    if not api_key:
        print("❌ API key is required!")
        return
    
    # Test with a few cities
    test_cities = ["London", "New York", "Paris", "Tokyo"]
    
    for city in test_cities:
        print(f"\n{'='*50}")
        success = test_weather_api(api_key, city)
        if not success:
            print(f"❌ Test failed for {city}")
            break
    else:
        print(f"\n{'='*50}")
        print("🎉 All tests passed! WeatherAPI.com integration is working correctly.")
        print("\nNext steps:")
        print("1. Add this API key to your Django weather configuration")
        print("2. Add the cities you want to monitor")
        print("3. Enable automatic updates")
        print("4. Run the management command: python manage.py fetch_weather --once")

if __name__ == "__main__":
    main()
