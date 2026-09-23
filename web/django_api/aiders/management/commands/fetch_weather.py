import time
from django.core.management.base import BaseCommand
from django.utils import timezone
from aiders.models import WeatherConfig
from aiders.views import fetch_weather_data


class Command(BaseCommand):
    help = 'Periodically fetch weather data from WeatherAPI.com'

    def add_arguments(self, parser):
        parser.add_argument(
            '--interval',
            type=int,
            default=None,
            help='Override update interval in minutes (default: use config setting)'
        )
        parser.add_argument(
            '--once',
            action='store_true',
            help='Run once and exit (don\'t run continuously)'
        )

    def handle(self, *args, **options):
        self.stdout.write(
            self.style.SUCCESS('Starting weather data fetching service...')
        )

        if options['once']:
            self.stdout.write('Running weather fetch once...')
            fetch_weather_data()
            self.stdout.write(
                self.style.SUCCESS('Weather data fetch completed.')
            )
            return

        while True:
            try:
                # Get active configuration
                config = WeatherConfig.objects.filter(is_active=True).first()
                
                if not config:
                    self.stdout.write(
                        self.style.WARNING('No active weather configuration found. Waiting 5 minutes...')
                    )
                    time.sleep(300)  # Wait 5 minutes
                    continue

                # Use command line interval override or config interval
                interval_minutes = options['interval'] or config.update_interval
                
                self.stdout.write(f'Fetching weather data at {timezone.now()}')
                fetch_weather_data()
                self.stdout.write(
                    self.style.SUCCESS(f'Weather data fetched. Next update in {interval_minutes} minutes.')
                )
                
                # Wait for the specified interval
                time.sleep(interval_minutes * 60)
                
            except KeyboardInterrupt:
                self.stdout.write(
                    self.style.SUCCESS('Weather fetching service stopped by user.')
                )
                break
            except Exception as e:
                self.stdout.write(
                    self.style.ERROR(f'Error in weather fetching service: {e}')
                )
                # Wait 5 minutes before retrying on error
                time.sleep(300)
