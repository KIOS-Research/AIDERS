#!/bin/bash
echo "-- Running entrypoint.web.sh --"

# Check if in the database there is already a superuser.
ensure_superuser() {
	superUserExists=$(python manage.py ensureAdminUser --superUserExists)

	if [ "$superUserExists" = True ]; then
		echo "A Superuser already exists in the Database!"
	else
		echo "No Superuser was found in the Database!"
		python manage.py ensureAdminUser --username="$ADMIN_USER" --email="$ADMIN_USER@admin.com" --password="$ADMIN_PASSWORD" --firstname="Admin" --lastname="User" --createSuperuser
	fi
}

echo "Checking if the Database is up..."
while ! echo exit | nc -z $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
	sleep 1
done

cd app
echo "Database is up and running!"
echo "Initializing Database..."
python3 manage.py makemigrations aiders --noinput
echo "Creating Database tables..."
python3 manage.py migrate
echo "Checking if the admin account exists..."
# ensure_superuser
echo "Reset Database Clients..."
python3 manage.py resetDatabaseClients

# run the python application
echo -e "\n** Running Python main app **"

if [ "$DEBUG" = 1 ]; then
	echo -e "\n** Running in debug mode! **"
  	python manage.py customFeatures &
	python manage.py runserver 0.0.0.0:$WEB_PORT &
else
	echo -e "\n** Running in production mode! **"
	python manage.py collectstatic --noinput
  	python manage.py customFeatures &
	gunicorn django_api.wsgi:application --bind 0.0.0.0:$WEB_PORT --timeout 600 --workers 4 --worker-class sync &
	# gunicorn django_api.wsgi:application --bind $NET_IP:$WEB_PORT &&
fi

# Gunicorn is WSGI only, so django's websockets (/ws/chat/) need an ASGI server
# of their own. Keep it to a single process: CHANNEL_LAYERS uses the in-memory
# backend, which is not shared between processes.
# It also runs in debug mode so that nginx routes websockets the same way in
# both modes. Note that daphne does not autoreload, so consumer changes need a
# container restart.
echo -e "\n** Starting daphne (websockets) on port $WEB_WS_PORT **"
daphne -b 0.0.0.0 -p $WEB_WS_PORT django_api.asgi:application &

echo "Platform started!"

# Start weather fetching service in background if enabled
if [ "$ENABLE_WEATHER_FETCH" = 1 ]; then
	echo "Starting weather data fetching service..."
	python manage.py fetch_weather &
	WEATHER_PID=$!
	echo "Weather service started with PID: $WEATHER_PID"
else
	echo "Weather fetching service is disabled. Set ENABLE_WEATHER_FETCH=1 to enable."
fi

sleep 2

echo "-- END entrypoint.web.sh --"

# keep the container alive
tail -f /dev/null
