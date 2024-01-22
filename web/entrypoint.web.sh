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
while ! nc -z $SQL_HOST $SQL_PORT; do
	sleep 0.1
done
cd app
echo "Database is up and running!"
echo "Initializing Database..."
python3 manage.py makemigrations aiders --noinput
echo "Creating Database tables..."
python3 manage.py migrate
echo "Checking if the admin account exists..."
ensure_superuser
echo "Reset Database Clients..."
python3 manage.py resetDatabaseClients

# run the python application
echo -e "\n** Running Python main app **"

if [ "$DEBUG" = 1 ]; then
	echo -e "\n** Running in debug mode! **"
  python manage.py customFeatures &
	python manage.py runserver $NET_IP:$WEB_PORT &
else
	echo -e "\n** Running in production mode! **"
	python manage.py collectstatic --noinput
  python manage.py customFeatures &
	gunicorn django_api.wsgi:application --bind 0.0.0.0:$WEB_PORT &
	# gunicorn django_api.wsgi:application --bind $NET_IP:$WEB_PORT &&
fi

echo "Platform started!"

sleep 2

echo "-- END entrypoint.web.sh --"

# keep the container alive
tail -f /dev/null
