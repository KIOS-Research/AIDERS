#!/usr/bin/env bash
pwd=`pwd`

echo ""
echo "Deleting the database..."
echo "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾"
python3 manage.py reset_db --noinput

echo ""
echo "Deleting Migrations..."
echo "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾"
sudo rm -r "${pwd}/aiders/migrations"

echo ""
echo "Creating Migrations..."
echo "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾"
python3 "${pwd}/manage.py" makemigrations aiders

echo ""
echo "Executing Migrations..."
echo "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾"
python3 "${pwd}/manage.py" migrate

echo ""
echo "Deleting Temp Files..."
echo "‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾"
sudo rm -rf "${pwd}/aiders/media/*"
sudo rm -rf "${pwd}/results_TM/*"
# find . | grep -E "(__pycache__|\.pyc|\.pyo$)" | xargs rm -rf

#username='g'
#pass='g'
#echo "====== Creating Super User with username: ${username} and pass: ${pass} ====== "
#echo "from django.contrib.auth import get_user_model; get_user_model().objects.create_superuser('${username}', 'admin@example.com', '${pass}')" | python3 manage.py shell

#python3 "${pwd}/manage.py" runserver "$NET_IP:$DJANGO_PORT"


