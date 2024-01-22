#!/bin/bash
echo "-- Running entrypoint.alg.sh --"

# wait for mysql
while ! nc -z $SQL_HOST $SQL_PORT; do
    sleep 0.1
done

# run the python application
echo -e "\n** Running Python main app **"
python -u /app/main.py &

# start the HTTP server
# echo -e "\n** Starting the HTTP server **"
# cd app && gunicorn -w 5 -b 0.0.0.0:8765 server:app &


echo "-- END entrypoint.alg.sh --"

# keep the container alive
tail -f /dev/null