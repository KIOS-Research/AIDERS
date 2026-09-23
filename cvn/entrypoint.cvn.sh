#!/bin/bash
echo "-- Running entrypoint.cv.sh --"

# wait for mysql
while ! echo exit | nc -z $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
	sleep 1
done

# run the python application
echo -e "\n** Running Python main app **"
python -u /app/main.py &

# start the HTTP server
# echo -e "\n** Starting the HTTP server **"
# cd app && gunicorn -w 5 -b 0.0.0.0:8765 server:app &


echo "-- END entrypoint.cv.sh --"

# keep the container alive
tail -f /dev/null