#!/bin/bash
echo "-- Running entrypoint.ws.sh --"

# wait for mysql
while ! nc -z $SQL_HOST $SQL_PORT; do
    sleep 0.1
done

# run the compiled Go app
./app

echo "-- END entrypoint.ws.sh --"

# keep the container alive
tail -f /dev/null