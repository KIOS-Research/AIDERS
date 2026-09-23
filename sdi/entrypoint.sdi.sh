#!/bin/bash
echo "-- Running entrypoint.sdi.sh --"

# wait for mysql
while ! nc -z $DB_HOST $DB_PORT; do
    sleep 0.1
done

# run the compiled Go app
./app

echo "-- END entrypoint.sdi.sh --"

# keep the container alive
tail -f /dev/null