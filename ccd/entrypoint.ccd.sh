#!/bin/bash
echo "-- Running entrypoint.ccd.sh --"

# wait for mysql
while ! echo exit | nc -z $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
	sleep 1
done

# run the compiled Go app
./app

echo "-- END entrypoint.ccd.sh --"

# keep the container alive
tail -f /dev/null