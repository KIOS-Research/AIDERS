#!/bin/bash
echo "-- Running entrypoint.ws.sh --"

# wait for mysql
while ! echo exit | telnet $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
	sleep 1
done

# run the compiled Go app
./app

echo "-- END entrypoint.ws.sh --"

# keep the container alive
tail -f /dev/null