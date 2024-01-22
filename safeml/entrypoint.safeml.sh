#!/bin/bash
echo "-- Running entrypoint.safeml.sh --"

cd /deepKnowledgeSetup/

python3 setup.py install

cd ../..

# wait for mysql
while ! nc -z $SQL_HOST $SQL_PORT; do
    sleep 0.1
done

# run the python application
echo -e "\n** Running Python main app **"
python3 -u /app/main.py &

# sleep 2

echo "-- END entrypoint.safeml.sh --"

# keep the container alive
tail -f /dev/null