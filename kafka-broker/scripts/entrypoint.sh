#!/bin/bash
set -e

# Wait for Kafka to start
echo "Waiting for Kafka to start..."
sleep 10
# while ! nc -z localhost 9092; do
#   echo "Kafka is not ready yet. Retrying in 5 seconds..."
#   sleep 5
# done

# Create topics
echo "Creating topics..."

/opt/kafka/bin/kafka-cluster.sh cluster-id --bootstrap-server :9092

/opt/kafka/bin/kafka-console-producer.sh --bootstrap-server :9092 --topic CrisisClassification

# kafka kafka-topics.sh --create --topic ObjectDetection \
# kafka kafka-topics.sh --create --topic CrisisClassification \

# /usr/bin/kafka-topics --create --topic ObjectDetection --bootstrap-server localhost:9092 --partitions 3 --replication-factor 1
# /usr/bin/kafka-topics --create --topic CrisisClassification --bootstrap-server localhost:9092 --partitions 3 --replication-factor 1
# /usr/bin/kafka-topics --create --topic PathPlanning_Intput --bootstrap-server localhost:9092 --partitions 3 --replication-factor 1
# /usr/bin/kafka-topics --create --topic PathPlanning_Output --bootstrap-server localhost:9092 --partitions 3 --replication-factor 1

# List topics to verify
# echo "Listing topics..."
# kafka kafka-topics.sh --list --bootstrap-server localhost:9092
# /usr/bin/kafka-topics --list --bootstrap-server localhost:9092

# Additional configurations (if needed)
echo "Kafka setup complete."
exec "$@"

# keep the container alive
tail -f /dev/null