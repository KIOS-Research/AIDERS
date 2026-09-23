import threading
import json
from datetime import datetime
from confluent_kafka import Consumer, KafkaException

def consume_topic(topic_name, kafka_config, message_counter):
    """Function to consume messages from a specific topic."""
    consumer = Consumer(kafka_config)
    consumer.subscribe([topic_name])

    try:
        while True:
            msg = consumer.poll(timeout=1.0)
            if msg is None:
                continue
            if msg.error():
                if msg.error().code() == KafkaException._PARTITION_EOF:
                    continue
                else:
                    print(f"Error: {msg.error()}")
                    break

            # Retrieve the timestamp of the message
            timestamp_type, timestamp = msg.timestamp()
            try:
                timestamp = datetime.fromtimestamp(timestamp / 1000).strftime('%Y-%m-%d %H:%M:%S')
            except (ValueError, TypeError):
                timestamp = "Invalid timestamp"
            # if timestamp_type == 0:  # CREATE_TIME
            #     timestamp_str = f"Published at: {timestamp} ms since epoch"
            # elif timestamp_type == 1:  # LOG_APPEND_TIME
            #     timestamp_str = f"Log appended at: {timestamp} ms since epoch"
            # else:
            #     timestamp_str = "Timestamp not available"

            try:
                msg_value = msg.value().decode('utf-8') if msg.value() else "No value"
                # convert string to JSON if needed
                msg_value = json.loads(msg_value) if msg_value else {}
                print(f"[{timestamp}] Received message from {topic_name}: {msg_value}")
                # message_counter[topic_name] += 1
                # print(f"[{timestamp}] Received message {topic_name} #{message_counter[topic_name]} -- {msg_value['msg_id']}")
            except Exception as e:
                print(f"Error decoding message value: {e}")
                # msg_value = "Decoding error"
            # print(f"Received message from {topic_name}: {msg.value().decode('utf-8')}")
    finally:
        consumer.close()

def main():
    KAFKA_SERVER = 'apps.edutel.uniwa.gr:9092'

    # Kafka configuration for high-frequency topic
    high_frequency_config = {
        'bootstrap.servers': KAFKA_SERVER,
        'group.id': 'high-frequency-consumer-group2',
        'auto.offset.reset': 'latest'
    }

    # Kafka configuration for low-frequency topic
    low_frequency_config = {
        'bootstrap.servers': KAFKA_SERVER,
        'group.id': 'low-frequency-consumer-group2',
        'auto.offset.reset': 'latest'
    }


    # Define the topics
    high_frequency_topic = 'UAV_Telemetry'
    low_frequency_topic = 'PathPlanning_Output'

    # Initialize counters for each topic
    message_counter = {
        high_frequency_topic: 0,
        low_frequency_topic: 0
    }


    # Create threads for consuming each topic
    high_freq_thread = threading.Thread(target=consume_topic, args=(high_frequency_topic, high_frequency_config, message_counter))
    low_freq_thread = threading.Thread(target=consume_topic, args=(low_frequency_topic, low_frequency_config, message_counter))

    # Start the threads
    high_freq_thread.start()
    low_freq_thread.start()

    # Wait for threads to finish (optional, depending on your use case)
    high_freq_thread.join()
    low_freq_thread.join()

if __name__ == "__main__":
    main()