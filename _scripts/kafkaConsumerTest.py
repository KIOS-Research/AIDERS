from kafka import KafkaConsumer
import json

counter = 0
def main():
    # Kafka configuration
    # topic = 'PathPlanning_Output'  # Topic to subscribe to
    topic = 'ObjectDetection'  # Topic to subscribe to
    bootstrap_servers = 'apps.edutel.uniwa.gr:9092'  # Kafka broker address
    # bootstrap_servers = '192.168.0.10:9092'  # Kafka broker address

    # Create a Kafka consumer
    consumer = KafkaConsumer(
        topic,
        bootstrap_servers=bootstrap_servers,
        group_id='pathplanning_group4',  # Consumer group ID
        auto_offset_reset='latest',   # Start reading at the earliest message
        enable_auto_commit=True,         # Automatically commit offsets
        value_deserializer=lambda x: json.loads(x.decode('utf-8')), 
    )

    print(f"Subscribed to topic: {topic}")

    try:
        for message in consumer:
            global counter
            counter += 1
            # print(f"Received message: {message.value.decode('utf-8')}")
            # print(f"Received messages: {counter}")

            for record in message.value['records']:
                value  = record["value"]
                header = value["header"]
                msg_identifier = header["msgIdentifier"]
                print(f"OD MSG ID: {msg_identifier}")            

    except KeyboardInterrupt:
        print("Consumer interrupted by user")

    finally:
        consumer.close()

if __name__ == "__main__":
    main()