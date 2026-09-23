from confluent_kafka.admin import AdminClient, NewTopic

def create_kafka_topic(broker, topic_name, num_partitions=1, replication_factor=1):
    # Create an AdminClient instance
    admin_client = AdminClient({
        "bootstrap.servers": broker
    })

    # Define the topic configuration
    topic = NewTopic(topic_name, num_partitions=num_partitions, replication_factor=replication_factor)

    # Attempt to create the topic
    try:
        futures = admin_client.create_topics([topic])
        for topic, future in futures.items():
            try:
                future.result()  # Wait for the topic creation to complete
                print(f"Topic '{topic}' created successfully.")
            except Exception as e:
                print(f"Failed to create topic '{topic}': {e}")
    except Exception as e:
        print(f"Error while creating topic: {e}")

# Example usage
if __name__ == "__main__":
    broker_address = "localhost:9092"  # Kafka broker address
    create_kafka_topic(broker_address, "UAV_Telemetry")
    create_kafka_topic(broker_address, "ObjectDetection")
    create_kafka_topic(broker_address, "CrisisClassification")
    create_kafka_topic(broker_address, "PathPlanning_Input")
    create_kafka_topic(broker_address, "PathPlanning_Output")
    create_kafka_topic(broker_address, "PathPlanning_Mission_Output")
