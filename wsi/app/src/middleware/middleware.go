package middleware

import (
	"context"
	"encoding/json"
	"log"
	"mdemet/wsi/src/db"
	"os"
	"time"

	"github.com/segmentio/kafka-go"
)

// Kafka producer variables
var KafkaTelemetryWriter *kafka.Writer

// InitKafka initializes Kafka producers
func InitKafkaProducer() {
	// Check if Kafka is active
	if os.Getenv("KAFKA_ACTIVE") != "1" {
		log.Println("🔕 Kafka is disabled. No Kafka connection will be established.")
		return
	}

	// if kafka is local, get KAFKA_LOCAL_IP instead of KAFKA_IP
	kafkaIp := os.Getenv("KAFKA_IP")
	if os.Getenv("KAFKA_LOCAL") == "1" {
		kafkaIp = os.Getenv("KAFKA_LOCAL_IP")
	}

	kafkaHost := kafkaIp
	kafkaPort := os.Getenv("KAFKA_PORT")
	kafkaBrokerAddr := kafkaHost + ":" + kafkaPort
	kafkaTelemetryTopic := os.Getenv("KAFKA_TELEMETRY_TOPIC")

	log.Println("Checking environment variables:")
	log.Printf("KAFKA_IP: %s", kafkaIp)
	log.Printf("KAFKA_PORT: %s", os.Getenv("KAFKA_PORT"))
	log.Printf("KAFKA_TELEMETRY_TOPIC: %s", os.Getenv("KAFKA_TELEMETRY_TOPIC"))

	if kafkaHost == "" || kafkaPort == "" || kafkaTelemetryTopic == "" {
		log.Fatal("❌ Missing Kafka environment variables!")
	}

	log.Printf("Kafka Broker Address: %s", kafkaBrokerAddr)
	KafkaTelemetryWriter = newKafkaWriter(kafkaTelemetryTopic, kafkaBrokerAddr)
	log.Println("✅ Kafka producers initialized.")
}

// Create a Kafka writer
func newKafkaWriter(topic string, kafkaBrokerAddr string) *kafka.Writer {
	return &kafka.Writer{
		Addr:         kafka.TCP(kafkaBrokerAddr),
		Topic:        topic,
		Balancer:     &kafka.LeastBytes{},
		BatchSize:    1,                     // Send messages immediately (disable batching)
		BatchTimeout: 10 * time.Millisecond, // Reduce max wait time
	}
}

// Struct for Kafka messages
type KafkaMessage struct {
	DroneName      string                   `json:"drone_name"`
	DroneID        int64                    `json:"drone_id"`
	Timestamp      float64                  `json:"timestamp"`
	DroneTelemetry db.DroneTelemetryMessage `json:"telemetry"`
	// ConnectionDuration int                      `json:"connection_duration"`
	// MissionLogID       int                      `json:"mission_log_id"`
	// OperationID        int                      `json:"operation_id"`
	// FovPolygon         [][2]float64             `json:"fov_polygon"`
}

// Send Telemetry to Kafka
func SendTelemetryToKafka(droneName string, droneId int64, message []byte, droneMsg db.DroneTelemetryMessage) {
	// Check if Kafka is active
	if os.Getenv("KAFKA_ACTIVE") != "1" {
		// log.Printf("🔕 Kafka is disabled. Skipping telemetry send for DroneID %d.", droneId)
		return
	}

	// Check if Kafka writer is initialized
	if KafkaTelemetryWriter == nil {
		log.Printf("❌ Kafka writer not initialized. Skipping telemetry send for DroneID %d.", droneId)
		return
	}

	msg := KafkaMessage{
		DroneName:      droneName,
		DroneID:        droneId,
		Timestamp:      float64(time.Now().UnixMilli()) / 1000.0,
		DroneTelemetry: droneMsg,
		// ConnectionDuration: int(connectionDuration), // convert float64 to int
		// MissionLogID:       missionLogId,
		// OperationID:        operationId,
		// FovPolygon:         fovPolygon,
	}
	msgBytes, _ := json.Marshal(msg)

	for i := 0; i < 5; i++ { // Retry up to 5 times
		err := KafkaTelemetryWriter.WriteMessages(context.Background(), kafka.Message{
			Key:   []byte(droneName),
			Value: msgBytes,
		})
		if err == nil {
			// log.Printf("Kafka message sent to topic %s for DroneID %d.", KafkaTelemetryWriter.Topic, droneId)
			return
		}
		log.Printf("Retry %d: Failed to send Kafka message: %v", i+1, err)
		// time.Sleep(3 * time.Second)
	}
	log.Printf("Failed to send Kafka message after retries.")
}
