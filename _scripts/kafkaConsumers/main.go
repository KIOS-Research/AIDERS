package main

import (
	"encoding/json"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"

	"github.com/confluentinc/confluent-kafka-go/kafka"
)

// Function to consume messages from a specific topic
func consumeTopic(topicName string, kafkaConfig *kafka.ConfigMap, messageCounter *sync.Map, wg *sync.WaitGroup) {
	defer wg.Done()
	log.Printf("Starting consumer for topic: %s", topicName)
	consumer, err := kafka.NewConsumer(kafkaConfig)
	if err != nil {
		log.Fatalf("Failed to create consumer: %s", err)
	}
	defer consumer.Close()

	err = consumer.SubscribeTopics([]string{topicName}, nil)
	if err != nil {
		log.Fatalf("Failed to subscribe to topic %s: %s", topicName, err)
	}

	messageChannel := make(chan *kafka.Message, 1000) // Buffered channel

	// Goroutine for processing messages
	go func() {
		for msg := range messageChannel {
			var msgValue map[string]interface{}
			if err := json.Unmarshal(msg.Value, &msgValue); err != nil {
				log.Printf("Error decoding message value: %s", err)
				continue
			}

			// Increment message counter
			counter, _ := messageCounter.LoadOrStore(topicName, 0)
			messageCounter.Store(topicName, counter.(int)+1)

			log.Printf("Processed message %s #%d -- %v", topicName, counter.(int)+1, msgValue["msg_id"])
		}
	}()

	for {
		msg, err := consumer.ReadMessage(1000) // Timeout set to 1000ms
		if err != nil {
			if kafkaErr, ok := err.(kafka.Error); ok && kafkaErr.Code() == kafka.ErrTimedOut {
				continue
			}
			log.Printf("Error consuming message: %s", err)
			break
		}
		messageChannel <- msg // Send message to the channel
	}
}

func main() {
	const kafkaServer = "apps.edutel.uniwa.gr:9092"

	// Kafka configuration for high-frequency topic
	// highFrequencyConfig := &kafka.ConfigMap{
	// 	"bootstrap.servers": kafkaServer,
	// 	"group.id":          "high-frequency-consumer-group2",
	// 	"auto.offset.reset": "earliest",
	// 	"enable.auto.commit": true, // Enable auto-commit
	// }

	// Kafka configuration for low-frequency topic
	lowFrequencyConfig := &kafka.ConfigMap{
		"bootstrap.servers":  kafkaServer,
		"group.id":           "low-frequency-consumer-group2",
		"auto.offset.reset":  "earliest",
		"enable.auto.commit": true, // Enable auto-commit
	}

	// Define the topics
	// highFrequencyTopic := "ObjectDetection"
	lowFrequencyTopic := "PathPlanning_Output"

	// Initialize counters for each topic
	messageCounter := &sync.Map{}

	// WaitGroup to manage Goroutines
	var wg sync.WaitGroup

	// Create Goroutines for consuming each topic
	wg.Add(2)
	// go consumeTopic(highFrequencyTopic, highFrequencyConfig, messageCounter, &wg)
	go consumeTopic(lowFrequencyTopic, lowFrequencyConfig, messageCounter, &wg)

	// Handle graceful shutdown
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	<-sigChan
	log.Println("Shutting down consumers...")
	wg.Wait()
	log.Println("Consumers stopped.")
}
