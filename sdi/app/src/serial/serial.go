package serial

import (
	"bufio"
	"fmt"
	"log"
	"strings"
	"time"

	"github.com/tarm/serial"
)

var SerialPort *serial.Port
var SerialReader *bufio.Reader

func OpenSerialPort(port string, baud int) error {
	c := &serial.Config{Name: port, Baud: baud}
	s, err := serial.OpenPort(c)
	if err != nil {
		return err
	}
	SerialPort = s
	SerialReader = bufio.NewReader(SerialPort) // Initialize SerialReader
	log.Println("Lora Serial Connected!")
	return nil
}

func Reconnect(port string, baud int) error {
	SerialPort.Close()
	time.Sleep(5 * time.Second)
	return OpenSerialPort(port, baud)
}

func SendMessage(message string) error {
	_, err := SerialPort.Write([]byte(message))
	if err != nil {
		log.Println("Error writing to serial port: ", err)
		return err
	}
	// NOTE: For DEBUG purposes
	// log.Printf("Sent: %s", message)
	return nil
}

// func GetMessage(buf []byte) (string, error) {
// 	n, err := SerialPort.Read(buf)
// 	if err != nil {
// 		return "", err
// 	}
// 	return string(buf[:n]), nil
// }

func GetMessage() (string, error) {
	message, err := SerialReader.ReadString('\n')
	fmt.Println("Message: ", message)
	if err != nil {
		return "", err
	}
	return message, nil
}

func ProcessMessage(incomingMessage string) {
	// Lora Communication messages
	// Lora To Platform
	// ConnectLora,MasterLoraName,Connect
	// TelemetryLora,LoraName,Latitude,Longitude,PM1,PM25,AccelerationX,AccelerationY,AccelerationZ,ReceivedSignalStrengthIndication
	// MonitorLora,LoraName,BatteryPercentage,CpuUsage,HeapMemory,ReceivedSignalStrengthIndication
	// Platform To Lora
	// HandshakeLora,Connect
	// Note Connect Should be True or False
	trimmedMessage := strings.TrimSpace(incomingMessage)
	messageParts := strings.Split(trimmedMessage, ",")

	log.Println("**************** Processing message:", trimmedMessage) // Debug log to see message parts

	switch messageParts[0] {
	case "ConnectLora":
		log.Println("+++++++++++++ Connecting Lora Master:", messageParts[1])
		if len(messageParts) == 3 {
			handleLoraMasterConnectionStatus(messageParts)
		} else {
			log.Printf("Invalid message length for ConnectLora: %d", len(messageParts))
		}
	// case "TelemetryLora":
	// 	if len(messageParts) == 10 {
	// 		fmt.Println("NEW MSG: ", messageParts)
	// 		handleTelemetryLora(messageParts)
	// 	} else {
	// 		log.Printf("Invalid message length for TelemetryLora: %d", len(messageParts))
	// 	}
	case "TelemetryLora":
		if len(messageParts) == 14 {
			handleTelemetryLora(messageParts)
		} else {
			log.Printf("Invalid message length for TelemetryLora: %d", len(messageParts))
		}
	case "MonitorLora":
		if len(messageParts) == 6 {
			handleMonitorLora(messageParts)
		} else {
			log.Printf("Invalid message length for MonitorLora: %d", len(messageParts))
		}
	default:
		log.Printf("Unknown message type: %s", messageParts[0])
	}
	// NOTE: For DEBUG purposes
	log.Printf("Received: %s", incomingMessage)
}

func Run(port string, baud int) {
	for {
		err := OpenSerialPort(port, baud)
		if err != nil {
			log.Println("Error opening serial port: ", err)
			log.Println("Attempting to reconnect...")
			time.Sleep(5 * time.Second) // wait for 5 seconds before trying to reconnect
			continue
		}

		// buf := make([]byte, 128)

		for {
			message, err := GetMessage()
			if err != nil {
				log.Println("Error reading from serial port: ", err)
				log.Println("Attempting to reconnect...")
				err = Reconnect(port, baud)
				if err != nil {
					log.Println("Failed to reconnect: ", err)
				}
				continue
			}
			ProcessMessage(message)
		}
	}
}
