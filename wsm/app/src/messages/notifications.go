package messages

import (
	"encoding/json"
	"log"
	"fmt"
)

// Incoming POST request body structure for receiving message from client and sending notification to client

type IncomingNotification struct {
	NotificationMsg string `json:"message"`
	NotificationSender string `json:"sender"`

}

// // Handle incoming POST request to receive notification from client
// func ReceiveNotification(w http.ResponseWriter, r *http.Request){
// 	var incomingNotification IncomingNotification 
// 	err := json.NewDecoder(r.Body).Decode(&incomingNotification)
// 	if err != nil {
// 		http.Error(w, err.Error(), http.StatusBadRequest)
// 		return
// 	}
// 	log.Println(("*********************************"))
// 	log.Printf("%c Receive notification from: %s", '\U000027A1', incomingNotification.NotificationMsg)
// 	log.Printf("%c Incoming message: %s", '\U000027A1', incomingNotification.NotificationSender)
// }

func GenerateNotification(_msg json.RawMessage) []byte{
	
	var incomingNotification IncomingNotification 
	err := json.Unmarshal([]byte(_msg), &incomingNotification)
	if err != nil {
		fmt.Println("Error unmarshalling _msg:", err)
		return nil
	}

	fmt.Println("Message: %s", incomingNotification.NotificationMsg)
	fmt.Println("Sender: %s", incomingNotification.NotificationSender)
	// build outgoing message
	outgoingMsg := map[string]interface{}{
		"type": "notification",
		"info": map[string]interface{}{
			"sender": incomingNotification.NotificationSender,
			"message": incomingNotification.NotificationMsg,
		},
	}

	jsonMessage, err := json.Marshal(outgoingMsg)
	if err != nil {
		log.Printf("Error marshalling message: %v", err)
		return nil
	}

	return jsonMessage
}

