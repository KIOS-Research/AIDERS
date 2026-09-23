package messages

import (
	"encoding/json"
	//"log"
	"fmt"
	//"time"

)

type IncomingLidar struct {
	LidarCommand   int    `json:"command"`
	LidarSender    string `json:"sender"`
	LidarSessionId int    `json:"session_id"`
}


//to send start or stop lidar command
func GenerateLidarCommand(_msg json.RawMessage) []byte{
	
	var incomingLidar IncomingLidar
	err := json.Unmarshal([]byte(_msg), &incomingLidar)
	if err != nil {
		fmt.Println("Error unmarshalling _msg:", err)
		return nil
	}

	fmt.Println("Command:", incomingLidar.LidarCommand)
	fmt.Println("Sender:", incomingLidar.LidarSender)

	// build outgoing message
	outgoingMsg := map[string]interface{}{
		"type": "lidar",
		"lidarCommand": incomingLidar.LidarCommand,
		"sessionId": incomingLidar.LidarSessionId,
	}
	jsonMessage, err := json.Marshal(outgoingMsg)
	if err != nil {
		fmt.Printf("Error marshalling message: %v", err)
		return nil
	}
	return jsonMessage
}