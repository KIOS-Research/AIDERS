package messages

import (
	"encoding/json"
	"fmt"
	"log"
)

type IncomingBuildMap struct {
	Command string `json:"command"`
	Interval int    `json:"interval"`
}

func GenerateBuildMapMessage(_msg json.RawMessage) []byte {
	// Unmarshal the JSON string into the struct
	var incomingBuildMap IncomingBuildMap
	err := json.Unmarshal([]byte(_msg), &incomingBuildMap)
	if err != nil {
		fmt.Println("Error unmarshalling _msg:", err)
		return nil
	}

	buildmapCommand := 0
	if incomingBuildMap.Command == "START" {
		buildmapCommand = 1
	}

	// build outgoing message
	outgoingMsg := map[string]interface{}{
		"type":            "buildmap",
		"buildmapCommand": buildmapCommand,
		"interval":        incomingBuildMap.Interval,
	}

	jsonMessage, err := json.Marshal(outgoingMsg)
	if err != nil {
		log.Printf("Error marshalling message: %v", err)
		return nil
	}

	return jsonMessage
}
