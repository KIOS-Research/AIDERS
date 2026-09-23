package messages

import (
	"encoding/json"
	"fmt"
	"log"
)

type IncomingBuildMap struct {
	Command string `json:"command"`
	Overlap int    `json:"overlap"`
}

func GenerateBuildMapMessage(_msg json.RawMessage) []byte {
	// Unmarshal the JSON string into the struct
	var incomingMission IncomingBuildMap
	err := json.Unmarshal([]byte(_msg), &incomingMission)
	if err != nil {
		fmt.Println("Error unmarshalling _msg:", err)
		return nil
	}

	buildmapCommand := 0
	if incomingMission.Command == "START" {
		buildmapCommand = 1
	}

	// build outgoing message
	outgoingMsg := map[string]interface{}{
		"type":            "buildmap",
		"buildmapCommand": buildmapCommand,
		"interval":        3, // TODO: dynamic interval
	}

	jsonMessage, err := json.Marshal(outgoingMsg)
	if err != nil {
		log.Printf("Error marshalling message: %v", err)
		return nil
	}

	return jsonMessage
}
