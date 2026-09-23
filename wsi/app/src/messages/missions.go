package messages

import (
	"encoding/json"
	"fmt"
	"log"
)

type IncomingMission struct {
	Action                string        `json:"action"`
	Grid                  bool          `json:"grid"`
	MissionSpeed          int           `json:"missionSpeed"`
	MissionGimbal         string        `json:"missionGimbal"`
	MissionRepeat         int           `json:"missionRepeat"`
	CaptureAndStoreImages bool          `json:"captureAndStoreImages"`
	MissionPath           []([]float64) `json:"missionPath"` // A slice of slices of float64
}

func GenerateMissionMessage(_msg json.RawMessage) []byte {
	// Unmarshal the JSON string into the struct
	var incomingMission IncomingMission
	err := json.Unmarshal([]byte(_msg), &incomingMission)
	if err != nil {
		fmt.Println("Error unmarshalling _msg:", err)
		return nil
	}

	missionCommandValue := 0
	if incomingMission.Action == "CANCEL_MISSION" {
		missionCommandValue = 1
	} else if incomingMission.Action == "PAUSE_MISSION" {
		missionCommandValue = 2
	} else if incomingMission.Action == "RESUME_MISSION" {
		missionCommandValue = 3
	}

	missionCommand := map[string]interface{}{
		"missionCommandValue":   missionCommandValue,
		"grid":                  incomingMission.Grid,
		"captureAndStoreImages": incomingMission.CaptureAndStoreImages,
		"repeat":                incomingMission.MissionRepeat,
	}

	// Looping through missionPath
	gpsInputs := []map[string]interface{}{}
	for _, point := range incomingMission.MissionPath {
		point := map[string]interface{}{
			"latitude":    point[1],
			"longitude":   point[0],
			"altitude":    int(point[2]),
			"speed":       incomingMission.MissionSpeed,
			"gimbalAngle": incomingMission.MissionGimbal,
			"stayTime":    0,
			"photo":       false,
		}
		gpsInputs = append(gpsInputs, point)
	}

	// build outgoing message
	outgoingMsg := map[string]interface{}{
		"type":           "mission",
		"missionCommand": missionCommand,
		"gpsInput":       gpsInputs,
	}

	jsonMessage, err := json.Marshal(outgoingMsg)
	if err != nil {
		log.Printf("Error marshalling message: %v", err)
		return nil
	}

	return jsonMessage
}
