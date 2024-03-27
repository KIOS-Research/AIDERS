package ws

import (
	"encoding/json"
	"log"
	"mdemet/ws/src/db"
	"net/http"

	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		// check the token before upgrading the connection
		token := r.URL.Query().Get("token")
		log.Println("Token:", token)
		tokenIsValid := db.CheckTokenValidity(token)
		if tokenIsValid {
			log.Println("Token is valid")
		} else {
			log.Println("Token is invalid")
		}
		return tokenIsValid
	},
}

// defines the structure of incoming WS messages from the client
type IncomingMessage struct {
	OperationId   int    `json:"operation_id"`
	OperationName string `json:"operation_name"`
	GetAllData    int    `json:"get_all_data"`
}

type AllDataResponseMessage struct {
	Drones   []db.Drone  `json:"drones"`
	Devices  []db.Device `json:"devices"`
	Baloras  []db.Balora `json:"baloras"`
	ErrorMsg string      `json:"error_msg"`
}

type FramesResponseMessage struct {
	Drones []db.DroneVideoFrames `json:"drones"`
}

// initializes and maintains a websocket connection with the client
// it reads the message sent by the client and responds with data retrieved from the database
func HandleWebsocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil) // upgrade the HTTP connection to a WebSocket connection
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Client connected")

	for {
		_, p, err := conn.ReadMessage() // read message from the client
		if err != nil {
			log.Println(err)
			return
		}
		// fmt.Printf("Received raw message: %s\n", p) // print the received message

		var receivedMessage IncomingMessage
		err = json.Unmarshal(p, &receivedMessage) // unmarshal the JSON data into a struct
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error

		// retrieve all data for all connected clients
		if receivedMessage.GetAllData == 1 {
			var allDataResponse AllDataResponseMessage

			drones := db.GetDrones(receivedMessage.OperationId) // drones
			allDataResponse.Drones = drones
			if allDataResponse.Drones == nil {
				allDataResponse.Drones = []db.Drone{}
			}
			devices := db.GetDevices(receivedMessage.OperationId) // devices
			allDataResponse.Devices = devices
			if allDataResponse.Devices == nil {
				allDataResponse.Devices = []db.Device{}
			}
			baloras := db.GetBaloras(receivedMessage.OperationId) // baloras
			allDataResponse.Baloras = baloras
			if allDataResponse.Baloras == nil {
				allDataResponse.Baloras = []db.Balora{}
			}

			// TODO:
			allDataResponse.ErrorMsg = "" // error message

			responseJSON, jsonError = json.Marshal(allDataResponse)
			if jsonError != nil {
				log.Println(jsonError)
			}

			// printableResponse, _ := json.MarshalIndent(allDataResponse, "", "    ")
			// fmt.Println(string(printableResponse))
		} else {
			var framesResponse FramesResponseMessage
			drones := db.GetDronesVideoFrames(receivedMessage.OperationId) // drones' video frames
			framesResponse.Drones = drones
			if framesResponse.Drones == nil {
				framesResponse.Drones = []db.DroneVideoFrames{}
			}

			responseJSON, jsonError = json.Marshal(framesResponse)
			if jsonError != nil {
				log.Println(jsonError)
			}
			// printableResponse, _ := json.MarshalIndent(framesResponse, "", "    ")
			// fmt.Println(string(printableResponse))
		}

		err = conn.WriteMessage(websocket.TextMessage, responseJSON) // respond to the client
		if err != nil {
			log.Println(err)
			return
		}
	}
}

// Lidar interface
type IncomingMessageForLidar struct {
	LidarSessionId int `json:"lidar_session_id"`
	LatestPointId  int `json:"latest_point_id"`
	NumberOfPoints int `json:"number_of_points"`
}
type LidarStartedPosition struct {
	Longitude int `json:"longitude"`
	Latitude  int `json:"latitude"`
	Altitude  int `json:"altitude"`
	Heading   int `json:"heading"`
}
type ResponseMessageLidar struct {
	LidarSessionId         int                  `json:"lidar_session_id"`
	LidarPoints            []db.LidarPoint      `json:"lidar_points"`
	LidarOriginCoordinates *db.LidarCoordinates `json:"lidar_origin_coordinates"`
}

func HandleWebsocketConnectionForLidar(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil) // upgrade the HTTP connection to a WebSocket connection
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Lidar client connected")
	for {
		_, p, err := conn.ReadMessage() // read message from the client
		if err != nil {
			log.Println(err)
			return
		}
		// fmt.Printf("Received raw message: %s\n", p) // print the received message

		var receivedMessage IncomingMessageForLidar
		err = json.Unmarshal(p, &receivedMessage) // unmarshal the JSON data into a struct
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error
		var lidarOriginCoordinates *db.LidarCoordinates
		lidarPoints := db.GetLidarPointBySessionIdAndLatestId(receivedMessage.LidarSessionId, receivedMessage.LatestPointId, receivedMessage.NumberOfPoints)
		if receivedMessage.LatestPointId == 0 {
			coordinates := db.GetAllDataLidarOriginCoordinatesByLidarSessionId(receivedMessage.LidarSessionId)
			lidarOriginCoordinates = &coordinates
		}
		responseMessage := ResponseMessageLidar{
			LidarSessionId:         receivedMessage.LidarSessionId,
			LidarPoints:            lidarPoints,
			LidarOriginCoordinates: lidarOriginCoordinates,
		}
		responseJSON, jsonError = json.Marshal(responseMessage)
		if jsonError != nil {
			log.Println(jsonError)
		}

		err = conn.WriteMessage(websocket.TextMessage, responseJSON) // respond to the client
		if err != nil {
			log.Println(err)
			return
		}
	}
}

// Cv interface
type IncomingMessageForCv struct {
	OperationId                   int                              `json:"operationId"`
	CrowdLocalizationLoaded       []db.LoadedDetectionDataPerDrone `json:"CrowdLocalizationLoaded"`
	DisasterLoaded                []db.LoadedDetectionDataPerDrone `json:"DisasterLoaded"`
	VehicleAndPersonTrackerLoaded []db.LoadedDetectionDataPerDrone `json:"VehicleAndPersonTrackerLoaded"`
	ActiveCrowdLocalization       bool                             `json:"ActiveCrowdLocalization"`
	ActiveDisasterClassification  bool                             `json:"ActiveDisasterClassification"`
	ActiveVehicleAndPersonTracker bool                             `json:"ActiveVehicleAndPersonTracker"`
}

type CvResponseMessage struct {
	CrowdLocalization       []*db.CrowdLocalization `db:"crowd_localization" json:"crowd_localization"`
	DisasterClassification  []*db.DetectedDisaster  `db:"disaster_classification" json:"disaster_classification"`
	VehicleAndPersonTracker []*db.DetectedTracker   `db:"vehicle_and_person_tracker" json:"vehicle_and_person_tracker"`
}

// initializes and maintains a websocket connection with the client for Computer Vision results
// it reads the message sent by the client and responds with data retrieved from the database
func HandleWebsocketConnectionForCv(w http.ResponseWriter, r *http.Request) {
	// upgrade the HTTP connection to a WebSocket connection
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Cv client connected")
	for {
		_, p, err := conn.ReadMessage() // read message from the client
		if err != nil {
			log.Println(err)
			return
		}

		var receivedMessage IncomingMessageForCv
		// unmarshal the JSON data into a struct
		err = json.Unmarshal(p, &receivedMessage)
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}
		// Declare the data variables
		var crowdLocalizationData []*db.CrowdLocalization
		var disasterClassificationData []*db.DetectedDisaster
		var vehicleAndPersonTrackerData []*db.DetectedTracker

		// Get the data from the database only if the corresponding active flag is true
		if receivedMessage.ActiveCrowdLocalization {
			crowdLocalizationData = db.GetLatestCrowdLocalizationResultsForActiveDrones(receivedMessage.OperationId, receivedMessage.CrowdLocalizationLoaded)
		}
		if receivedMessage.ActiveDisasterClassification {
			disasterClassificationData = db.GetLatestDetectedDisasterResultsForActiveDrones(receivedMessage.OperationId, receivedMessage.DisasterLoaded)
		}
		if receivedMessage.ActiveVehicleAndPersonTracker {
			vehicleAndPersonTrackerData = db.GetLatestDetectedVehicleAndPersonTrackerForActiveDrones(receivedMessage.OperationId)
		}

		// Create the CvResponseMessage
		responseMessage := CvResponseMessage{
			CrowdLocalization:       crowdLocalizationData,
			DisasterClassification:  disasterClassificationData,
			VehicleAndPersonTracker: vehicleAndPersonTrackerData,
		}

		// Marshal the responseMessage into JSON
		jsonResponse, err := json.Marshal(responseMessage)
		if err != nil {
			log.Println("Error encoding JSON:", err)
			return
		}

		// Send the jsonResponse back to the client
		err = conn.WriteMessage(websocket.TextMessage, jsonResponse)
		if err != nil {
			log.Println("Error sending message:", err)
			return
		}
	}
}
