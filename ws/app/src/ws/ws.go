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
		// log.Println("Token:", token)
		tokenIsValid := db.CheckTokenValidity(token)
		if tokenIsValid {
			log.Println("Token is valid")
		} else {
			log.Println("Token is invalid")
		}
		return tokenIsValid
	},
}

//////////////////////////////////////////////
// TELEMETRY FOR DRONES, DEVICES, BALORAS
//////////////////////////////////////////////

// defines the structure of incoming WS messages from the client
type IncomingMessage struct {
	OperationId int `json:"operation_id"`
}

type AllDataResponseMessage struct {
	Drones       []db.Drone       `json:"drones"`
	Devices      []db.Device      `json:"devices"`
	Baloras      []db.Balora      `json:"baloras"`
	StaticCameras []db.StaticCamera `json:"static_cameras"`
	ErrorMsg     string             `json:"error_msg"`
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

		staticCameras := db.GetStaticCameras(receivedMessage.OperationId) // static cameras
		allDataResponse.StaticCameras = staticCameras
		if allDataResponse.StaticCameras == nil {
			allDataResponse.StaticCameras = []db.StaticCamera{}
		}

		// TODO:
		allDataResponse.ErrorMsg = "" // error message

		responseJSON, jsonError = json.Marshal(allDataResponse)
		if jsonError != nil {
			log.Println(jsonError)
		}

		// printableResponse, _ := json.MarshalIndent(allDataResponse, "", "    ")
		// fmt.Println(string(printableResponse))

		err = conn.WriteMessage(websocket.TextMessage, responseJSON) // respond to the client
		if err != nil {
			log.Println(err)
			return
		}
	}
}

//////////////////////////////////////////////
// GROUND VEHICLES
//////////////////////////////////////////////

type IncomingMessageForGroundVehicles struct {
	OperationId int `json:"operation_id"`
}

type GroundVehiclesResponseMessage struct {
	GroundVehicles []db.GroundVehicle `json:"ground_vehicles"`
	ErrorMsg       string             `json:"error_msg"`
}

// HandleWebsocketConnectionForGroundVehicles streams ground vehicle data for an operation
// Only returns vehicles that have been updated within the last 30 seconds
func HandleWebsocketConnectionForGroundVehicles(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Ground Vehicles client connected")

	for {
		_, p, err := conn.ReadMessage()
		if err != nil {
			log.Println(err)
			return
		}

		var receivedMessage IncomingMessageForGroundVehicles
		err = json.Unmarshal(p, &receivedMessage)
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error

		groundVehicles := db.GetGroundVehicles(receivedMessage.OperationId)
		
		responseMessage := GroundVehiclesResponseMessage{
			GroundVehicles: groundVehicles,
			ErrorMsg:       "",
		}

		if responseMessage.GroundVehicles == nil {
			responseMessage.GroundVehicles = []db.GroundVehicle{}
		}

		responseJSON, jsonError = json.Marshal(responseMessage)
		if jsonError != nil {
			log.Println(jsonError)
		}

		err = conn.WriteMessage(websocket.TextMessage, responseJSON)
		if err != nil {
			log.Println(err)
			return
		}
	}
}

//////////////////////////////////////////////
// DRONE REMOTE IDS
//////////////////////////////////////////////

type IncomingMessageForDroneRids struct {
	OperationId int `json:"operation_id"`
}

type DroneRidsResponseMessage struct {
	DroneRids []db.DroneRid `json:"drone_rids"`
	ErrorMsg  string        `json:"error_msg"`
}

// HandleWebsocketConnectionForDroneRids streams drone Remote ID data
// Only returns devices that have been updated within the last 60 seconds
func HandleWebsocketConnectionForDroneRids(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Drone RIDs client connected")

	for {
		_, p, err := conn.ReadMessage()
		if err != nil {
			log.Println(err)
			return
		}

		var receivedMessage IncomingMessageForDroneRids
		err = json.Unmarshal(p, &receivedMessage)
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error

		droneRids := db.GetDroneRids(receivedMessage.OperationId)
		
		responseMessage := DroneRidsResponseMessage{
			DroneRids: droneRids,
			ErrorMsg:  "",
		}

		if responseMessage.DroneRids == nil {
			responseMessage.DroneRids = []db.DroneRid{}
		}

		responseJSON, jsonError = json.Marshal(responseMessage)
		if jsonError != nil {
			log.Println(jsonError)
		}

		err = conn.WriteMessage(websocket.TextMessage, responseJSON)
		if err != nil {
			log.Println(err)
			return
		}
	}
}

//////////////////////////////////////////////
// ADSB AIRCRAFT
//////////////////////////////////////////////

// defines the structure of incoming WS messages from the client
type IncomingMessageForAdsbAircraft struct {
	OperationId int `json:"operation_id"`
}

type AdsbAircraftResponseMessage struct {
	AdsbAircraft []db.AdsbAircraft `json:"adsb_aircraft"`
	ErrorMsg     string            `json:"error_msg"`
}

// initializes and maintains a websocket connection with the client for ADS-B aircraft
func HandleWebsocketConnectionForAdsbAircraft(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("ADS-B Aircraft client connected")

	for {
		_, p, err := conn.ReadMessage()
		if err != nil {
			log.Println(err)
			return
		}

		var receivedMessage IncomingMessageForAdsbAircraft
		err = json.Unmarshal(p, &receivedMessage)
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error

		adsbAircraft := db.GetAdsbAircraft(receivedMessage.OperationId)
		
		responseMessage := AdsbAircraftResponseMessage{
			AdsbAircraft: adsbAircraft,
			ErrorMsg:     "",
		}

		if responseMessage.AdsbAircraft == nil {
			responseMessage.AdsbAircraft = []db.AdsbAircraft{}
		}

		responseJSON, jsonError = json.Marshal(responseMessage)
		if jsonError != nil {
			log.Println(jsonError)
		}

		err = conn.WriteMessage(websocket.TextMessage, responseJSON)
		if err != nil {
			log.Println(err)
			return
		}
	}
}

//////////////////////////////////////////////
// STREAM AND DETECT VIDEO FRAMES
//////////////////////////////////////////////

type IncomingMessageForFrames struct {
	OperationId int `json:"operation_id"`
}

type FramesResponseMessage struct {
	Drones []db.DroneVideoFrames `json:"drones"`
}

// initializes and maintains a websocket connection with the client for video frames
func HandleWebsocketConnectionForFrames(w http.ResponseWriter, r *http.Request) {

	conn, err := upgrader.Upgrade(w, r, nil) // upgrade the HTTP connection to a WebSocket connection
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Frames Client connected")

	for {
		_, p, err := conn.ReadMessage() // read message from the client
		if err != nil {
			log.Println(err)
			return
		}
		// fmt.Printf("Received raw message: %s\n", p) // print the received message

		var receivedMessage IncomingMessageForFrames
		err = json.Unmarshal(p, &receivedMessage) // unmarshal the JSON data into a struct
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error
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
		//printableResponse, _ := json.MarshalIndent(framesResponse, "", "    ")
		//fmt.Println(string(printableResponse))

		err = conn.WriteMessage(websocket.TextMessage, responseJSON) // respond to the client
		if err != nil {
			log.Println(err)
			return
		}
	}
}

//////////////////////////////////////////////
// CRISIS CLASSIFICATION
//////////////////////////////////////////////

// type IncomingNotification struct {
// 	NotificationMsg    string `json:"message"`
// 	NotificationSender string `json:"sender"`
// }

type IncomingMessageForCrisis struct {
	GetAllData    bool   `json:"get_all_data"`
	LastID        int    `json:"last_id"`
	LastTimestamp string `json:"last_timestamp"` // used to filter data based on the last timestamp
}

type ResponseMessageForCrisis struct {
	CrisisData []db.CrisisClassification `json:"crisis_data"`
}

func HandleWebsocketConnectionForCrisisClassification(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil) // upgrade the HTTP connection to a WebSocket connection
	if err != nil {
		log.Println(err)
		return
	}
	defer conn.Close()
	log.Println("Crisis classification client connected")

	for {
		_, p, err := conn.ReadMessage() // read message from the client
		if err != nil {
			log.Println(err)
			return
		}

		var receivedMessage IncomingMessageForCrisis
		err = json.Unmarshal(p, &receivedMessage)
		if err != nil {
			log.Println("Error decoding JSON:", err)
			return
		}

		var responseJSON []byte
		var jsonError error

		if receivedMessage.GetAllData {
			crisisData := db.GetAllCrisisClassificationData(receivedMessage.LastID, receivedMessage.LastTimestamp)
			// log.Println(crisisData)
			responseMessage := ResponseMessageForCrisis{
				CrisisData: crisisData,
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
}

//////////////////////////////////////////////
// LIDAR
//////////////////////////////////////////////

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

// initializes and maintains a websocket connection with the client for Lidar results
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

//////////////////////////////////////////////
// DETECTED OBJECTS
//////////////////////////////////////////////

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
	CrowdLocalization          []*db.CrowdLocalization         `db:"crowd_localization" json:"crowd_localization"`
	DisasterClassification     []*db.DetectedDisaster          `db:"disaster_classification" json:"disaster_classification"`
	VehicleAndPersonTracker    []*db.DetectedObject            `db:"vehicle_and_person_tracker" json:"vehicle_and_person_tracker"`
	DetectionObjectDescription []*db.DetectedObjectDescription `db:"detection_description" json:"detection_description"`
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
		var detectionObjectsData []*db.DetectedObject
		var detectionDescriptionData []*db.DetectedObjectDescription

		// Get the data from the database only if the corresponding active flag is true
		if receivedMessage.ActiveCrowdLocalization {
			crowdLocalizationData = db.GetLatestCrowdLocalizationResultsForActiveDrones(receivedMessage.OperationId, receivedMessage.CrowdLocalizationLoaded)
		}
		if receivedMessage.ActiveDisasterClassification {
			disasterClassificationData = db.GetLatestDetectedDisasterResultsForActiveDrones(receivedMessage.OperationId, receivedMessage.DisasterLoaded)
		}
		if receivedMessage.ActiveVehicleAndPersonTracker {
			detectionObjectsData = db.GetLatestDetectedObjectsForActiveDetections(receivedMessage.OperationId)
			//log.Println("DD:", detectionObjectsData)
			detectionDescriptionData = db.GetLatestDetectedDescriptionForActiveDetections(receivedMessage.OperationId)
		}

		// Create the CvResponseMessage
		responseMessage := CvResponseMessage{
			CrowdLocalization:          crowdLocalizationData,
			DisasterClassification:     disasterClassificationData,
			VehicleAndPersonTracker:    detectionObjectsData,
			DetectionObjectDescription: detectionDescriptionData,
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
