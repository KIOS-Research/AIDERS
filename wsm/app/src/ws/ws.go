package ws

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"strings"
	"sync"
	"time"

	"mdemet/wsm/src/cfc"
	"mdemet/wsm/src/db"
	"mdemet/wsm/src/messages"
	"mdemet/wsm/src/requests"

	"github.com/gorilla/websocket"
)

type TempDroneMessage struct {
	Telemetry    json.RawMessage `json:"telemetry"`
	Lidar        json.RawMessage `json:"lidar"`
	Notification json.RawMessage `json:"notification"`
}

var clients = make(map[string]*websocket.Conn) // list of connected clients
var LidarSessionID = 0

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

// Handle incoming websocket connections
func WebsocketConnectionHandler(_w http.ResponseWriter, _r *http.Request) {
	ws, err := upgrader.Upgrade(_w, _r, nil)
	if err != nil {
		log.Fatal(err)
	}
	defer ws.Close()

	fmt.Println("Full URL:", _r.URL.String())
	// Get the client's name from the URL
	pathSegments := strings.Split(_r.URL.Path, "/")

	//handle camera and drone models when using WS simulator
	var name string
	var cameraModel string
	var droneModel string

	if pathSegments[len(pathSegments)-3] == "" {
		name = pathSegments[len(pathSegments)-1]
		cameraModel = "cameraModel_temp"
		droneModel = "droneModel_temp"
	} else {
		name = pathSegments[len(pathSegments)-3]
		// Receiving drone and camera models to update buildmap cameraModel
		droneModel = pathSegments[len(pathSegments)-2]
		cameraModel = pathSegments[len(pathSegments)-1]

	}
	// check if name is empty or contains special characters
	if name == "" || strings.ContainsAny(name, "!@#$%^&*()+{}|:<>?") {
		log.Printf("%c Invalid client name: '%s'", '\U0000274C', name)
		return
	}

	clientConnected(ws, name, droneModel, cameraModel) // Handle client connection
}

// Handle client connection
func clientConnected(_ws *websocket.Conn, _name string, _droneModel string, _cameraModel string) {
	clients[_name] = _ws // Add/update client's ws connection object to the list

	droneId, isAlreadyConnected := db.GetDroneByName(_name) // retrieve client from database
	log.Printf("Drone ID: %v", droneId)
	log.Printf("isAlreadyConnected: %v", isAlreadyConnected)

	if droneId != 0 {
		if isAlreadyConnected {
			log.Printf("%c Drone '%s' is already connected", '\U0001F44D', _name)
		} else {
			db.UpdateDroneConnectionStatusAndType(droneId, 1, "WS") // flag drone as connected in the database
			log.Printf("%c Drone '%s' has re-connected", '\U0001F504', _name)
		}
	} else {
		// droneObj := map[string]interface{}{
		// 	"name":                    _name,
		// 	"model":                   _droneModel, //set drone model
		// 	"ip":                      "1.1.1.1",
		// 	"cameraModel":             _cameraModel, //set camera model
		// 	"ballisticAvailable":      false,
		// 	"waterSamplerAvailable":   false,
		// 	"weatherStationAvailable": false,
		// 	"multispectralAvailable":  false,
		// 	"lidarAvailable":          false,
		// 	"type":                    "WS",
		// }

		// // droneId = db.SaveDrone(droneObj)      // create drone entry in the database
		// // db.CreateDroneInitialEntries(droneId) // create entry in aiders_telemetrylatest and aiders_detection
		// log.Printf("%c Drone '%s' has connected", '\U0001F680', _name)
	}

	// legacyLSC
	// requests.StartDroneLiveStreamCapture(int(droneId), _name) // request stream capture start

	printClientsList() // TEMP: print clients list
	listenForMessages(_ws, _name)
}

// Handle client disconnection
func clientDisconnected(_name string, _err error) {
	droneId, _ := db.GetDroneByName(_name) // retrieve client from database

	if droneId != 0 {
		db.UpdateDroneConnectionStatusAndType(droneId, 0, "WS") // flag drone as disconnected in the database
		requests.StopDroneDetector(int(droneId), _name)         // stop detector service
	}

	log.Printf("%c error from %s: %v", '\U0000274C', _name, _err)
	log.Printf("%c Client disconnected: %s", '\U0001F480', _name)
	clients[_name].Close()
	delete(clients, _name)
	printClientsList() // TEMP: print clients list
}

// Listen for incoming messages from client
func listenForMessages(_ws *websocket.Conn, _name string) {
	droneId, _ := db.GetDroneByName(_name) // retrieve client from database

	connectedTimestamp := time.Now() // save the timestamp when the client connected
	fmt.Println(connectedTimestamp)

	for {
		_, msg, err := _ws.ReadMessage() // read incoming websocket message
		if err != nil {
			clientDisconnected(_name, err)
			break
		}

		// log.Printf("%c Message received from: %s", '\U00002B05', _name)
		//log.Printf("Incoming Msg: %s", string(msg))

		//check if the drone msg is telemetry,lidar, or notification
		var tempMsg TempDroneMessage
		err = json.Unmarshal(msg, &tempMsg)
		if err != nil {
			log.Printf("Error unmarshalling raw JSON: %v", err)
		}

		var droneMsg db.DroneTelemetryMessage
		var lidarMsg []db.LidarPoint
		var notification messages.IncomingNotification

		// waitgroup to synchronize goroutines
		var wg sync.WaitGroup

		// channel to capture errors from goroutines
		errCh := make(chan error, 3)

		//to launch 3 goroutines (one for each type)
		wg.Add(3)

		// handle telemetry in a goroutine
		go func() {
			defer wg.Done() // Decrement the WaitGroup counter when done
			if tempMsg.Telemetry != nil {
				err := json.Unmarshal(tempMsg.Telemetry, &droneMsg)
				if err != nil {
					errCh <- fmt.Errorf("Error unmarshaling telemetry message: %v", err)
					return
				}
				//log.Printf("Handled telemetry: %+v", droneMsg)
				var fovPolygon [][2]float64
				if droneMsg.GimbalAngle < -25 {
					FOVh := 68.0
					FOVv := 40.0

					// Convert Degress to Radians
					FOVhRadians := FOVh * (math.Pi / 180.0)
					FOVvRadians := FOVv * (math.Pi / 180.0)
					GimbalAngleRadians := (droneMsg.GimbalAngle + 90) * (math.Pi / 180.0)
					HeadingRadians := (droneMsg.Heading + 180%360) * (math.Pi / 180.0)

					fovPolygon = cfc.GetBoundingPolygon(droneMsg.Latitude, droneMsg.Longitude, FOVhRadians, FOVvRadians, droneMsg.Altitude, 0, GimbalAngleRadians, HeadingRadians)
				} else {
					fovPolygon = nil
				}
				connectionDuration := time.Since(connectedTimestamp).Seconds()

				missionLogId := 0 // TODO: get mission log id
				operationId := 1  // TODO: get operation id

				db.SaveDroneTelemetry(droneId, int(connectionDuration), droneMsg, missionLogId, operationId, fovPolygon) // save telemetry data to the database
				db.UpdateDroneTelemetryLatest(droneId, int(connectionDuration), droneMsg, missionLogId, operationId, fovPolygon)
			}
		}()

		// Handle lidar in a goroutine
		go func() {
			defer wg.Done() // Decrement the WaitGroup counter when done
			if tempMsg.Lidar != nil {
				err := json.Unmarshal(tempMsg.Lidar, &lidarMsg)
				if err != nil {
					errCh <- fmt.Errorf("Error unmarshaling lidar message: %v", err)
					return
				}
				//log.Printf("Lidar Message: %+v", lidarMsg)

				//get latest telemetry id from drone id
				telemetryId, err := db.GetDroneLatestTelemetryID(droneId)

				if err != nil {
					log.Println("Retreiving telemetry id Error:", err)
				}

				db.SaveDroneLidarPointDataInBatches(lidarMsg, telemetryId, LidarSessionID)

			}
		}()

		// Handle notification in a goroutine
		go func() {
			defer wg.Done() // Decrement the WaitGroup counter when done
			if tempMsg.Notification != nil {
				err := json.Unmarshal(tempMsg.Notification, &notification)
				if err != nil {
					errCh <- fmt.Errorf("Error unmarshaling notification message: %v", err)
					return
				}
				log.Printf("Notification Message: %+v", notification)
			}
		}()

		// Wait for all goroutines to finish
		wg.Wait()

		close(errCh)

	}
}

// {"latitude": 33.3333, "longitude": 35.353535, "altitude": 22, "velocity": 3.4, "heading": 90, "gimbalAngle": 45, "gpsSignal": 4, "satelliteNumber": 6, "homeLatitude": 33.3333, "homeLongitude": 35.353535, "droneState": "In_Mission", "batteryPercentage": 90}

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

// Incoming POST request body structure for sending message to client
type RequestBody struct {
	Name string          `json:"name"`
	Type string          `json:"type"`
	Msg  json.RawMessage `json:"msg"`
}

// Handle incoming POST request to send message to client
func SendMessageToClientHandler(w http.ResponseWriter, r *http.Request) {
	var requestBody RequestBody
	err := json.NewDecoder(r.Body).Decode(&requestBody)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	log.Println(("*********************************"))
	log.Printf("%c Request to send message to: %s", '\U000027A1', requestBody.Name)
	log.Printf("%c Incoming message type: %s", '\U000027A1', requestBody.Type)
	log.Printf("%c Incoming message: %s", '\U000027A1', requestBody.Msg)

	// check message type and handle accordingly
	outgoingMsg := []byte{}

	if requestBody.Type == "mission" {
		outgoingMsg = messages.GenerateMissionMessage(requestBody.Msg)
		sendWebsocketMessageToClient(requestBody.Name, outgoingMsg)
	} else if requestBody.Type == "buildmap" {
		outgoingMsg = messages.GenerateBuildMapMessage(requestBody.Msg)
		sendWebsocketMessageToClient(requestBody.Name, outgoingMsg)
	} else if requestBody.Type == "notification" {
		outgoingMsg = messages.GenerateNotification(requestBody.Msg)
		sendNotificationToClient(outgoingMsg)
	} else if requestBody.Type == "lidar" {
		outgoingMsg = messages.GenerateLidarCommand(requestBody.Msg)

		// TODO: save the lidar session ID in a variable
		//LidarSessionID = outgoingMsg["sessionID"]

		// Unmarshal outgoingMsg into a map to retrieve sessionID
		var lidarCommandMsg map[string]interface{}
		err := json.Unmarshal(outgoingMsg, &lidarCommandMsg)
		if err != nil {
			fmt.Println("Error unmarshalling outgoingMsg:", err)
			return
		}
		if sessionId, ok := lidarCommandMsg["sessionId"].(float64); ok {
			LidarSessionID = int(sessionId)
			//fmt.Printf("Retrieved LidarSessionID: %d\n", LidarSessionID)
		} else {
			fmt.Println("session id not found in data or is not of the expected type")
		}

		sendWebsocketMessageToClient(requestBody.Name, outgoingMsg)
	}

	//sendWebsocketMessageToClient(requestBody.Name, outgoingMsg)
	w.Write([]byte("Message sent to client"))
}

// Send notification to all clients
func sendNotificationToClient(_msg []byte) {

	i := 0
	for n := range clients {
		log.Printf("Client %d: %s", i, n)
		sendWebsocketMessageToClient(n, _msg)
		i++
	}
}

// Send websocket message to client by name
func sendWebsocketMessageToClient(_name string, _msg []byte) {

	log.Printf("%c Sending message to: %s", '\U000027A1', _name)
	log.Printf("%c Sending message: %s", '\U000027A1', string(_msg))

	// Check if client exists and send the message
	if client, ok := clients[_name]; ok {
		err := client.WriteMessage(websocket.TextMessage, []byte(_msg))
		if err != nil {
			clientDisconnected(_name, err)
		}
	} else {
		log.Printf("Client %s not found", _name)
	}
}

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

// DEBUG: print clients list
func printClientsList() {
	fmt.Println()
	log.Printf("%c Clients List:", '\U0001F4C3')
	i := 0
	for n := range clients {
		log.Printf("Client %d: %s", i, n)
		i++
	}
	fmt.Println()
}
