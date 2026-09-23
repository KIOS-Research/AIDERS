package main

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"strconv"
	"time"

	"mdemet/wsm/src/cfc"
	"mdemet/wsm/src/db"

	"github.com/gorilla/websocket"
)

type RequestBody struct {
	Id        string `json:"id"`
	Name      string `json:"name"`
	IPAddress string `json:"ip"`
	Port      string `json:"port"`
}

func main() {

	db.Init() // initialize the database connection
	// http.HandleFunc("/wsm/", ws.WebsocketConnectionHandler)
	http.HandleFunc("/wsm/connectToUav", handleConnectToUav)
	// http.HandleFunc("/wsm/sendMessageToClient", ws.SendMessageToClientHandler) // POST request to send message to client
	log.Println("Server is running on :" + os.Getenv("WSM_PORT"))
	err := http.ListenAndServe(":"+os.Getenv("WSM_PORT"), nil) // Start the WebSocket server
	if err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}

func handleConnectToUav(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Invalid request method", http.StatusMethodNotAllowed)
		return
	}

	var reqBody RequestBody
	err := json.NewDecoder(r.Body).Decode(&reqBody)
	if err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	ipAddress := reqBody.IPAddress
	if ipAddress == "" {
		http.Error(w, "IP address is required", http.StatusBadRequest)
		return
	}

	port := reqBody.Port
	if port == "" {
		http.Error(w, "Port is required", http.StatusBadRequest)
		return
	}

	// Connect to WebSocket server
	wsURL := fmt.Sprintf("ws://%s:%s/telemetry", ipAddress, port)
	fmt.Println("Connecting to WebSocket server at URL:", wsURL)
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		fmt.Printf("Failed to connect to WebSocket server: %v", err)
		http.Error(w, fmt.Sprintf("Failed to connect to WebSocket server: %v", err), http.StatusInternalServerError)
		return
	}
	// defer conn.Close()

	// Send a success response
	fmt.Println("Connected to WebSocket server successfully")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte("Connected to WebSocket server successfully"))

	droneId, _ := strconv.ParseInt(reqBody.Id, 10, 64)
	db.CreateDroneInitialEntries(droneId)
	db.UpdateDroneConnectionStatusAndType(droneId, 1, "MAVLINK", "WEBSOCKETS") // flag drone as connected in the database
	connectedTimestamp := time.Now()                                           // save the timestamp when the client connected
	log.Printf("%c Drone '%s' has connected", '\U0001F680', reqBody.Name)

	// Start reading messages from the WebSocket server
	go func() {
		for {
			_, message, err := conn.ReadMessage()
			if err != nil {
				if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
					log.Printf("Unexpected WebSocket closure: %v", err)
				} else {
					log.Printf("WebSocket connection closed: %v", err)
				}
				break
			}
			// log.Printf("Received message: %s", message)
			var droneMsg db.DroneTelemetryMessage
			err = json.Unmarshal(message, &droneMsg)
			if err != nil {
				log.Printf("Failed to unmarshal message: %v", err)
				continue
			}

			// Insert telemetry data into the database
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
		log.Println("Exiting WebSocket reading loop")
	}()
}
