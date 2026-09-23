package main

import (
	"log"
	"net/http"
	"os"

	"mdemet/ws/src/db"
	"mdemet/ws/src/ws"
)

func main() {
	db.Init() // initialize the database connection
	http.HandleFunc("/ws/", ws.HandleWebsocketConnection)
	http.HandleFunc("/ws/getFrames", ws.HandleWebsocketConnectionForFrames)
	// http.HandleFunc("/ws/getLidarPointsBySessionId", ws.HandleWebsocketConnectionForLidar)
	http.HandleFunc("/ws/getCrisisClassificationData", ws.HandleWebsocketConnectionForCrisisClassification)
	http.HandleFunc("/ws/getCurrentActiveComputerVisionResultsByOperationId", ws.HandleWebsocketConnectionForCv)
	http.HandleFunc("/ws/getGroundVehicles", ws.HandleWebsocketConnectionForGroundVehicles)
	http.HandleFunc("/ws/getDroneRids", ws.HandleWebsocketConnectionForDroneRids)
	http.HandleFunc("/ws/getAdsbAircraft", ws.HandleWebsocketConnectionForAdsbAircraft)
	log.Println("Websocket Server is running on :" + os.Getenv("WS_PORT"))
	err := http.ListenAndServe(":"+os.Getenv("WS_PORT"), nil) // Start the WebSocket server on port 8087
	if err != nil {
		log.Println("Error starting server: ", err)
	}
}
