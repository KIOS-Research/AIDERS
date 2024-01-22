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
	http.HandleFunc("/", ws.HandleWebsocketConnection)
	http.HandleFunc("/getLidarPointsBySessionId", ws.HandleWebsocketConnectionForLidar)
	log.Println("Websocket Server is running on :" + os.Getenv("WS_PORT"))
	err := http.ListenAndServe(":"+os.Getenv("WS_PORT"), nil) // Start the WebSocket server on port 8087
	if err != nil {
		log.Fatal("Error starting server: ", err)
	}
}
