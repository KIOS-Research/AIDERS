package main

import (
	"log"
	"net/http"
	"os"

	// "mdemet/wsi/src/cfc"
	"mdemet/wsi/src/db"
	"mdemet/wsi/src/middleware"
	"mdemet/wsi/src/ws"
)

func main() {

	// // Example parameters for GetBoundingPolygon
	// latitude := 40.7128   // New York City latitude
	// longitude := -74.0060 // New York City longitude
	// FOVh := 90.0          // Horizontal field of view in degrees
	// FOVv := 60.0          // Vertical field of view in degrees
	// altitude := 100.0     // Altitude in meters
	// roll := 0.0           // Roll in degrees
	// pitch := 20.0         // Pitch in degrees
	// heading := 10.0       // Heading in degrees

	// polygon := cfc.GetBoundingPolygon(latitude, longitude, FOVh, FOVv, altitude, roll, pitch, heading)
	// fmt.Println("Bounding Polygon Coordinates:")
	// for _, coord := range polygon {
	// 	fmt.Printf("Latitude: %f, Longitude: %f\n", coord[0], coord[1])
	// }
	// return

	db.Init()                                                                  // initialize the database connection
	middleware.InitKafkaProducer()                                             // initialize kafka connection
	http.HandleFunc("/wsi/", ws.WebsocketConnectionHandler)                    // WS connection handler for drones
	http.HandleFunc("/wsi/sendMessageToClient", ws.SendMessageToClientHandler) // POST request to send message to client

	http.HandleFunc("/wsi/deviceConnect/", ws.DeviceWebsocketConnectionHandler) // WS connection handler for devices
	http.HandleFunc("/wsi/sendMessageToDevice", ws.SendMessageToDeviceHandler) // POST request to send message to device


	log.Println("Websocket Server is running on :" + os.Getenv("WSI_PORT"))
	err := http.ListenAndServe(":"+os.Getenv("WSI_PORT"), nil) // Start the WebSocket server
	if err != nil {
		log.Fatal("ListenAndServe: ", err)
	}
}
