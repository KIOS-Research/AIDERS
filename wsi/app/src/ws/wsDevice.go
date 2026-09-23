package ws

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"strings"
	"time"

	"mdemet/wsi/src/db"
	"mdemet/wsi/src/messages"

	"github.com/gorilla/websocket"
)

var devices = make(map[string]*websocket.Conn) // list of connected devices

var deviceWsUpgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

type TempDeviceMessage struct {
	Telemetry json.RawMessage `json:"telemetry"`
	Notification json.RawMessage `json:"notification"`
}

// Handle incoming websocket connections
func DeviceWebsocketConnectionHandler(_w http.ResponseWriter, _r *http.Request) {
	fmt.Println("------------------------------------")
	ws, err := deviceWsUpgrader.Upgrade(_w, _r, nil)
	if err != nil {
		log.Fatal(err)
	}
	defer ws.Close()

	fmt.Println("Full URL:", _r.URL.String())
	// Get the device's name from the URL
	pathSegments := strings.Split(_r.URL.Path, "/")

	// var name string

	name := pathSegments[len(pathSegments)-1]

	// check if name is empty or contains special characters
	if name == "" || strings.ContainsAny(name, "!@#$%^&*()+{}|:<>?") {
		log.Printf("%c Invalid device name: '%s'", '\U0000274C', name)
		return
	}

	deviceConnected(ws, name) // Handle device connection
}

// Handle device connection
func deviceConnected(_ws *websocket.Conn, _name string) {
	devices[_name] = _ws // Add/update device's ws connection object to the list
	log.Println(("CHK 1......"))
	deviceId, isAlreadyConnected := db.GetDeviceByName(_name) // retrieve device from database
	log.Printf("Device ID: %v", deviceId)
	log.Printf("isAlreadyConnected: %v", isAlreadyConnected)

	if deviceId != 0 {
		if isAlreadyConnected {
			log.Printf("%c Device '%s' is already connected", '\U0001F44D', _name)
		} else {
			db.UpdateDeviceConnectionStatus(deviceId, 1) // flag device as connected in the database
			log.Printf("%c Device '%s' has re-connected", '\U0001F504', _name)

			operationId, err := db.GetDeviceOperationId(deviceId)
			if err != nil {
				log.Printf("Error getting device operation ID: %v", err)
				return
			}

			if operationId != 0 {
				_, err := db.DeactivateDeviceSessionAndCreateNew(deviceId, _name, operationId)
				if err != nil {
					log.Printf("Error managing device session: %v", err)
					return
				}
			}
		}
	} else {

		deviceObj := map[string]interface{}{
			"name":  _name,
			"model": "Unknown", // TODO: get device model
			"ip":    "1.1.1.1",
		}

		_ = db.SaveDevice(deviceObj) // create device entry in the database
		// db.CreateDeviceInitialEntries(deviceId) // create entry for device in the database
		log.Printf("%c Device '%s' has connected", '\U0001F680', _name)
	}

	printDevicesList() // TEMP: print devices list
	listenForDeviceMessages(_ws, _name)
}

// Handle device disconnection
func deviceDisconnected(_name string, _err error) {
	deviceId, _ := db.GetDeviceByName(_name) // retrieve device from database

	if deviceId != 0 {
		db.UpdateDeviceConnectionStatus(deviceId, 0) // flag device as disconnected in the database
	}

	log.Printf("%c error from %s: %v", '\U0000274C', _name, _err)
	log.Printf("%c Device disconnected: %s", '\U0001F480', _name)
	devices[_name].Close()
	delete(devices, _name)
	printDevicesList() // TEMP: print devices list
}

// Listen for incoming messages from device
func listenForDeviceMessages(_ws *websocket.Conn, _name string) {
	deviceId, _ := db.GetDeviceByName(_name) // retrieve device from database
	log.Println(deviceId)

	connectedTimestamp := time.Now() // save the timestamp when the device connected
	fmt.Println(connectedTimestamp)

	for {
		_, msg, err := _ws.ReadMessage() // read incoming websocket message

		if err != nil {
			deviceDisconnected(_name, err)
			break
		}

		// log.Printf("%c Message received from: %s", '\U00002B05', _name)
		//log.Printf("Incoming Msg: %s", string(msg))

		//check if the device msg is telemetry,lidar, or notification
		var tempMsg TempDeviceMessage
		err = json.Unmarshal(msg, &tempMsg)
		if err != nil {
			log.Printf("Error unmarshalling raw JSON: %v", err)
		}

		// log.Println(tempMsg)
		var deviceMsg db.DeviceTelemetryMessage

		if tempMsg.Telemetry != nil {
			err := json.Unmarshal(tempMsg.Telemetry, &deviceMsg)
			if err != nil {
				log.Printf("Error unmarshalling telemetry message: %v", err)
				return
			}
			// log.Printf("Handled telemetry: %+v", deviceMsg)

			connectionDuration := time.Since(connectedTimestamp).Seconds()

			missionLogId := 0 // TODO: get mission log id
			operationId := 1  // TODO: get operation id

			// middleware.SendTelemetryToKafka(_name, deviceId, tempMsg.Telemetry, deviceMsg)

			db.SaveDeviceTelemetry(deviceId, int(connectionDuration), deviceMsg, missionLogId, operationId) // save telemetry data to the database
			// db.UpdateDeviceTelemetryLatest(deviceId, int(connectionDuration), deviceMsg, missionLogId, operationId)
		}

		var notification messages.Notification
		if tempMsg.Notification != nil {
			err := json.Unmarshal(tempMsg.Notification, &notification)
			if err != nil {
				log.Printf("Error unmarshaling notification message: %v", err)
				return
			}
			log.Printf("Incoming Notification Message from device %s: %+v", _name, notification)
			
			// save notification to the database
			deviceOperationId, _ := db.GetDeviceOperationId(deviceId)
			db.SaveIncomingDeviceNotification(_name, notification.NotificationMsg, deviceOperationId)
		}		

	}
}


// Handle incoming POST request to send message to device
func SendMessageToDeviceHandler(w http.ResponseWriter, r *http.Request) {
	var requestBody RequestBody
	err := json.NewDecoder(r.Body).Decode(&requestBody)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	log.Println("*********************************")
	log.Printf("%c Request to send message to device: %s", '\U000027A1', requestBody.Name)
	log.Printf("%c Incoming message type: %s", '\U000027A1', requestBody.Type)
	log.Printf("%c Incoming message: %s", '\U000027A1', requestBody.Msg)

	outgoingMsg := []byte{}

	if requestBody.Type == "notification" {
		outgoingMsg = messages.GenerateNotification(requestBody.Msg)
		sendWebsocketMessageToDevice(requestBody.Name, outgoingMsg)
	}

	w.Write([]byte("Message sent to device"))
}


// Send websocket message to device by name
func sendWebsocketMessageToDevice(_name string, _msg []byte) {

	log.Printf("%c Sending message to: %s", '\U000027A1', _name)
	log.Printf("%c Sending message: %s", '\U000027A1', string(_msg))

	// Check if device exists and send the message
	if device, ok := devices[_name]; ok {
		err := device.WriteMessage(websocket.TextMessage, []byte(_msg))
		if err != nil {
			deviceDisconnected(_name, err)
		}
	} else {
		log.Printf("Device %s not found", _name)
	}
}

// DEBUG: print devices list
func printDevicesList() {
	fmt.Println()
	log.Printf("%c Devices List:", '\U0001F4C3')
	i := 0
	for n := range devices {
		log.Printf("Device %d: %s", i, n)
		i++
	}
	fmt.Println()
}
