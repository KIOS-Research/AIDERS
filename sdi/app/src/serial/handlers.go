package serial

import (
	"log"
	"sdi/src/db"
	"strconv"
	"strings"
	"time"
)

var loraMasterId int64
var connectedTime int64

func handleLoraMasterConnectionStatus(messageParts []string) {
	loraMasterName := messageParts[1]
	loraIsRequestingConnection := strings.ToLower(messageParts[2])

	if loraIsRequestingConnection == "true" {

		var isConnected bool
		loraMasterId, isConnected = db.GetLoraMasterIdAndIsConnectedByName(loraMasterName)

		if loraMasterId != 0 {
			if isConnected {
				log.Printf("👍 Lora '%s' is already connected", loraMasterName)
			} else {
				db.UpdateLoraMasterConnectionStatus(loraMasterName, true)
				connectedTime = time.Now().Unix()
				log.Printf("🔄 Lora '%s' has re-connected", loraMasterName)
			}
		} else {
			db.CreateLoraMaster(loraMasterName)
			log.Printf("🚀 Lora '%s' has connected", loraMasterName)
		}
		// HandShake
		SendMessage("HandshakeLora,True")
	} else {
		db.UpdateLoraMasterConnectionStatus(loraMasterName, false)
		log.Printf("💀 Lora '%s' has disconnected", loraMasterName)
	}
}

func handleTelemetryLora(messageParts []string) {
	loraName := messageParts[1]
	latitude, _ := strconv.ParseFloat(messageParts[2], 64)
	longitude, _ := strconv.ParseFloat(messageParts[3], 64)
	pm1, _ := strconv.ParseFloat(messageParts[4], 64)
	pm25, _ := strconv.ParseFloat(messageParts[5], 64)
	nox, _ := strconv.ParseFloat(messageParts[6], 64)
	voc, _ := strconv.ParseFloat(messageParts[7], 64)
	temp, _ := strconv.ParseFloat(messageParts[8], 64)
	humidity, _ := strconv.ParseFloat(messageParts[9], 64)
	acc_x, _ := strconv.ParseFloat(messageParts[10], 64)
	acc_y, _ := strconv.ParseFloat(messageParts[11], 64)
	acc_z, _ := strconv.ParseFloat(messageParts[12], 64)
	received_signal_strength_indication, _ := strconv.ParseFloat(messageParts[13], 64)


	secondsOn := time.Now().Unix() - connectedTime

	operationId := db.GetOperationIdByLoraMasterId(loraMasterId)
	loraId := db.GetLoraIdByLoraName(loraName)
	if loraId == 0 {
		loraId = db.CreateLora(loraName, loraMasterId)
	}

	loraTelemetry := db.LoraTelemetryQueryParams{
		Time:                             time.Now(),
		Latitude:                         latitude,
		Longitude:                        longitude,
		PM1:                              pm1,
		PM25:                             pm25,
		AccX:                             acc_x,
		AccY:                             acc_y,
		AccZ:                             acc_z,
		NOx:                              nox,
		VOC:                              voc,
		Humidity:                         humidity,
		Temp:                             temp,
		ReceivedSignalStrengthIndication: received_signal_strength_indication,
		SecondsOn:                        secondsOn,
		BaloraID:                         loraId,
		BaloraMasterID:                   loraMasterId,
		OperationID:                      operationId,
	}

	db.SaveLoraTelemetry(loraTelemetry)

}

func handleMonitorLora(messageParts []string) {
	loraName := messageParts[1]
	battery_percentage, _ := strconv.ParseFloat(messageParts[2], 64)
	cpu_usage, _ := strconv.ParseFloat(messageParts[3], 64)
	heap_memory, _ := strconv.ParseFloat(messageParts[4], 64)
	received_signal_strength_indication, _ := strconv.ParseFloat(messageParts[5], 64)
	secondsOn := time.Now().Unix() - connectedTime
	operationId := db.GetOperationIdByLoraMasterId(loraMasterId)

	loraId := db.GetLoraIdByLoraName(loraName)
	if loraId == 0 {
		loraId = db.CreateLora(loraName, loraMasterId)
	}

	baloraMonitor := db.BaloraMonitorQueryParams{
		Time:                             time.Now(),
		CpuUsage:                         cpu_usage,
		HeapMemory:                       heap_memory,
		BatteryPercentage:                battery_percentage,
		ReceivedSignalStrengthIndication: received_signal_strength_indication,
		SecondsOn:                        secondsOn,
		BaloraID:                         loraId,
		BaloraMasterID:                   loraMasterId,
		OperationID:                      operationId,
	}
	db.SaveBaloraMonitor(baloraMonitor)

}
