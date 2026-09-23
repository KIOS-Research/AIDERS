package db

import (
	"log"
	"time"
)

func GetLoraMasterIdAndIsConnectedByName(_name string) (int64, bool) {
	// query to retrieve data for a baloramaster by name
	query := `
		SELECT
			bm.id,
			bm.is_connected_with_platform
		FROM
			aiders_baloramaster bm
		WHERE
			bm.name = :name;
	`

	queryParams := map[string]interface{}{
		"name": _name,
	}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
		return 0, false
	}
	defer rows.Close()

	// Scan the result into variables
	var id int64
	var isConnected bool
	if rows.Next() {
		if err := rows.Scan(&id, &isConnected); err != nil {
			log.Println(err)
		}
	} else {
		// No rows in the result set, return 0 for id
		id = 0
	}

	// Return the id and is_connected_with_platform
	return id, isConnected
}

func UpdateLoraMasterConnectionStatus(loraMasterName string, status bool) {
	// query to update the is_connected_with_platform status for a baloramaster by name
	query := `
		UPDATE
			aiders_baloramaster bm
		SET
			bm.is_connected_with_platform = :status
		WHERE
			bm.name = :name;
	`

	queryParams := map[string]interface{}{
		"name":   loraMasterName,
		"status": status,
	}

	// Use NamedExec to execute the query
	_, err := Conn.NamedExec(query, queryParams)
	if err != nil {
		log.Println(err)
	}
}

func CreateLoraMaster(loraMasterName string) {
	// log.Println("+++++++++++++ Creating new Lora Master:", loraMasterName)
	// query to update the is_connected_with_platform status for a baloramaster by name
	query := `
		INSERT INTO
			aiders_baloramaster (time, name, is_connected_with_platform)
		VALUES (:time, :name, :status);
	`

	queryParams := map[string]interface{}{
		"time":   time.Now(),
		"name":   loraMasterName,
		"status": true,
	}
	// Use NamedExec to execute the query
	_, err := Conn.NamedExec(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	// log.Println("------------ Created new Lora Master:", loraMasterName)
}

func GetOperationIdByLoraMasterId(loraMasterId int64) int {
	// query to get the operation_id for a baloramaster by id
	query := `
		SELECT 
			operation_id 
		FROM 
			aiders.aiders_baloramaster 
		WHERE 
			id = :id
	`
	queryParams := map[string]interface{}{
		"id": loraMasterId,
	}

	var operationId int
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	for rows.Next() {
		err := rows.Scan(&operationId)
		if err != nil {
			log.Println(err)
		}
	}

	return operationId
}

func GetLoraIdByLoraName(loraName string) int {
	// query to get the operation_id for a baloramaster by id
	query := `
        SELECT 
            id 
        FROM 
            aiders_balora 
        WHERE name = ? 
        LIMIT 1
    `
	var loraId int
	rows, err := Conn.Query(query, loraName)
	if err != nil {
		log.Println(err)
		return 0 // Return 0 or any appropriate default value when there's an error
	}
	defer rows.Close()
	if rows.Next() {
		err := rows.Scan(&loraId)
		if err != nil {
			log.Println(err)
			return 0 // Return 0 or any appropriate default value when there's an error
		}
	} else {
		return 0 // Return 0 or any appropriate default value when no record is found
	}
	return loraId
}

func CreateLora(loraName string, loraMasterId int64) int {
	// query to insert a new lora device with the given name
	query := `
        INSERT INTO 
            aiders_balora (name, baloraMaster_id, time) 
        VALUES 
            (?, ?, ?)
    `
	// Execute the insert query
	result, err := Conn.Exec(query, loraName, loraMasterId, time.Now())
	if err != nil {
		log.Println(err)
		return 0 // Return 0 or any appropriate default value when there's an error
	}
	// Get the ID of the last inserted row
	loraId, err := result.LastInsertId()
	if err != nil {
		log.Println(err)
		return 0 // Return 0 or any appropriate default value when there's an error
	}
	// Convert the ID to int and return
	return int(loraId)
}

type LoraTelemetryQueryParams struct {
	Time                             time.Time `db:"time"`
	Latitude                         float64   `db:"latitude"`
	Longitude                        float64   `db:"longitude"`
	PM1                              float64   `db:"pm1"`
	PM25                             float64   `db:"pm25"`
	NOx                              float64   `db:"nox"`
	VOC                              float64   `db:"voc"`
	Temp                             float64   `db:"temp"`
	Humidity                         float64   `db:"humidity"`
	AccX                             float64   `db:"acc_x"`
	AccY                             float64   `db:"acc_y"`
	AccZ                             float64   `db:"acc_z"`
	ReceivedSignalStrengthIndication float64   `db:"received_signal_strength_indication"`
	SecondsOn                        int64     `db:"secondsOn"`
	BaloraID                         int       `db:"balora_id"`
	BaloraMasterID                   int64     `db:"baloraMaster_id"`
	OperationID                      int       `db:"operation_id"`
}

func SaveLoraTelemetry(loraTelemetry LoraTelemetryQueryParams) {
	query := `
	INSERT INTO aiders_baloratelemetry 
		(time, latitude, longitude, pm1, pm25, nox, voc, temp, humidity, acc_x, acc_y, acc_z, received_signal_strength_indication, secondsOn, balora_id, baloraMaster_id, operation_id)
	VALUES 
		(:time, :latitude, :longitude, :pm1, :pm25, :nox, :voc, :temp, :humidity, :acc_x, :acc_y, :acc_z, :received_signal_strength_indication, :secondsOn, :balora_id, :baloraMaster_id, :operation_id);
	`
	_, err := Conn.NamedExec(query, loraTelemetry)
	if err != nil {
		log.Println(err)
	}
}

type BaloraMonitorQueryParams struct {
	Time                             time.Time `db:"time"`
	CpuUsage                         float64   `db:"cpu_usage"`
	HeapMemory                       float64   `db:"heap_memory"`
	BatteryPercentage                float64   `db:"battery_percentage"`
	ReceivedSignalStrengthIndication float64   `db:"received_signal_strength_indication"`
	SecondsOn                        int64     `db:"secondsOn"`
	BaloraID                         int       `db:"balora_id"`
	BaloraMasterID                   int64     `db:"baloraMaster_id"`
	OperationID                      int       `db:"operation_id"`
}

func SaveBaloraMonitor(baloraMonitor BaloraMonitorQueryParams) {
	query := `
	INSERT INTO aiders_baloramonitor 
		(time, cpu_usage, heap_memory, battery_percentage, received_signal_strength_indication, secondsOn, balora_id, baloraMaster_id, operation_id)
	VALUES 
		(:time, :cpu_usage, :heap_memory, :battery_percentage, :received_signal_strength_indication, :secondsOn, :balora_id, :baloraMaster_id, :operation_id);
	`
	_, err := Conn.NamedExec(query, baloraMonitor)
	if err != nil {
		log.Println(err)
	}
}
