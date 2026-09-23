package db

import (
	"database/sql"
	"fmt"
	"math"
	"time"
)

type DeviceTelemetryMessage struct {
	Latitude          float64 `json:"latitude"`
	Longitude         float64 `json:"longitude"`
	Altitude          float64 `json:"altitude"`
	Heading           float64 `json:"heading"`
	BatteryPercentage int32   `json:"batteryPercentage"`
}

// fetches a device's id and connection status by its name from the database
func GetDeviceByName(name string) (int64, bool) {
	var id int64
	var isConnectedWithPlatform bool
	query := "SELECT id, is_connected_with_platform FROM aiders_device WHERE name = ? LIMIT 1"
	err := Conn.QueryRow(query, name).Scan(&id, &isConnectedWithPlatform)
	if err != nil {
		if err == sql.ErrNoRows {
			return 0, false // Assuming 0 and false as default values when no rows are found
		}
		return 0, false
	}
	return id, isConnectedWithPlatform
}

// fetches a telemetry id by done id from the database
func GetDeviceLatestTelemetryID(deviceID int64) (int, error) {
	query := "SELECT id FROM aiders.aiders_devicetelemetry WHERE device_id = ? ORDER BY time DESC LIMIT 1"

	var telemetryID int
	err := Conn.QueryRow(query, deviceID).Scan(&telemetryID)

	if err != nil {
		if err == sql.ErrNoRows {
			return 0, nil // No telemetry found for this deviceID
		}
		return 0, fmt.Errorf("error executing query: %w", err)
	}

	return telemetryID, nil
}

// updates the connection status of a device in the database
func UpdateDeviceConnectionStatus(_deviceID int64, _status int) error {
	// Assuming db is a *sql.DB object that's already been initialized elsewhere
	query := "UPDATE aiders_device SET is_connected_with_platform = ? WHERE id = ?"

	// Prepare the statement
	stmt, err := Conn.Prepare(query)
	if err != nil {
		return fmt.Errorf("error preparing query: %w", err)
	}
	defer stmt.Close()

	// Execute the statement with parameters
	_, err = stmt.Exec(_status, _deviceID)
	if err != nil {
		return fmt.Errorf("error executing query: %w", err)
	}

	return nil
}

// inserts a new device record into the database and returns the inserted ID
func SaveDevice(deviceObj map[string]interface{}) int64 {
	query := `INSERT INTO aiders_device
              (name, operator, model, ip, is_connected_with_platform)
              VALUES (?, ?, ?, ?, ?)`

	// Prepare the statement
	stmt, err := Conn.Prepare(query)
	if err != nil {
		return 0
	}
	defer stmt.Close()

	// Execute the statement
	res, err := stmt.Exec(
		deviceObj["name"], deviceObj["name"], deviceObj["model"], deviceObj["ip"], 1,
	)
	if err != nil {
		return 0
	}

	// Get the last inserted ID
	deviceId, err := res.LastInsertId()
	if err != nil {
		return 0
	}

	return deviceId
}

// // checks for an existing telemetry record for a device and inserts a new one if none exists
// func CreateDeviceInitialEntries(deviceId int64) error {
// 	// Check if a record with the same device_id already exists
// 	checkQuery := "SELECT * FROM aiders_devicetelemetrylatest WHERE device_id = ?"
// 	var existingRecord int
// 	err := Conn.QueryRow(checkQuery, deviceId).Scan(&existingRecord)
// 	if err != nil && err != sql.ErrNoRows {
// 		return err
// 	}
// 	// If no record exists, insert a new one
// 	if err == sql.ErrNoRows {
// 		insertQuery := "INSERT INTO aiders_devicetelemetrylatest (device_id, time_updated) VALUES (?, ?)"
// 		_, err := Conn.Exec(insertQuery, deviceId, time.Now())
// 		if err != nil {
// 			return err
// 		}

// 		insertQuery2 := `INSERT INTO aiders_detection
// 						(device_id, detection_status, detection_type_str, detection_model)
// 						VALUES (?, ?, ?, ?)`
// 		_, err2 := Conn.Exec(insertQuery2, deviceId, "DETECTION_INITIAL_STATUS", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
// 		if err2 != nil {
// 			return err2
// 		}
// 	}
// 	return nil
// }

// inserts a new telemetry record for a device into the database
func SaveDeviceTelemetry(deviceId int64, secondsOn int, telemetryMsg DeviceTelemetryMessage, missionLogId int, operationId int) (telemetryId int64, err error) {
	// fmt.Println("Saving telemetry")
	// fmt.Println(telemetryMsg.BatteryPercentage)

	query := `
        INSERT INTO aiders_devicetelemetry
        (device_id, latitude, longitude, altitude, heading, battery_percentage, operation_id, secondsOn, time,
		orientation_x, orientation_y, orientation_z, accelerometer_x, accelerometer_y, accelerometer_z, gyroscope_x, gyroscope_y, gyroscope_z, geomagnetic_x, geomagnetic_y, geomagnetic_z, light, step, pressure, proximity)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)
    `
	res, err := Conn.Exec(query, deviceId, telemetryMsg.Latitude, telemetryMsg.Longitude, int(math.Round(telemetryMsg.Altitude)), telemetryMsg.Heading,
		telemetryMsg.BatteryPercentage, operationId, secondsOn, time.Now())
	if err != nil {
		fmt.Println(err)
		return 0, err
	}

	return res.LastInsertId()
}

// // updates the latest telemetry record for a device in the database
// func UpdateDeviceTelemetryLatest(deviceId int64, secondsOn int, telemetryMsg DeviceTelemetryMessage, missionLogId int, operationId int, fovPolygon [][2]float64) error {
// 	// fmt.Println("Updating telemetry")
// 	fovPolygonJSON, err := json.Marshal(fovPolygon)
// 	if err != nil {
// 		fmt.Println(err)
// 		return err
// 	}

// 	query := `
//         UPDATE aiders_devicetelemetrylatest
//         SET lat = ?, lon = ?, alt = ?, heading = ?, velocity = ?, gps_signal = ?, satellites = ?,
//         homeLat = ?, homeLon = ?, device_state = ?, mission_log_id = ?, gimbal_angle = ?,
//         water_sampler_in_water = ?, battery_percentage = ?, operation_id = ?, secondsOn = ?, fov_coordinates = ?, time_updated = ?
//         WHERE device_id = ?
//     `
// 	_, err = Conn.Exec(query, telemetryMsg.Latitude, telemetryMsg.Longitude, telemetryMsg.Altitude, telemetryMsg.Heading, telemetryMsg.Velocity,
// 		telemetryMsg.GpsSignal, telemetryMsg.SatelliteNumber, telemetryMsg.HomeLatitude, telemetryMsg.HomeLongitude, telemetryMsg.DeviceState, nil,
// 		telemetryMsg.GimbalAngle, false, telemetryMsg.BatteryPercentage, operationId, secondsOn, fovPolygonJSON, time.Now(), deviceId)

// 	return err
// }

// deactivates current device session and creates a new one
func DeactivateDeviceSessionAndCreateNew(deviceId int64, deviceName string, operationId int) (int64, error) {
	// Update query to deactivate current sessions
	updateQuery := "UPDATE aiders_devicesession SET is_active = 0 WHERE device_id = ?"

	// Execute update query
	_, err := Conn.Exec(updateQuery, deviceId)
	if err != nil {
		return 0, fmt.Errorf("error deactivating device session: %w", err)
	}

	// Get current datetime and format it
	currentDatetime := time.Now()
	formattedDatetime := currentDatetime.Format("2006-01-02_15.04.05")
	folderPath := fmt.Sprintf("Device_Images_%s_%s", deviceName, formattedDatetime)

	// Insert query to create new session
	insertQuery := `INSERT INTO aiders_devicesession 
					(device_id, start_time, is_active, operation_id, user_id, folder_path) 
					VALUES (?, ?, ?, ?, ?, ?)`

	// Execute insert query
	res, err := Conn.Exec(insertQuery, deviceId, currentDatetime, 1, operationId, 1, folderPath)
	if err != nil {
		return 0, fmt.Errorf("error creating new device session: %w", err)
	}

	// Get the session ID of the newly created record
	sessionId, err := res.LastInsertId()
	if err != nil {
		return 0, fmt.Errorf("error getting session ID: %w", err)
	}

	return sessionId, nil
}

// fetches the operation_id for a device by its id
func GetDeviceOperationId(deviceId int64) (int, error) {
	var operationId int
	query := "SELECT operation_id FROM aiders.aiders_device WHERE id = ?"
	err := Conn.QueryRow(query, deviceId).Scan(&operationId)
	if err != nil {
		if err == sql.ErrNoRows {
			return 0, fmt.Errorf("no device found with id %d", deviceId)
		}
		return 0, fmt.Errorf("error executing query: %w", err)
	}
	return operationId, nil
}

// saves an incoming device notification to the database
func SaveIncomingDeviceNotification(_name, _msg string, _deviceOperationId int) error {
	query := "INSERT INTO aiders_devicenotification (sender, receiver, incoming, message, operation_id, timestamp) VALUES (?, ?, ?, ?, ?, ?)"
	_, err := Conn.Exec(query, _name, "platform", 1, _msg, _deviceOperationId, time.Now())
	if err != nil {
		return fmt.Errorf("error saving device notification: %w", err)
	}
	return nil
}