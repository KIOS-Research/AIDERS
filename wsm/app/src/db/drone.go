package db

import (
	"database/sql"
	"encoding/json"
	"fmt"
	"time"
)

type DroneTelemetryMessage struct {
	Latitude          float64  `json:"latitude"`
	Longitude         float64  `json:"longitude"`
	Altitude          float64  `json:"altitude"`
	Velocity          float64  `json:"velocity"`
	Heading           float64  `json:"heading"`
	GimbalAngle       float64  `json:"gimbalAngle"`
	GpsSignal         *int     `json:"gpsSignal"`
	SatelliteNumber   *int     `json:"satelliteNumber"`
	HomeLatitude      *string  `json:"homeLatitude"`
	HomeLongitude     *string  `json:"homeLongitude"`
	DroneState        *string  `json:"droneState"`
	BatteryPercentage *float64 `json:"batteryPercentage"`
	VtolState         *string  `json:"vtolState"`
}

// fetches a drone's id and connection status by its name from the database
func GetDroneByName(name string) (int64, bool) {
	var id int64
	var isConnectedWithPlatform bool
	query := "SELECT id, is_connected_with_platform FROM aiders_drone WHERE drone_name = ? LIMIT 1"
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
func GetDroneLatestTelemetryID(droneID int64) (int, error) {
	query := "SELECT id FROM aiders.aiders_telemetry WHERE drone_id = ? ORDER BY time DESC LIMIT 1"

	var telemetryID int
	err := Conn.QueryRow(query, droneID).Scan(&telemetryID)

	if err != nil {
		if err == sql.ErrNoRows {
			return 0, nil // No telemetry found for this droneID
		}
		return 0, fmt.Errorf("error executing query: %w", err)
	}

	return telemetryID, nil
}

// updates the connection status of a drone in the database
func UpdateDroneConnectionStatusAndType(_droneID int64, _status int, _type string, _connectionType string) error {
	// Assuming db is a *sql.DB object that's already been initialized elsewhere
	var query string
	if _status == 0 {
		//query = "UPDATE aiders_drone SET is_connected_with_platform = ?, type = ?, build_map_activated = 0 WHERE id = ?"
		query = "UPDATE aiders_drone SET is_connected_with_platform = ?, type = ?, connection_type = ?, build_map_activated = 0, is_live_stream_connected = 0 WHERE id = ?"
	} else {
		query = "UPDATE aiders_drone SET is_connected_with_platform = ?, type = ?, connection_type = ? WHERE id = ?"
	}

	// Prepare the statement
	stmt, err := Conn.Prepare(query)
	if err != nil {
		return fmt.Errorf("error preparing query: %w", err)
	}
	defer stmt.Close()

	// Execute the statement with parameters
	_, err = stmt.Exec(_status, _type, _connectionType, _droneID)
	if err != nil {
		return fmt.Errorf("error executing query: %w", err)
	}

	return nil
}

// // inserts a new drone record into the database and returns the inserted ID
// func SaveDrone(droneObj map[string]interface{}) int64 {
// 	query := `INSERT INTO aiders_drone
//               (drone_name, ip, model, camera_model, time, is_connected_with_platform,
//                ballistic_available, water_sampler_available, weather_station_available,
//                multispectral_available, lidar_available, drone_movement_available, build_map_activated, type)
//               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`

// 	// Prepare the statement
// 	stmt, err := Conn.Prepare(query)
// 	if err != nil {
// 		return 0
// 	}
// 	defer stmt.Close()

// 	// Execute the statement
// 	res, err := stmt.Exec(
// 		droneObj["name"], droneObj["ip"], droneObj["model"], droneObj["cameraModel"], time.Now(),
// 		1, droneObj["ballisticAvailable"], droneObj["waterSamplerAvailable"], droneObj["weatherStationAvailable"],
// 		droneObj["multispectralAvailable"], droneObj["lidarAvailable"], 0, 0, droneObj["type"],
// 	)
// 	if err != nil {
// 		return 0
// 	}

// 	// Get the last inserted ID
// 	droneId, err := res.LastInsertId()
// 	if err != nil {
// 		return 0
// 	}

// 	return droneId
// }

// checks for an existing telemetry record for a drone and inserts a new one if none exists
func CreateDroneInitialEntries(droneId int64) error {
	// Check if a record with the same drone_id already exists
	checkQuery := "SELECT * FROM aiders_telemetrylatest WHERE drone_id = ?"
	var existingRecord int
	err := Conn.QueryRow(checkQuery, droneId).Scan(&existingRecord)
	if err != nil && err != sql.ErrNoRows {
		return err
	}
	// If no record exists, insert a new one
	if err == sql.ErrNoRows {
		insertQuery := "INSERT INTO aiders_telemetrylatest (drone_id, time_updated) VALUES (?, ?)"
		_, err := Conn.Exec(insertQuery, droneId, time.Now())
		if err != nil {
			return err
		}

		insertQuery2 := `INSERT INTO aiders_detection
						(drone_id, detection_status, detection_type_str, detection_model)
						VALUES (?, ?, ?, ?)`
		_, err2 := Conn.Exec(insertQuery2, droneId, "DETECTION_INITIAL_STATUS", "NO_ACTIVE_DETECTOR", "NO_ACTIVE_MODEL")
		if err2 != nil {
			return err2
		}
	}
	return nil
}

// inserts a new telemetry record for a drone into the database
func SaveDroneTelemetry(droneId int64, secondsOn int, telemetryMsg DroneTelemetryMessage, missionLogId int, operationId int, fovPolygon [][2]float64) (telemetryId int64, err error) {
	// fmt.Println("Saving telemetry")
	fovPolygonJSON, err := json.Marshal(fovPolygon)
	if err != nil {
		return 0, err
	}

	// TODO: Get home coordinates from the telemetryMsg
	var homeLat float64 = 0
	var homeLon float64 = 0

	query := `
        INSERT INTO aiders_telemetry
        (drone_id, lat, lon, alt, heading, velocity, gps_signal, satellites, homeLat, homeLon, drone_state, mission_log_id,
        gimbal_angle, water_sampler_in_water, battery_percentage, operation_id, secondsOn, fov_coordinates, time)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    `
	res, err := Conn.Exec(query, droneId, telemetryMsg.Latitude, telemetryMsg.Longitude, telemetryMsg.Altitude, telemetryMsg.Heading, telemetryMsg.Velocity,
		telemetryMsg.GpsSignal, telemetryMsg.SatelliteNumber, homeLat, homeLon, telemetryMsg.DroneState, nil,
		telemetryMsg.GimbalAngle, false, telemetryMsg.BatteryPercentage, operationId, secondsOn, fovPolygonJSON, time.Now())
	if err != nil {
		fmt.Println(err)
		return 0, err
	}

	return res.LastInsertId()
}

// updates the latest telemetry record for a drone in the database
func UpdateDroneTelemetryLatest(droneId int64, secondsOn int, telemetryMsg DroneTelemetryMessage, missionLogId int, operationId int, fovPolygon [][2]float64) error {
	// fmt.Println("Updating telemetry")
	fovPolygonJSON, err := json.Marshal(fovPolygon)
	if err != nil {
		fmt.Println(err)
		return err
	}

	// TODO: Get home coordinates from the telemetryMsg
	var homeLat float64 = 0
	var homeLon float64 = 0

	query := `
        UPDATE aiders_telemetrylatest
        SET lat = ?, lon = ?, alt = ?, heading = ?, velocity = ?, gps_signal = ?, satellites = ?,
        homeLat = ?, homeLon = ?, drone_state = ?, mission_log_id = ?, gimbal_angle = ?,
        water_sampler_in_water = ?, battery_percentage = ?, operation_id = ?, secondsOn = ?, fov_coordinates = ?, time_updated = ?
        WHERE drone_id = ?
    `
	_, err = Conn.Exec(query, telemetryMsg.Latitude, telemetryMsg.Longitude, telemetryMsg.Altitude, telemetryMsg.Heading, telemetryMsg.Velocity,
		telemetryMsg.GpsSignal, telemetryMsg.SatelliteNumber, homeLat, homeLon, telemetryMsg.DroneState, nil,
		telemetryMsg.GimbalAngle, false, telemetryMsg.BatteryPercentage, operationId, secondsOn, fovPolygonJSON, time.Now(), droneId)

	return err
}
