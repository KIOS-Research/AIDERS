package db

import (
	"log"
)

// represents the structure of the devices json object
type Device struct {
	ID                      int             `db:"id" json:"id"`
	Name                    string          `db:"name" json:"name"`
	Operator                string          `db:"operator" json:"operator"`
	IP                      string          `db:"ip" json:"ip"`
	Model                   string          `db:"model" json:"model"`
	OperationID             *int            `db:"operation_id" json:"operation_id"`
	IsConnectedWithPlatform bool            `db:"is_connected_with_platform" json:"is_connected_with_platform"`
	Time                    *string         `db:"time" json:"time"`
	Telemetry               DeviceTelemetry `db:"" json:"telemetry"`
	Images                  string          `db:"images" json:"images"`
}

// represents the structure of the device's telemetry
type DeviceTelemetry struct {
	ID                int     `db:"telemetry_id" json:"id"`
	Time              string  `db:"telemetry_time" json:"time"`
	DeviceID          int     `db:"device_id" json:"device_id"`
	Latitude          float64 `db:"latitude" json:"latitude"`
	Longitude         float64 `db:"longitude" json:"longitude"`
	Altitude          float64 `db:"altitude" json:"altitude"`
	Heading           float64 `db:"heading" json:"heading"`
	Ox                float64 `db:"orientation_x" json:"orientation_x"`
	Oy                float64 `db:"orientation_y" json:"orientation_y"`
	Oz                float64 `db:"orientation_z" json:"orientation_z"`
	Ax                float64 `db:"accelerometer_x" json:"accelerometer_x"`
	Ay                float64 `db:"accelerometer_y" json:"accelerometer_y"`
	Az                float64 `db:"accelerometer_z" json:"accelerometer_z"`
	Gx                float64 `db:"gyroscope_x" json:"gyroscope_x"`
	Gy                float64 `db:"gyroscope_y" json:"gyroscope_y"`
	Gz                float64 `db:"gyroscope_z" json:"gyroscope_z"`
	GeoX              float64 `db:"geomagnetic_x" json:"geomagnetic_x"`
	GeoY              float64 `db:"geomagnetic_y" json:"geomagnetic_y"`
	GeoZ              float64 `db:"geomagnetic_z" json:"geomagnetic_z"`
	Light             float64 `db:"light" json:"light"`
	Step              float64 `db:"step" json:"step"`
	Pressure          float64 `db:"pressure" json:"pressure"`
	Proximity         float64 `db:"proximity" json:"proximity"`
	BatteryPercentage float64 `db:"battery_percentage" json:"battery_percentage"`
	OperationID       *int    `db:"telemetry_operation_id" json:"operation_id"`
	SecondsOn         float64 `db:"secondsOn" json:"secondsOn"`
}

type DeviceQueryParams struct {
	OperationID int `db:"operation_id"`
}

func GetDevices(_operationId int) []Device {
	// query to retrieve data for connected devices
	query := `
		SELECT
			d.*,

			t.id AS telemetry_id,
			t.time AS telemetry_time,
			t.device_id,
			t.latitude,
			t.longitude,
			t.altitude,
			t.heading,
			t.orientation_x,
			t.orientation_y,
			t.orientation_z,
			t.accelerometer_x,
			t.accelerometer_y,
			t.accelerometer_z,
			t.gyroscope_x,
			t.gyroscope_y,
			t.gyroscope_z,
			t.geomagnetic_x,
			t.geomagnetic_y,
			t.geomagnetic_z,
			t.light,
			t.step,
			t.pressure,
			t.proximity,
			t.battery_percentage,
			t.operation_id AS telemetry_operation_id,
			t.secondsOn,

			"" AS images
		FROM
			aiders_device d
		LEFT JOIN
			aiders_devicetelemetry t ON d.id = t.device_id AND t.id = (
				SELECT max(id)
				FROM aiders_devicetelemetry
				WHERE device_id = d.id
			)		
		WHERE
			d.is_connected_with_platform = 1 AND d.operation_id = :operation_id
		ORDER BY
			d.id ASC
	`

	queryParams := DeviceQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()
	// Iterate over the result set and scan into the 'devices' slice
	var devices []Device
	for rows.Next() {
		var device Device
		if err := rows.StructScan(&device); err != nil {
			log.Println(err)
		}

		devices = append(devices, device)
	}

	// Check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return devices
}
