package main

import (
	"encoding/json"
	"fmt"
	"log"

	_ "github.com/go-sql-driver/mysql"
	"github.com/jmoiron/sqlx"
)

// represents the structure of the drones json object
type Drone struct {
	ID                        int       `db:"id"`
	DroneName                 string    `db:"drone_name"`
	IP                        string    `db:"ip"`
	Model                     string    `db:"model"`
	CameraModel               string    `db:"camera_model"`
	OperationID               *int      `db:"operation_id"`
	IsConnectedWithPlatform   bool      `db:"is_connected_with_platform"`
	MissionID                 *int      `db:"mission_id"`
	BallisticAvailable        bool      `db:"ballistic_available"`
	BuildMapActivated         bool      `db:"build_map_activated"`
	DroneMovementAvailable    bool      `db:"drone_movement_available"`
	LidarAvailable            bool      `db:"lidar_available"`
	MultispectralAvailable    bool      `db:"multispectral_available"`
	WaterSamplerAvailable     bool      `db:"water_sampler_available"`
	WeatherStationAvailable   bool      `db:"weather_station_available"`
	RequestedCollaboration    *bool     `db:"requested_collaboration"`
	RespondingToCollaboration *bool     `db:"responding_to_collaboration"`
	Time                      string    `db:"time"`
	Telemetry                 Telemetry `db:"" json:"telemetry"`
	Weather                   string    `db:"weather"`
	VideoFrameURL             *string   `db:"video_frame_url"`
	DetectedFrameUrl          *string   `db:"detected_frame_url"`
	BuildMapLastImage         string    `db:"build_map_last_image"`
}

// represents the structure of the drone's telemetry
type Telemetry struct {
	ID                  int     `db:"telemetry_id"`
	Time                string  `db:"telemetry_time"`
	DroneID             int     `db:"drone_id"`
	BatteryPercentage   int     `db:"battery_percentage"`
	GPSSignal           int     `db:"gps_signal"`
	Satellites          int     `db:"satellites"`
	Heading             int     `db:"heading"`
	Velocity            int     `db:"velocity"`
	HomeLat             float64 `db:"homeLat"`
	HomeLon             float64 `db:"homeLon"`
	Lat                 float64 `db:"lat"`
	Lon                 float64 `db:"lon"`
	Alt                 int     `db:"alt"`
	DroneState          string  `db:"drone_state"`
	SecondsOn           float64 `db:"secondsOn"`
	GimbalAngle         int     `db:"gimbal_angle"`
	WaterSamplerInWater bool    `db:"water_sampler_in_water"`
	OperationID         *int    `db:"telemetry_operation_id"`
	MissionLogID        *int    `db:"mission_log_id"`
}

func main() {
	// Database connection
	db, err := sqlx.Connect("mysql", "a:a@tcp(127.0.0.1:3306)/aiders")
	if err != nil {
		log.Println(err)
	}
	defer db.Close()

	// Query to retrieve drones and their latest telemetry
	query := `
		SELECT
			d.*,
			t.id AS telemetry_id,
			t.time AS telemetry_time,
			t.drone_id,
			t.battery_percentage,
			t.gps_signal,
			t.satellites,
			t.heading,
			t.velocity,
			t.homeLat,
			t.homeLon,
			t.lat,
			t.lon,
			t.alt,
			t.drone_state,
			t.secondsOn,
			t.gimbal_angle,
			t.water_sampler_in_water,
			t.operation_id AS telemetry_operation_id,
			t.mission_log_id,
			"" AS weather,
			rf.frame AS video_frame_url,
			df.latest_frame_url AS detected_frame_url,
			"" AS build_map_last_image
		FROM
			aiders_drone d
		LEFT JOIN
			aiders_rawframe rf ON d.id = rf.drone_id AND rf.id = (
				SELECT max(id)
				FROM aiders_rawframe
				WHERE drone_id = d.id
			)
		LEFT JOIN
			aiders_detectionsession df ON d.id = df.drone_id AND df.id = (
				SELECT max(id)
				FROM aiders_detectionsession
				WHERE drone_id = d.id
			)			
		LEFT JOIN
			aiders_telemetry t ON d.id = t.drone_id AND t.id = (
				SELECT max(id)
				FROM aiders_telemetry
				WHERE drone_id = d.id
			)
		WHERE
			d.is_connected_with_platform = 1 AND d.operation_id = 1
		ORDER BY
			d.id ASC
	`

	var drones []Drone
	if err := db.Select(&drones, query); err != nil {
		log.Println(err)
	}

	// Convert the result to JSON
	resultJSON, err := json.MarshalIndent(drones, "", "    ")
	if err != nil {
		log.Println(err)
	}

	fmt.Println(string(resultJSON))
}
