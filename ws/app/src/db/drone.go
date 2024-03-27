package db

import (
	"log"
)

// represents the structure of the drones json object
type Drone struct {
	ID                        int       `db:"id" json:"id"`
	DroneName                 string    `db:"drone_name" json:"drone_name"`
	IP                        *string   `db:"ip" json:"ip"`
	Type                      string    `db:"type" json:"type"`
	Model                     string    `db:"model" json:"model"`
	Port                      *string   `db:"port" json:"port"`
	LiveStreamUrl             *string   `db:"live_stream_url" json:"live_stream_url"`
	CameraModel               string    `db:"camera_model" json:"camera_model"`
	OperationID               *int      `db:"operation_id" json:"operation_id"`
	IsConnectedWithPlatform   bool      `db:"is_connected_with_platform" json:"is_connected_with_platform"`
	MissionID                 *int      `db:"mission_id" json:"mission_id"`
	BallisticAvailable        bool      `db:"ballistic_available" json:"ballistic_available"`
	BuildMapActivated         bool      `db:"build_map_activated" json:"build_map_activated"`
	DroneMovementAvailable    bool      `db:"drone_movement_available" json:"drone_movement_available"`
	LidarAvailable            bool      `db:"lidar_available" json:"lidar_available"`
	MultispectralAvailable    bool      `db:"multispectral_available" json:"multispectral_available"`
	WaterSamplerAvailable     bool      `db:"water_sampler_available" json:"water_sampler_available"`
	WeatherStationAvailable   bool      `db:"weather_station_available" json:"weather_station_available"`
	RequestedCollaboration    *bool     `db:"requested_collaboration" json:"requested_collaboration"`
	RespondingToCollaboration *bool     `db:"responding_to_collaboration" json:"responding_to_collaboration"`
	Time                      string    `db:"time" json:"time"`
	Telemetry                 Telemetry `db:"" json:"telemetry"`
	VideoFrameURL             *string   `db:"video_frame_url" json:"video_frame_url"`
	DetectedFrameUrl          *string   `db:"detected_frame_url" json:"detected_frame_url"`
	Detection                 Detection `db:"" json:"detection"`
	DetectionSessionId        *int      `db:"detection_session_id" json:"detection_session_id"`
	DetectionActive           *bool     `db:"detection_active" json:"detection_active"`
	Weather                   Weather   `db:"" json:"weather"`
	BuildMapLastImage         string    `db:"build_map_last_image" json:"build_map_last_image"`
}

// represents the structure of the drone's telemetry
type Telemetry struct {
	ID                  int     `db:"telemetry_id" json:"id"`
	Time                string  `db:"telemetry_time" json:"time"`
	DroneID             int     `db:"drone_id" json:"drone_id"`
	BatteryPercentage   float64 `db:"battery_percentage" json:"battery_percentage"`
	GPSSignal           int     `db:"gps_signal" json:"gps_signal"`
	Satellites          int     `db:"satellites" json:"satellites"`
	Heading             float64 `db:"heading" json:"heading"`
	Velocity            float64 `db:"velocity" json:"velocity"`
	HomeLat             float64 `db:"homeLat" json:"homeLat"`
	HomeLon             float64 `db:"homeLon" json:"homeLon"`
	Lat                 float64 `db:"lat" json:"lat"`
	Lon                 float64 `db:"lon" json:"lon"`
	Alt                 float64 `db:"alt" json:"alt"`
	DroneState          string  `db:"drone_state" json:"drone_state"`
	VtolState           *string `db:"vtol_state" json:"vtol_state"`
	FOVCoordinates      *string `db:"fov_coordinates" json:"fov_coordinates"`
	SecondsOn           float64 `db:"secondsOn" json:"secondsOn"`
	GimbalAngle         float64 `db:"gimbal_angle" json:"gimbal_angle"`
	WaterSamplerInWater bool    `db:"water_sampler_in_water" json:"water_sampler_in_water"`
	OperationID         *int    `db:"telemetry_operation_id" json:"operation_id"`
	MissionLogID        *int    `db:"mission_log_id" json:"mission_log_id"`
}

type Detection struct {
	DetectionStatus  string `db:"detection_status" json:"detection_status"`
	DetectionTypeStr string `db:"detection_type_str" json:"detection_type_str"`
	DetectionModel   string `db:"detection_model" json:"detection_model"`
}

type Weather struct {
	Time          *string  `db:"weather_time" json:"time"`
	WindSpeed     *float64 `db:"wind_speed" json:"wind_speed"`
	WindDirection *float64 `db:"wind_direction" json:"wind_direction"`
	Temperature   *float64 `db:"temperature" json:"temperature"`
	Pressure      *float64 `db:"pressure" json:"pressure"`
	Humidity      *float64 `db:"humidity" json:"humidity"`
	Heading       *float64 `db:"weather_heading" json:"heading"`
}

type DroneQueryParams struct {
	OperationID int `db:"operation_id"`
}

func GetDrones(_operationId int) []Drone {

	// query to retrieve data for connected drones
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
			t.vtol_state,
			t.secondsOn,
			t.gimbal_angle,
			t.water_sampler_in_water,
			t.operation_id AS telemetry_operation_id,
			t.mission_log_id,
			t.fov_coordinates,

			ls.latest_frame_url AS video_frame_url,

			ds.id AS detection_session_id,
			ds.latest_frame_url AS detected_frame_url,
			ds.is_active AS detection_active,

			det.detection_status,
			det.detection_type_str,
			det.detection_model,

			w.time as weather_time,
			w.wind_speed,
			w.wind_direction,
			w.temperature,
			w.pressure,
			w.humidity,
			w.heading as weather_heading,

			"" AS build_map_last_image
		FROM
			aiders_drone d
		LEFT JOIN
			aiders_telemetry t ON d.id = t.drone_id AND t.id = (
				SELECT max(id)
				FROM aiders_telemetry
				WHERE drone_id = d.id
			)			
		LEFT JOIN
			aiders_livestreamsession ls ON d.id = ls.drone_id AND ls.id = (
				SELECT max(id)
				FROM aiders_livestreamsession
				WHERE drone_id = d.id
			)
		LEFT JOIN
			aiders_detectionsession ds ON d.id = ds.drone_id AND ds.id = (
				SELECT max(id)
				FROM aiders_detectionsession
				WHERE drone_id = d.id
			)			
		LEFT JOIN
			aiders_detection det ON d.id = det.drone_id
		LEFT JOIN
			aiders_weatherstation w ON d.id = w.drone_id AND w.id = (
				SELECT max(id)
				FROM aiders_weatherstation
				WHERE drone_id = d.id
			)			
		WHERE
			d.is_connected_with_platform = 1 AND d.operation_id = :operation_id
		ORDER BY
			d.id ASC
	`

	queryParams := DroneQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	// Iterate over the result set and scan into the 'drones' slice
	var drones []Drone
	for rows.Next() {
		var drone Drone
		if err := rows.StructScan(&drone); err != nil {
			log.Println(err)
		}
		drones = append(drones, drone)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return drones
}

type DroneVideoFrames struct {
	DroneName        string  `db:"drone_name" json:"drone_name"`
	VideoFrameURL    *string `db:"video_frame_url" json:"video_frame_url"`
	DetectedFrameUrl *string `db:"detected_frame_url" json:"detected_frame_url"`
}

func GetDronesVideoFrames(_operationId int) []DroneVideoFrames {

	// query to retrieve data for connected drones
	query := `
		SELECT
			d.drone_name,
			ls.latest_frame_url AS video_frame_url,
			ds.latest_frame_url AS detected_frame_url
		FROM
			aiders_drone d		
		LEFT JOIN
			aiders_livestreamsession ls ON d.id = ls.drone_id AND ls.id = (
				SELECT max(id)
				FROM aiders_livestreamsession
				WHERE drone_id = d.id
			)
		LEFT JOIN
			aiders_detectionsession ds ON d.id = ds.drone_id AND ds.id = (
				SELECT max(id)
				FROM aiders_detectionsession
				WHERE drone_id = d.id
			)						
		WHERE
			d.is_connected_with_platform = 1 AND d.operation_id = :operation_id
		ORDER BY
			d.id ASC
	`

	queryParams := DroneQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	// Iterate over the result set and scan into the 'drones' slice
	var drones []DroneVideoFrames
	for rows.Next() {
		var droneVideoFrames DroneVideoFrames
		if err := rows.StructScan(&droneVideoFrames); err != nil {
			log.Println(err)
		}
		drones = append(drones, droneVideoFrames)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return drones
}
