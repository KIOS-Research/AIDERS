package db

import (
	"fmt"
	"log"
)

type DetectionQueryParams struct {
	OperationId int `db:"operation_id"`
}

type LoadedDetectionDataPerDrone struct {
	ID        int    `json:"id"`
	DroneId   int    `json:"droneId"`
	DroneName string `json:"droneName"`
}

type CrowdLocalization struct {
	ID          int    `db:"id" json:"id"`
	Coordinates string `db:"coordinates" json:"coordinates"`
	Time        string `db:"time" json:"time"`
	DroneId     int    `db:"drone_id" json:"droneId"`
	DroneName   string `db:"drone_name" json:"droneName"`
}

func GetLatestCrowdLocalizationResultsForActiveDrones(_operationId int, CrowdLocalizationLoaded []LoadedDetectionDataPerDrone) []*CrowdLocalization {
	// SQL query to fetch the latest crowd localization result.
	// Loops for all Drone that are in the current Operation
	query := `
	SELECT 
		drone.id as drone_id, 
		drone.drone_name, 
		crowd_localization.id, 
		crowd_localization.time, 
		crowd_localization.coordinates
	FROM 
		aiders.aiders_drone drone
	JOIN (
		SELECT *, MAX(time) OVER (PARTITION BY drone_id) as max_time
		FROM aiders.aiders_detectioncrowdlocalizationresults
	) crowd_localization ON drone.id = crowd_localization.drone_id AND crowd_localization.time = crowd_localization.max_time
	JOIN aiders.aiders_detectionsession detection_session ON crowd_localization.detection_session_id = detection_session.id
	WHERE 
		drone.operation_id = :operation_id AND
		detection_session.is_active = true;
`

	// Prepare the query parameters.
	queryParams := DetectionQueryParams{OperationId: _operationId}

	// Execute the query.
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	var results []*CrowdLocalization
	for rows.Next() {
		var result CrowdLocalization
		if err := rows.StructScan(&result); err != nil {
			log.Println(err)
		}

		// Check if the result is already loaded.
		alreadyLoaded := false
		for _, loaded := range CrowdLocalizationLoaded {
			if loaded.ID == result.ID && loaded.DroneId == result.DroneId {
				alreadyLoaded = true
				break
			}
		}

		// If the result is not already loaded, add it to the results.
		if !alreadyLoaded {
			results = append(results, &result)
		}
	}

	// Check for errors from iterating over rows.
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return results
}

type DetectedDisaster struct {
	ID                    int     `db:"id" json:"id"`
	Latitude              float64 `db:"lat" json:"latitude"`
	Longitude             float64 `db:"lon" json:"longitude"`
	EarthquakeProbability float64 `db:"earthquake_probability" json:"earthquakeProbability"`
	FireProbability       float64 `db:"fire_probability" json:"fireProbability"`
	FloodProbability      float64 `db:"flood_probability" json:"floodProbability"`
	Time                  string  `db:"time" json:"time"`
	DroneId               int     `db:"drone_id" json:"droneId"`
	DroneName             string  `db:"drone_name" json:"droneName"`
}

// GetLatestDetectedDisasterResultsForActiveDrones retrieves the latest detected disaster results
// for active drones in the specified operation that have active disaster classification detection.
func GetLatestDetectedDisasterResultsForActiveDrones(_operationId int, DisasterLoaded []LoadedDetectionDataPerDrone) []*DetectedDisaster {
	// SQL query to fetch the latest detected disaster result.
	// Loops for all Drone that are in the current Operation
	query := `
	SELECT 
		drone.id as drone_id, 
		drone.drone_name, 
		detected_disaster.id, 
		detected_disaster.time, 
		detected_disaster.lat, 
		detected_disaster.lon, 
		detected_disaster.earthquake_probability, 
		detected_disaster.fire_probability, 
		detected_disaster.flood_probability
	FROM 
		aiders.aiders_drone drone
	JOIN (
		SELECT *, MAX(time) OVER (PARTITION BY drone_id) as max_time
		FROM aiders.aiders_detecteddisaster detected_disaster
	) detected_disaster ON drone.id = detected_disaster.drone_id AND detected_disaster.time = detected_disaster.max_time
	JOIN aiders.aiders_detectionsession detection_session ON detected_disaster.detection_session_id = detection_session.id
	WHERE 
		drone.operation_id = :operation_id AND
		detection_session.is_active = true;
`

	// Prepare the query parameters.
	queryParams := DetectionQueryParams{OperationId: _operationId}

	// Execute the query.
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	var results []*DetectedDisaster
	for rows.Next() {
		var result DetectedDisaster
		if err := rows.StructScan(&result); err != nil {
			log.Println(err)
		}

		// Check if the result is already loaded.
		alreadyLoaded := false
		for _, loaded := range DisasterLoaded {
			if loaded.ID == result.ID && loaded.DroneId == result.DroneId {
				alreadyLoaded = true
				break
			}
		}

		// If the result is not already loaded, add it to the results.
		if !alreadyLoaded {
			results = append(results, &result)
		}
	}

	// Check for errors from iterating over rows.
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return results
}

type DetectedTracker struct {
	ID                 int64   `db:"id" json:"id"`
	Time               string  `db:"time" json:"time"`
	Lat                float64 `db:"lat" json:"lat"`
	Lon                float64 `db:"lon" json:"lon"`
	Label              string  `db:"label" json:"label"`
	TrackId            int     `db:"track_id" json:"track_id"`
	DistanceFromDrone  float64 `db:"distance_from_drone" json:"distance_from_drone"`
	DetectionSessionId int64   `db:"detection_session_id" json:"detectionSession_id"`
	FrameId            int64   `db:"frame_id" json:"frame_id"`
	OperationId        int64   `db:"operation_id" json:"operationId"`
	DroneId            int     `db:"drone_id" json:"droneId"`
	DroneName          string  `db:"drone_name" json:"drone_name"`
}

func GetLatestDetectedVehicleAndPersonTrackerForActiveDrones(_operationId int) []*DetectedTracker {
	// get the detected objects form the last frame of the current session
	query := `
		SELECT d.id as drone_id, d.drone_name, do.*
		FROM aiders_detectedobject do
		JOIN (
			SELECT ds.drone_id, MAX(ds.id) as latest_session_id
			FROM aiders_detectionsession ds
			WHERE ds.operation_id = :operation_id
			AND ds.is_active = true
			GROUP BY ds.drone_id
		) latest_sessions ON do.detection_session_id = latest_sessions.latest_session_id
		JOIN aiders_drone d ON latest_sessions.drone_id = d.id
		JOIN (
			SELECT df.detection_session_id, MAX(df.id) as latest_frame_id
			FROM aiders_detectionframe df
			GROUP BY df.detection_session_id
		) latest_frames ON do.frame_id = latest_frames.latest_frame_id
		WHERE do.detection_session_id = latest_frames.detection_session_id;
	`

	// Prepare the query parameters.
	queryParams := DetectionQueryParams{OperationId: _operationId}

	// Execute the query.
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	var results []*DetectedTracker
	for rows.Next() {
		var result DetectedTracker
		if err := rows.StructScan(&result); err != nil {
			log.Println(err)
		}
		results = append(results, &result)
		fmt.Print(result) // print the received message
	}

	// Check for errors from iterating over rows.
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return results
}
