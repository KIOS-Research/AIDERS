package db

import (
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

type DetectedObject struct {
	ID                 int64    `db:"id" json:"id"`
	Time               string   `db:"time" json:"time"`
	Lat                float64  `db:"lat" json:"lat"`
	Lon                float64  `db:"lon" json:"lon"`
	Label              string   `db:"label" json:"label"`
	TrackId            int      `db:"track_id" json:"track_id"`
	DistanceFromDrone  *float64 `db:"distance_from_drone" json:"distance_from_drone"`
	DetectionSessionID int64    `db:"detection_session_id" json:"detection_session_id"`
	FrameID            int64    `db:"frame_id" json:"frame_id"`
	BoundingBoxes      *string  `db:"bounding_boxes" json:"bounding_boxes"`
	Confidence         *float64 `db:"confidence" json:"confidence"`
	//ObjectID           *int     `db:"object_id" json:"object_id"`
	OperationId int64  `db:"operation_id" json:"operationId"`
	DroneId     int    `db:"drone_id" json:"droneId"`
	DroneName   string `db:"drone_name" json:"drone_name"`

	// Fields from DetectionInfo
	MsgIdentifier *string `db:"msg_identifier" json:"msg_identifier"`
	UavStatus     *string `db:"uav_status" json:"uav_status"`
	SentUtc       *string `db:"sent_utc" json:"sent_utc"`
	District      *string `db:"district" json:"district"`
	MissionID     *string `db:"mission_id" json:"mission_id"`
}

func GetLatestDetectedObjectsForActiveDetections(operationId int) []*DetectedObject {
	// Initialize detectedObjects as an empty slice of pointers to DetectedObject
	detectedObjects := []*DetectedObject{}

	// SQL query to get the latest detected objects for active drones
	queryDetectionObject := `
	    SELECT
       		ado.id,
    		ado.time,
    		ado.lat,
    		ado.lon,
    		ado.label,
    		ado.bounding_boxes,
    		ado.confidence,
    		ado.track_id,
    		ado.distance_from_drone,
    		ado.detection_session_id,
    		ado.frame_id,
    		ado.operation_id,
    		ado.drone_id,
    		ad.drone_name
        	FROM aiders_detectedobject AS ado
        JOIN (
            SELECT ad.frame_id
            FROM aiders_detectionsession AS ads
            JOIN aiders_detectedobject AS ad ON ads.id = ad.detection_session_id
            WHERE ads.operation_id = :operation_id
              AND ads.is_active = 1
            ORDER BY ad.time DESC
            LIMIT 1
        ) AS latest_frame ON ado.frame_id = latest_frame.frame_id
        JOIN aiders_drone AS ad ON ado.drone_id = ad.id;
			
	`

	// Prepare the query parameters
	queryParams := DetectionQueryParams{OperationId: operationId}

	// Execute the query to get detected objects
	rows, err := Conn.NamedQuery(queryDetectionObject, queryParams)
	if err != nil {
		return nil
	}

	defer rows.Close()

	// Scan rows into detectedObjects slice
	for rows.Next() {
		var detectedObject DetectedObject
		if err := rows.StructScan(&detectedObject); err != nil {
			log.Println(err)
			return detectedObjects // Return empty slice on error
		}
		// Append the address of detectedObject to the slice
		detectedObjects = append(detectedObjects, &detectedObject)
	}

	// Check for any errors after iterating over the rows
	if err := rows.Err(); err != nil {
		log.Println(err)
		return detectedObjects // Return empty slice on error
	}
	return detectedObjects
}

type DetectedObjectDescription struct {
	ID                 int64  `db:"id" json:"id"`
	TrackId            int    `db:"track_id" json:"track_id"`
	Description        string `db:"description" json:"description"`
	IsSuspicious       bool   `db:"is_suspicious" json:"is_suspicious"`
	ShouldFollow       bool   `db:"should_follow" json:"should_follow"`
	UpdatedAt          string `db:"updated_at" json:"updated_at"`
	DetectionSessionId int64  `db:"detection_session_id" json:"detection_session"`
	UpdatedById        int64  `db:"updated_by_id" json:"updated_by_id"`
}

func GetLatestDetectedDescriptionForActiveDetections(operationId int) []*DetectedObjectDescription {
	var detectedDescriptions []*DetectedObjectDescription

	// SQL query to get the latest detection descriptions
	query := `
        SELECT
            dod.id,
            dod.track_id,
            dod.description,
            dod.is_suspicious,
            dod.should_follow,
            dod.updated_at,
            dod.detection_session_id,
            dod.updated_by_id
        FROM aiders_detectedobjectdescription dod
        JOIN aiders_detectionsession ds ON dod.detection_session_id = ds.id
        WHERE ds.is_active = 1
          AND ds.operation_id = :operation_id
          AND dod.updated_at = (
              SELECT MAX(dod_inner.updated_at)
              FROM aiders_detectedobjectdescription dod_inner
              WHERE dod_inner.track_id = dod.track_id
                AND dod_inner.detection_session_id = dod.detection_session_id
          );
    `
	// Prepare the query parameters
	queryParams := DetectionQueryParams{OperationId: operationId}

	// Execute the second query to get detected descriptions
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		return nil
	}
	defer rows.Close()

	// Scan rows into detectedDescriptions slice
	for rows.Next() {
		var detectedDescription DetectedObjectDescription
		if err := rows.StructScan(&detectedDescription); err != nil {
			return nil
		}
		detectedDescriptions = append(detectedDescriptions, &detectedDescription)
	}
	if err := rows.Err(); err != nil {
		return nil
	}
	return detectedDescriptions
}
