package db

import "log"

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
			tl.live_stream_frame_url AS video_frame_url,
			ds.latest_frame_url AS detected_frame_url
		FROM
			aiders_drone d		
		LEFT JOIN
			aiders_telemetrylatest tl ON d.id = tl.drone_id AND tl.id = (
				SELECT max(id)
				FROM aiders_telemetrylatest
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
