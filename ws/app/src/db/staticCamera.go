package db

import (
	"log"
)

// represents the structure of the staticCameras json object
type StaticCamera struct {
	ID                      int             `db:"id" json:"id"`
	Name                    string          `db:"name" json:"name"`
	Model                   string          `db:"model" json:"model"`
	OperationID             *int            `db:"operation_id" json:"operation_id"`
	Latitude			    *float64        `db:"latitude" json:"latitude"`
	Longitude			    *float64        `db:"longitude" json:"longitude"`
	Time                    *string         `db:"time" json:"time"`
}


type StaticCameraQueryParams struct {
	OperationID int `db:"operation_id"`
}

func GetStaticCameras(_operationId int) []StaticCamera {
	// query to retrieve data for connected staticCameras
	query := `
		SELECT
			name, model, operation_id, latitude, longitude, time
		FROM
			aiders_staticcamera	
		WHERE
			is_connected_with_platform = 1 AND operation_id = :operation_id
		ORDER BY
			id ASC
	`

	queryParams := StaticCameraQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()
	// Iterate over the result set and scan into the 'staticCameras' slice
	var staticCameras []StaticCamera
	for rows.Next() {
		var staticCamera StaticCamera
		if err := rows.StructScan(&staticCamera); err != nil {
			log.Println(err)
		}

		staticCameras = append(staticCameras, staticCamera)
	}

	// Check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return staticCameras
}
