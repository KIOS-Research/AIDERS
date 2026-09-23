package db

import (
	"log"
)

// represents the structure of ground vehicles
type GroundVehicle struct {
	ID          int      `db:"id" json:"id"`
	Name        string   `db:"name" json:"name"`
	Model       string   `db:"model" json:"model"`
	Service     *string  `db:"service" json:"service"`
	Lat         *float64 `db:"lat" json:"lat"`
	Lon         *float64 `db:"lon" json:"lon"`
	Speed       float64  `db:"speed" json:"speed"`
	Heading     float64  `db:"heading" json:"heading"`
	Status      string   `db:"status" json:"status"`
	LastUpdated string   `db:"last_updated" json:"last_updated"`
	OperationID *int     `db:"operation_id" json:"operation_id"`
}

type GroundVehicleQueryParams struct {
	OperationID int `db:"operation_id"`
}

// GetGroundVehicles retrieves all ground vehicles for an operation
// that have been updated within the last 30 seconds
func GetGroundVehicles(_operationId int) []GroundVehicle {
	query := `
	SELECT
		gv.id,
		gv.name,
		gv.model,
		gv.service,
		gv.lat,
		gv.lon,
		gv.speed,
		gv.heading,
		gv.status,
		gv.last_updated,
		gv.operation_id
	FROM
		aiders_groundvehicle gv
	WHERE
		gv.last_updated >= DATE_SUB(NOW(), INTERVAL 30 SECOND)
	ORDER BY
		gv.id ASC
	`
	// AND gv.operation_id = :operation_id

	queryParams := GroundVehicleQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	// Iterate over the result set and scan into the 'groundVehicles' slice
	var groundVehicles []GroundVehicle
	for rows.Next() {
		var vehicle GroundVehicle
		if err := rows.StructScan(&vehicle); err != nil {
			log.Println(err)
		}
		groundVehicles = append(groundVehicles, vehicle)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return groundVehicles
}
