package db

import (
	"log"
)

// represents the structure of the baloras json object
type Balora struct {
	ID         int             `db:"balora_id" json:"id"`
	Name       string          `db:"name" json:"name"`
	MasterName string          `db:"baloraMaster_name" json:"baloraMaster_id"`
	Telemetry  BaloraTelemetry `db:"" json:"telemetry"`
}

// represents the structure of the balora's telemetry
type BaloraTelemetry struct {
	ID             int     `db:"id" json:"id"`
	Time           string  `db:"time" json:"time"`
	BaloraMasterID int     `db:"balora_id" json:"balora_id"`
	BaloraID       int     `db:"baloraMaster_id" json:"baloraMaster_id"`
	Latitude       float64 `db:"latitude" json:"latitude"`
	Longitude      float64 `db:"longitude" json:"longitude"`
	Pm1            float64 `db:"pm1" json:"pm1"`
	Pm25           float64 `db:"pm25" json:"pm25"`
	Ax             float64 `db:"acc_x" json:"acc_x"`
	Ay             float64 `db:"acc_y" json:"acc_y"`
	Az             float64 `db:"acc_z" json:"acc_z"`
	SignalStrength float64 `db:"received_signal_strength_indication" json:"received_signal_strength_indication"`
	OperationID    *int    `db:"operation_id" json:"operation_id"`
	SecondsOn      float64 `db:"secondsOn" json:"secondsOn"`
}

type BaloraQueryParams struct {
	OperationID int `db:"operation_id"`
}

func GetBaloras(_operationId int) []Balora {
	// query to retrieve data for connected baloras
	query := `
		SELECT
			b.id AS balora_id,
			b.name,
			bm.name AS baloraMaster_name,
			t.*
		FROM
			aiders_balora b
		JOIN
			aiders_baloramaster bm ON b.baloraMaster_id = bm.id
		LEFT JOIN
			aiders_baloratelemetry t ON b.id = t.balora_id AND t.id = (
				SELECT max(id)
				FROM aiders_baloratelemetry
				WHERE balora_id = b.id
			)    
		WHERE
			bm.operation_id = :operation_id
			AND bm.is_connected_with_platform = 1;
	`

	queryParams := BaloraQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()
	// Iterate over the result set and scan into the 'baloras' slice
	var baloras []Balora
	for rows.Next() {
		var balora Balora
		if err := rows.StructScan(&balora); err != nil {
			log.Println(err)
		}

		baloras = append(baloras, balora)
	}

	// Check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return baloras
}
