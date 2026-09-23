package db

import (
	"log"
	//"database/sql"
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
	Nox            float64 `db:"nox" json:"nox"`
	Voc            float64 `db:"voc" json:"voc"`
	Temp           float64 `db:"temp" json:"temp"`
	Humidity       float64 `db:"humidity" json:"humidity"`
	Ax             float64 `db:"acc_x" json:"acc_x"`
	Ay             float64 `db:"acc_y" json:"acc_y"`
	Az             float64 `db:"acc_z" json:"acc_z"`
	SignalStrength float64 `db:"received_signal_strength_indication" json:"received_signal_strength_indication"`
	OperationID    *int    `db:"operation_id" json:"operation_id"`
	SecondsOn      float64 `db:"secondsOn" json:"secondsOn"`

	//running averages

	// Pm1RunningAvg       float64 `db:"pm1_avg" json:"pm1_avg"`
	// Pm25RunningAvg      float64 `db:"pm25_avg" json:"pm25_avg"`
	// NoxRunningAvg       float64 `db:"nox_avg" json:"nox_avg"`
	// VocRunningAvg       float64 `db:"voc_avg" json:"voc_avg"`
	// TempRunningAvg      float64 `db:"temp_avg" json:"temp_avg"`
	// HumidityRunningAvg  float64 `db:"humidity_avg" json:"humidity_avg"`
}

type BaloraQueryParams struct {
	OperationID int `db:"operation_id"`
}

type AverageResult struct {
	Average float64 `db:"average"`
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

	// query := `
	// SELECT
	// 	b.id AS balora_id,
	// 	b.name,
	// 	bm.name AS baloraMaster_name,
	// 	t.*,
	// 	(SELECT AVG(pm25) 
	// 	 FROM (SELECT pm25 FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS pm25_avg,
	// 	(SELECT AVG(pm1) 
	// 	 FROM (SELECT pm1 FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS pm1_avg,
	// 	(SELECT AVG(nox) 
	// 	 FROM (SELECT nox FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS nox_avg,
	// 	(SELECT AVG(voc) 
	// 	 FROM (SELECT voc FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS voc_avg,
	// 	(SELECT AVG(temp) 
	// 	 FROM (SELECT temp FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS temp_avg,
	// 	(SELECT AVG(humidity) 
	// 	 FROM (SELECT humidity FROM aiders_baloratelemetry WHERE balora_id = b.id ORDER BY time DESC LIMIT 10) sub) AS humidity_avg
	// FROM
	// 	aiders_balora b
	// JOIN
	// 	aiders_baloramaster bm ON b.baloraMaster_id = bm.id
	// LEFT JOIN
	// 	aiders_baloratelemetry t ON b.id = t.balora_id AND t.id = (
	// 		SELECT max(id)
	// 		FROM aiders_baloratelemetry
	// 		WHERE balora_id = b.id
	// 	)
	// WHERE
	// 	bm.operation_id = :operation_id
	// 	AND bm.is_connected_with_platform = 1;
	// `


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


// CalculateRunningAverage calculates the running average of a given column from the last 10 values in the database
// func CalculateRunningAverage(column string) (float64, error) {
// 	query := `
// 		SELECT AVG(` + column + `) AS average
// 		FROM (
// 			SELECT ` + column + `
// 			FROM aiders_baloratelemetry
// 			ORDER BY time DESC
// 			LIMIT 10
// 		) AS last_ten_values;
// 	`

// 	var result AverageResult

// 	err := Conn.Get(&result.Average, query) // Use Conn.Get if you're using sqlx
// 	if err != nil {
// 		if err == sql.ErrNoRows {
// 			log.Printf("No data found to calculate average for %s.\n", column)
// 			return 0, nil // Return 0 if no rows are found
// 		}
// 		log.Printf("Error calculating average for %s: %v\n", column, err)
// 		return 0, err
// 	}

// 	return result.Average, nil
// }

// func GetRunningAveragePM1() (float64, error) {
// 	return CalculateRunningAverage("pm1")
// }

// func GetRunningAveragePM25() (float64, error) {
//     return CalculateRunningAverage("pm25")
// }

// func GetRunningAverageNOx() (float64, error) {
// 	return CalculateRunningAverage("nox")
// }

// func GetRunningAverageVOC() (float64, error) {
// 	return CalculateRunningAverage("voc")
// }
