package db

import (
	"log"
)

// represents the structure of drone Remote ID devices
type DroneRid struct {
	ID          int      `db:"id" json:"id"`
	Address     string   `db:"address" json:"address"`
	Name        string   `db:"name" json:"name"`
	BasicID     string   `db:"basic_id" json:"basic_id"`
	OperatorID  string   `db:"operator_id" json:"operator_id"`
	Lat         *float64 `db:"lat" json:"lat"`
	Lon         *float64 `db:"lon" json:"lon"`
	AltitudeM   *float64 `db:"altitude_m" json:"altitude_m"`
	SpeedMS     *float64 `db:"speed_m_s" json:"speed_m_s"`
	RSSI        *int     `db:"rssi" json:"rssi"`
	LastSeen    string   `db:"last_seen" json:"last_seen"`
}

type DroneRidQueryParams struct {
	OperationID int `db:"operation_id"`
}

// GetDroneRids retrieves all drone Remote ID devices
// that have been updated within the last 60 seconds
func GetDroneRids(_operationId int) []DroneRid {
	query := `
	SELECT
		dr.id,
		dr.address,
		dr.name,
		dr.basic_id,
		dr.operator_id,
		ST_Y(dr.location) as lat,
		ST_X(dr.location) as lon,
		dr.altitude_m,
		dr.speed_m_s,
		dr.rssi,
		dr.last_seen
	FROM
		aiders_dronerid dr
	WHERE
		dr.last_seen >= DATE_SUB(NOW(), INTERVAL 60 SECOND)
	ORDER BY
		dr.last_seen DESC
	`

	queryParams := DroneRidQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	// Iterate over the result set and scan into the 'droneRids' slice
	var droneRids []DroneRid
	for rows.Next() {
		var rid DroneRid
		if err := rows.StructScan(&rid); err != nil {
			log.Println(err)
		}
		droneRids = append(droneRids, rid)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return droneRids
}
