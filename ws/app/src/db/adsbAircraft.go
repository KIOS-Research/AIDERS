package db

import (
	"log"
)

// represents the structure of ADS-B aircraft
type AdsbAircraft struct {
	ID              int      `db:"id" json:"id"`
	Icao24          string   `db:"icao24" json:"icao24"`
	Callsign        string   `db:"callsign" json:"callsign"`
	Lat             *float64 `db:"latitude" json:"lat"`
	Lon             *float64 `db:"longitude" json:"lon"`
	AltitudeFt      *int     `db:"altitude_ft" json:"altitude_ft"`
	GroundSpeedKts  *int     `db:"ground_speed_kts" json:"ground_speed_kts"`
	TrackDeg        *float64 `db:"track_deg" json:"track_deg"`
	VerticalRateFpm *int     `db:"vertical_rate_fpm" json:"vertical_rate_fpm"`
	Squawk          string   `db:"squawk" json:"squawk"`
	Emergency       bool     `db:"emergency" json:"emergency"`
	OnGround        bool     `db:"on_ground" json:"on_ground"`
	LastSeen        string   `db:"last_seen" json:"last_seen"`
}

type AdsbAircraftQueryParams struct {
	OperationID int `db:"operation_id"`
}

// GetAdsbAircraft retrieves all ADS-B aircraft
// that have been updated within the last 60 seconds
func GetAdsbAircraft(_operationId int) []AdsbAircraft {
	query := `
	SELECT
		a.id,
		a.icao24,
		a.callsign,
		a.latitude,
		a.longitude,
		a.altitude_ft,
		a.ground_speed_kts,
		a.track_deg,
		a.vertical_rate_fpm,
		a.squawk,
		a.emergency,
		a.on_ground,
		a.last_seen
	FROM
		aiders_adsbaircraft a
	WHERE
		a.last_seen >= DATE_SUB(NOW(), INTERVAL 60 SECOND)
	ORDER BY
		a.last_seen DESC
	`

	queryParams := AdsbAircraftQueryParams{OperationID: _operationId}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Println(err)
	}
	defer rows.Close()

	// Iterate over the result set and scan into the 'aircraft' slice
	var aircraft []AdsbAircraft
	for rows.Next() {
		var ac AdsbAircraft
		if err := rows.StructScan(&ac); err != nil {
			log.Println(err)
		}
		aircraft = append(aircraft, ac)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Println(err)
	}

	return aircraft
}
