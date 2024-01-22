package db

import (
	"log"
)

type LidarPoint struct {
	ID    int     `db:"id" json:"id"`
	X     float64 `db:"x" json:"x"`
	Y     float64 `db:"y" json:"y"`
	Z     float64 `db:"z" json:"z"`
	Red   float64 `db:"red" json:"red"`
	Green float64 `db:"green" json:"green"`
	Blue  float64 `db:"blue" json:"blue"`
}

type LidarPointQueryParams struct {
	SessionId      int `db:"session_id"`
	LatestPointId  int `db:"latest_point_id"`
	NumberOfPoints int `db:"number_of_points"`
}

func GetLidarPointBySessionIdAndLatestId(_sessionId int, _latestId int, _numberOfPoints int) []LidarPoint {
	query := `
		SELECT
			id, x, y, z, red, green, blue
		FROM
			aiders.aiders_lidarpoint
		WHERE
			lidar_point_session_id = :session_id
		AND
			id > :latest_point_id
		LIMIT :number_of_points`

	queryParams := LidarPointQueryParams{SessionId: _sessionId, LatestPointId: _latestId, NumberOfPoints: _numberOfPoints}
	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Fatal(err)
	}
	defer rows.Close()

	var lidarPoints []LidarPoint
	for rows.Next() {
		var lidarPoint LidarPoint
		if err := rows.StructScan(&lidarPoint); err != nil {
			log.Fatal(err)
		}
		lidarPoints = append(lidarPoints, lidarPoint)
	}

	// check for errors from iterating over rows
	if err := rows.Err(); err != nil {
		log.Fatal(err)
	}

	return lidarPoints
}

type LidarCoordinates struct {
	Latitude  float64 `json:"latitude" db:"lat"`
	Longitude float64 `json:"longitude" db:"lon"`
}

type LidarCoordinatesQueryParams struct {
	SessionId int `db:"session_id"`
}

func GetAllDataLidarOriginCoordinatesByLidarSessionId(_sessionId int) LidarCoordinates {
	query := `
	SELECT lat, lon
	FROM aiders_telemetry
	JOIN aiders_lidarpoint ON aiders_telemetry.id = aiders_lidarpoint.telemetry_id
	WHERE aiders_lidarpoint.lidar_point_session_id = :session_id
	LIMIT 1
	`
	queryParams := LidarCoordinatesQueryParams{SessionId: _sessionId}
	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	if err != nil {
		log.Fatal(err)
	}
	defer rows.Close()
	var coordinates LidarCoordinates

	if rows.Next() {
		err := rows.Scan(&coordinates.Latitude, &coordinates.Longitude)
		if err != nil {
			log.Fatal(err)
		}
	}

	return coordinates
}
