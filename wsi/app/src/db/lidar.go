package db

import (
	"log"
	"time"
    // "fmt"
    // "strings"
)


type LidarPoint struct {
	//ID        int     `json:"id"`
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Z         float64 `json:"z"`
	Red       float64 `json:"red"`
	Green     float64 `json:"green"`
	Blue      float64 `json:"blue"`
	Intensity int     `json:"intensity"`
}


//inserts drone pointcloud idar points into the database

func SaveDroneLidarPointDataInBatches(lidarPoints []LidarPoint, telemetryID int, lidarSessionID int) error {
    log.Println("Session ID:", lidarSessionID)
    query := `
    INSERT INTO aiders_lidarpoint (x, y, z, intensity, red, green, blue, telemetry_id, lidar_point_session_id, time)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`

    // Begin a transaction
    tx, err := Conn.Begin()
    if err != nil {
        log.Printf("Error starting transaction: %v\n", err)
        return err
    }

    // Make the prepared statement
    stmt, err := tx.Prepare(query)
    if err != nil {
        log.Printf("Error preparing statement: %v\n", err)
        return err
    }

    defer stmt.Close()

    for _, lidarPoint := range lidarPoints {
        _, err := stmt.Exec(lidarPoint.X, lidarPoint.Y, lidarPoint.Z, lidarPoint.Intensity,
            lidarPoint.Red, lidarPoint.Green, lidarPoint.Blue, telemetryID, lidarSessionID, time.Now().UTC())
        if err != nil {
            log.Printf("Error executing prepared statement: %v\n", err)
            tx.Rollback()
            return err
        }
    }

    // Commit the transaction
    if err := tx.Commit(); err != nil {
        log.Printf("Error committing transaction: %v\n", err)
        return err
    }

    log.Println("LIDAR data saved successfully")
    return nil
}


//inserts drone pointcloud idar points into the database
// func SaveDroneLidarPointDataInBatches(lidarPoints []LidarPoint, telemetryID int, lidarSessionID int) error {
//     log.Println("Session ID:", lidarSessionID)
//     query := `
//         INSERT INTO aiders_lidarpoint (x, y, z, intensity, red, green, blue, telemetry_id, lidar_point_session_id, time)
//         VALUES `

//     var lidarValueArgs []interface{}
//     placeholders := make([]string, len(lidarPoints))

//     for i, lidarPoint := range lidarPoints {
//         lidarValueArgs = append(lidarValueArgs, lidarPoint.X, lidarPoint.Y, lidarPoint.Z, lidarPoint.Intensity,
//             lidarPoint.Red, lidarPoint.Green, lidarPoint.Blue, telemetryID, lidarSessionID, time.Now().UTC())
//         placeholders[i] = "(?, ?, ?, ?, ?, ?, ?, ?, ?, ?)"
//     }

//     fullQuery := query + fmt.Sprintf("%s", strings.Join(placeholders, ", "))

//     stmt, err := Conn.Prepare(fullQuery)
//     if err != nil {
//         log.Printf("Error preparing statement: %v\n", err)
//         return err
//     }
//     defer stmt.Close()

//     _, err = stmt.Exec(lidarValueArgs...)
//     if err != nil {
//         log.Printf("Error executing batch query: %v\n", err)
//         return err
//     }

//     log.Println("LIDAR data saved successfully")
//     return nil
// }

