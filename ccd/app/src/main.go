package main

import (
	"database/sql"
	"fmt"
	"log"
	"os"
	"time"

	_ "github.com/go-sql-driver/mysql"
)

const disconnectThreshold = 15 // in seconds
const checkFrequency = 15      // in seconds

// Create a map to store the connection status of clients
var clientPreviousConnectedState = make(map[int]bool)

var dbConn *sql.DB

/*
 *
 * Starts a micro-service that marks clients as disconnected based on the timestamp of their most recent telemetry
 *
 */
func main() {
	dbInit() // Connect to the database
	for {
		fmt.Println("---")
		checkForDisconnectedClients("aiders_drone", "aiders_telemetry", "drone_id")
		checkForDisconnectedClients("aiders_device", "aiders_devicetelemetry", "device_id")
		time.Sleep(checkFrequency * time.Second) // sleep
	}
}

// retrieves connected clients and checks how long ago they last sent telemetry
// if the time threshold is passed, the client is marked as disconnected
func checkForDisconnectedClients(_clientsTable, _telemetryTable, _telemetryField string) {
	fmt.Println(" ")
	fmt.Printf("Checking for disconnected clients in: %s \n", _clientsTable)
	connectedIds, err := getConnectedClients(_clientsTable) // get all connected clients
	if err != nil {
		log.Println(err)
	}
	for _, id := range connectedIds { // loop connected clients
		telemetryData, err := getLatestTelemetryForClient(id, _telemetryTable, _telemetryField) // get the time of the last telemetry
		if err != nil {
			log.Printf("Error retrieving telemetry data for ID %d: %v", id, err)
		} else {
			secondsSinceLastTelemetry, err := timeAgoInSeconds(telemetryData) // check how long it's been since last telemetry
			if err != nil {
				log.Printf("Error calculating time difference for ID %d: %v", id, err)
			} else {
				fmt.Printf("%s id %d: received %d seconds ago\n", _clientsTable, id, secondsSinceLastTelemetry)
				// TODO: publish a message to ROS /droneIds or /deviceIds (?)
				if secondsSinceLastTelemetry > disconnectThreshold {
					if !clientPreviousConnectedState[id] { // if the client has been disconnected twice
						fmt.Print("*** ")
						err := markClientAsDisconnected(id, _clientsTable) // mark client as disconnected
						if err != nil {
							log.Printf("Error updating is_connected for ID %d: %v", id, err)
						} else {
							fmt.Printf("%s id %d has been disconnected.\n", _clientsTable, id)
						}
					}
					clientPreviousConnectedState[id] = false
				} else {
					clientPreviousConnectedState[id] = true // reset the state  of client is connected
				}
			}
		}
	}
	fmt.Println(" ")
}

// retrieve all the clients from a specific table that are connected to the platform
func getConnectedClients(_tableName string) ([]int, error) {
	query := fmt.Sprintf("SELECT id FROM %s WHERE is_connected_with_platform = 1", _tableName)

	rows, err := dbConn.Query(query)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var ids []int
	for rows.Next() {
		var id int
		err := rows.Scan(&id)
		if err != nil {
			return nil, err
		}
		ids = append(ids, id)
	}

	return ids, nil
}

// retrieve the timestamp of the most recent telemetry for a specific client
func getLatestTelemetryForClient(_id int, _tableName string, _fieldName string) (string, error) {
	var lastTelemetryTimestamp string
	query := fmt.Sprintf("SELECT time FROM %s WHERE %s = ? ORDER BY time DESC LIMIT 1", _tableName, _fieldName)
	err := dbConn.QueryRow(query, _id).Scan(&lastTelemetryTimestamp)
	if err != nil {
		return "", err
	}
	return lastTelemetryTimestamp, nil
}

// mark a client as disconnected in the database
func markClientAsDisconnected(_id int, _tableName string) error {
	query := fmt.Sprintf("UPDATE %s SET is_connected_with_platform = 0 WHERE id = ?", _tableName)
	_, err := dbConn.Exec(query, _id)
	return err
}

// compare a timestamp to the current time and return the difference in seconds
func timeAgoInSeconds(_timestamp string) (int, error) {
	parsedTime, err := time.Parse("2006-01-02 15:04:05.999999", _timestamp)
	if err != nil {
		return 0, err
	}
	duration := time.Since(parsedTime)
	return int(duration.Seconds()), nil
}

// connect to the database
func dbInit() {
	dbHost := "127.0.0.1"
	dbPort := "3306"
	dbName := os.Getenv("SQL_DATABASE")
	dbUser := os.Getenv("SQL_USER")
	dbPassword := os.Getenv("SQL_PASSWORD")

	fmt.Println("Connecting to DB: " + dbName)
	var dbConnError error
	dbConn, dbConnError = sql.Open("mysql", dbUser+":"+dbPassword+"@tcp("+dbHost+":"+dbPort+")/"+dbName+"")
	if dbConnError != nil {
		panic(dbConnError.Error())
	}
	fmt.Println("DB connection successful.")
}
