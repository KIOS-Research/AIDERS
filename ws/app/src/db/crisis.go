package db

import (
	"encoding/json"
	"log"
)

type CrisisClassification struct {
	ID      int    `db:"id" json:"id"`
	SentUTC string `db:"sent_utc" json:"sent_utc"`
	Sender  string `db:"sender" json:"sender"`
	// ActionType string `db:"action_type" json:"action_type"`

	IncidentID       string          `db:"incident_id" json:"incident_id"`
	IncidentType     string          `db:"incident_type" json:"incident_type"`
	SeverityLevel    string          `db:"severity_level" json:"severity_level"`
	Latitude         float64         `db:"latitude" json:"latitude"`
	Longitude        float64         `db:"longitude" json:"longitude"`
	AreaType         string          `db:"area_type" json:"area_type"`
	TimeOfDay        string          `db:"time_of_day" json:"time_of_day"`
	DetectionSummary json.RawMessage `db:"detections_summary" json:"detections_summary"`
	Description      *string         `db:"description" json:"description"`
	Resolved         int             `db:"resolved" json:"resolved"`
	FalseAlarm       int             `db:"false_alarm" json:"false_alarm"`
	Timestamp        string          `db:"timestamp" json:"timestamp"`
	UpdatedAt        *string         `db:"updated_at" json:"updated_at"`
	UpdatedById      *int            `db:"updated_by_id" json:"updated_by_id"`
	UpdatedBy        *string         `db:"updated_by" json:"updated_by"`
}

// type CrisisClassificationQueryParams struct {
// 	LastID int `db:"id"`
// }

func GetAllCrisisClassificationData(_lastId int, _lastTimestamp string) []CrisisClassification {

	// select only incidents that were sent or updated within the last 2 hours
	query := `
    SELECT
        c.id, c.sent_utc, c.sender, c.area_type, c.time_of_day, c.detections_summary,
        c.incident_id, c.incident_type, c.severity_level,
        c.latitude, c.longitude, c.description, c.resolved, c.false_alarm, c.timestamp, c.updated_at, c.updated_by_id,
        u.username AS updated_by
    FROM
        aiders.aiders_crisisclassification c
    LEFT JOIN
        aiders.aiders_user u ON c.updated_by_id = u.id
    WHERE
        (c.sent_utc > :last_timestamp OR (c.updated_at IS NOT NULL AND c.updated_at > :last_timestamp))
        AND (c.sent_utc >= NOW() - INTERVAL 2 HOUR OR (c.updated_at IS NOT NULL AND c.updated_at >= NOW() - INTERVAL 2 HOUR))
    ORDER BY
        c.timestamp ASC
		`

	queryParams := map[string]interface{}{
		"last_timestamp": _lastTimestamp,
	}

	// Use NamedQuery to execute the query
	rows, err := Conn.NamedQuery(query, queryParams)
	// rows, err := Conn.Queryx(query)
	if err != nil {
		log.Println("Query error:", err)
		return nil
	}
	defer rows.Close()

	var crisisClassifications []CrisisClassification

	for rows.Next() {
		var crisis CrisisClassification
		err := rows.StructScan(&crisis)
		if err != nil {
			log.Println("Struct scan error:", err)
			continue
		}
		crisisClassifications = append(crisisClassifications, crisis)
	}

	if err := rows.Err(); err != nil {
		log.Println("Row iteration error:", err)
		return nil
	}

	return crisisClassifications
}
