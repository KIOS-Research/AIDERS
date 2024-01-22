package db

import (
	"fmt"
	"os"

	_ "github.com/go-sql-driver/mysql"
	"github.com/jmoiron/sqlx"
)

var Conn *sqlx.DB

// connect to the database
func Init() {
	dbHost := "127.0.0.1"
	dbPort := "3306"
	dbName := os.Getenv("SQL_DATABASE")
	dbUser := os.Getenv("SQL_USER")
	dbPassword := os.Getenv("SQL_PASSWORD")

	fmt.Println("Connecting to DB: " + dbName)
	var dbConnError error
	Conn, dbConnError = sqlx.Open("mysql", dbUser+":"+dbPassword+"@tcp("+dbHost+":"+dbPort+")/"+dbName+"")
	if dbConnError != nil {
		panic(dbConnError.Error())
	}
	fmt.Println("DB connection successful.")
}
