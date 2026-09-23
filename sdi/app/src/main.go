package main

import (
	"sdi/src/db"
	"sdi/src/serial"
)

func main() {
	db.Init() // initialize the database connection
	serial.Run("/dev/loraReceiver", 115200)
}
