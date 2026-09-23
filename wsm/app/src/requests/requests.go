package requests

import (
	"bytes"
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"time"
)

var (
	streamCaptureBaseUrl string
	detectionBaseUrl     string
)

// init() runs when the package is initialized
func init() {
	if os.Getenv("VIDEO_AND_CV_REMOTE") == "1" {
		streamCaptureBaseUrl = fmt.Sprintf("http://%s:%s/remote/lsc", os.Getenv("NGINX_HOST"), os.Getenv("NGINX_PORT"))
		detectionBaseUrl = fmt.Sprintf("http://%s:%s/remote/cv", os.Getenv("NGINX_HOST"), os.Getenv("NGINX_PORT"))
	} else {
		streamCaptureBaseUrl = fmt.Sprintf("http://%s:%s/lsc", os.Getenv("NGINX_HOST"), os.Getenv("NGINX_PORT"))
		detectionBaseUrl = fmt.Sprintf("http://%s:%s/cv", os.Getenv("NGINX_HOST"), os.Getenv("NGINX_PORT"))
	}
}

func makePostRequestWithRetries(url string, payload interface{}) {
	go postRequestThread(url, payload)
}

func postRequestThread(url string, payload interface{}) {
	jsonPayload, err := json.Marshal(payload)
	if err != nil {
		fmt.Println("Error marshaling payload:", err)
		return
	}

	retries := 1
	for {
		req, err := http.NewRequest("POST", url, bytes.NewBuffer(jsonPayload))
		if err != nil {
			fmt.Println("Error creating request:", err)
			return
		}

		// Set headers for the request
		req.Header.Set("Content-Type", "application/json")

		// Send the request
		client := &http.Client{}
		response, err := client.Do(req)
		if err != nil {
			fmt.Println("Error sending request:", err)
			return
		}
		defer response.Body.Close()

		if response.StatusCode == 200 {
			break // exit the loop
		}
		if retries == 3 {
			fmt.Printf("Request '%s' failed. Status code: %d\n", url, response.StatusCode)
			break
		}
		retries++
		time.Sleep(1 * time.Second) // wait before retrying
	}
}

func StartDroneLiveStreamCapture(droneId int, droneName string) {
	url := fmt.Sprintf("%s/startDroneStreamCapture", streamCaptureBaseUrl)
	payload := map[string]interface{}{
		"droneId":   droneId,
		"droneName": droneName,
	}
	makePostRequestWithRetries(url, payload)
}

func StopDroneDetector(droneId int, droneName string) {
	url := fmt.Sprintf("%s/stopDetection", detectionBaseUrl)
	payload := map[string]interface{}{
		"droneId":   droneId,
		"droneName": droneName,
	}
	makePostRequestWithRetries(url, payload)
}
