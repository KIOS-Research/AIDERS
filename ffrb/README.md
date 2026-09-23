# FFmpeg Rebroadcast API

This Dockerized application provides an API to rebroadcast RTMP streams using FFmpeg.

## API Endpoint

### POST `/rebroadcast`

**Request Body:**
```json
{
  "input_url": "rtmp://source-url",
  "output_url": "rtmp://destination-url"
}
```

**Response:**
- `200 OK`: Rebroadcast started successfully.
- `400 Bad Request`: Missing input or output URL.
- `500 Internal Server Error`: Error starting the rebroadcast.

## How to Use

1. Build the Docker image:
   ```bash
   docker build -t ffmpeg-rebroadcast .
   ```

2. Run the container:
   ```bash
   docker run -p 5000:5000 ffmpeg-rebroadcast
   ```

3. Use the API to start rebroadcasting:
   ```bash
   curl -X POST -H "Content-Type: application/json" \
        -d '{"input_url": "rtmp://source-url", "output_url": "rtmp://destination-url"}' \
        http://localhost:5000/rebroadcast
   ```


# TESTING

ffmpeg -hwaccel auto -re -i /dev/video0 -c:v libx264 -preset veryfast -bf 0 -f flv rtmp://192.168.0.17/live/tete

curl -X POST -H "Content-Type: application/json" \
     -d '{"input_url": "rtmp://192.168.0.17/live/tete", "output_url": "rtmp://192.168.0.10/live/SIM_0"}' \
     http://localhost:5000/rebroadcast

