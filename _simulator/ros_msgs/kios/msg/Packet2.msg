int64 seq
string serialVersionUID

float32 heading
float64 velocity
float64 longitude
float64 latitude
float32 altitude

string date_time
uint32 rostime_secs
uint32 flightTimeSecs

uint32 batteryThreshold
uint32 batteryPercentage

uint32 cpuTemp
uint32 gpuTemp

uint32 gpsSignal
uint32 satelliteNumber
float64 homeLatitude
float64 homeLongitude

# Drone Orientation
float64 d_quat_x     # rads
float64 d_quat_y   # rads
float64 d_quat_z    # rads
float64 d_quat_w    # rads
# Gimbal Orientation
float64 g_yaw     # rads
float64 g_pitch   # rads
float64 g_roll    # rads
