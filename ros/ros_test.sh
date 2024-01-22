# loraIds
rostopic pub /loraIds std_msgs/String "data: 'AAG,True'" &

sleep 2

# telemetry
# message = ID,time,lat,long,pm1,pm2.5,accx,accy,accz,rssi
rostopic pub AAG/TelemetryLora std_msgs/String "data: 'AAG_Sl,1690457530.5574203,00.000000,00.000000,10.00,20.00,0.32,-2.42,9.28,-76'" &

# old message = ID,time,lat,long,pm1,pm2.5,rssi,signalToNoise
# rostopic pub AAG/loremetry std_msgs/String "data: 'AAG_Sl,1690457530.5574203,33.23123,35.132131,1111,2222,7777,8888'"

sleep 2

# monitoring
# control_msg = ID + "," + battery + "," + cpu + "," + memory + "," + rssi;
rostopic pub AAG/MonitorLora std_msgs/String "data: 'AAG_Sl,69,77,88,-103'"

# disconnect
rostopic pub /loraIds std_msgs/String "data: 'AAG,False'" &


# disconnect SIM_Alpha
rostopic pub /droneIds std_msgs/String "data: '{\"DroneName\":\"SIM_Alpha\",\"Model\":\"DJI Phantom 4 Pro\",\"DroneIp\":\"127.0.0.1\",\"Connected\":\"False\"}'" &
