#!/bin/bash
echo "-- Running entrypoint.adsb.sh --"

# Clear any previous restart flags and logs
rm -f /tmp/restart_needed
rm -f /tmp/dump1090.log
touch /tmp/dump1090.log

# wait for mysql (optional - only if needed)
# while ! echo exit | nc -z $DB_HOST $DB_PORT > /dev/null 2>&1 ; do
# 	sleep 1
# done

# Check for RTL-SDR devices
echo -e "\n** Checking for RTL-SDR devices **"
if [ -e /dev/bus/usb ]; then
    echo "✓ USB bus is accessible"
    ls -la /dev/bus/usb/ 2>/dev/null || echo "⚠ Cannot list USB devices"
else
    echo "⚠ WARNING: /dev/bus/usb not found - USB devices not accessible"
fi

# Test RTL-SDR detection
echo -e "\nTesting RTL-SDR detection..."
rtl_test -t 2>&1 | head -20

echo -e "\n============================================"
echo "Starting ADS-B Container Services"
echo "============================================"
echo "This container will:"
echo "1. Monitor for RTL-SDR antenna connection"
echo "2. Auto-start dump1090 when antenna is detected"
echo "3. Auto-restart dump1090 if it crashes"
echo "4. Keep the Python scanner running continuously"
echo "============================================"

# Function to start dump1090
start_dump1090() {
    echo -e "\n[$(date '+%H:%M:%S')] Attempting to start dump1090..."
    /opt/dump1090/dump1090 --net 2>&1 | tee /tmp/dump1090.log &
    DUMP1090_PID=$!
    sleep 3
    
    if ps -p $DUMP1090_PID > /dev/null 2>&1; then
        echo "[$(date '+%H:%M:%S')] ✓ dump1090 is running (PID: $DUMP1090_PID)"
        return 0
    else
        echo "[$(date '+%H:%M:%S')] ⚠ dump1090 failed to start (no antenna detected)"
        return 1
    fi
}

# Start dump1090 monitor in background
(
    echo -e "\n** Starting dump1090 Monitor **"
    RETRY_DELAY=10
    
    while true; do
        # Check if dump1090 is running
        if ! pgrep -f "dump1090.*--net" > /dev/null; then
            echo "[$(date '+%H:%M:%S')] dump1090 is not running, attempting to start..."
            
            if start_dump1090; then
                echo "[$(date '+%H:%M:%S')] ✓ dump1090 started successfully"
                # Check for listening ports
                sleep 2
                if netstat -tuln | grep -q ':30003'; then
                    echo "[$(date '+%H:%M:%S')] ✓ dump1090 is listening on port 30003"
                fi
            else
                echo "[$(date '+%H:%M:%S')] ⚠ No RTL-SDR antenna detected"
                echo "[$(date '+%H:%M:%S')]   Waiting ${RETRY_DELAY}s before retry..."
                echo "[$(date '+%H:%M:%S')]   (Plug in antenna anytime - will auto-detect)"
            fi
        fi
        
        sleep $RETRY_DELAY
    done
) &

MONITOR_PID=$!
echo "dump1090 monitor started with PID: $MONITOR_PID"

# Give monitor a chance to start dump1090
sleep 5

# Start USB error monitor in background (will restart container on antenna disconnect)
(
    echo -e "\n** Starting USB Error Monitor **"
    echo "This will restart the container if antenna is disconnected"
    CHECK_INTERVAL=3
    ERROR_THRESHOLD=5
    ENTRYPOINT_PID=$$
    
    while true; do
        sleep $CHECK_INTERVAL
        
        if [ -f /tmp/dump1090.log ]; then
            # Count recent USB transfer errors
            ERROR_COUNT=$(tail -30 /tmp/dump1090.log 2>/dev/null | grep -c "cb transfer status")
            
            if [ "$ERROR_COUNT" -ge "$ERROR_THRESHOLD" ]; then
                echo ""
                echo "============================================"
                echo "[$(date '+%H:%M:%S')] ⚠ ANTENNA DISCONNECTED!"
                echo "USB transfer errors detected: $ERROR_COUNT"
                echo "Forcing container restart..."
                echo "Reconnect antenna and container will auto-start"
                echo "============================================"
                
                # Create flag file to signal main process to exit
                touch /tmp/restart_needed
                
                # Kill all child processes
                pkill -9 -f dump1090
                pkill -9 -f "python.*main.py"
                
                # Kill the tail process that keeps container alive
                pkill -9 -f "tail -f"
                
                exit 0
            fi
        fi
    done
) &

USB_MONITOR_PID=$!
echo "USB error monitor started with PID: $USB_MONITOR_PID"

# run the python application regardless of dump1090 status
echo -e "\n** Running ADS-B Scanner **"
python -u /app/main.py &

PYTHON_PID=$!

# start the HTTP server (if needed in future)
# echo -e "\n** Starting the HTTP server **"
# cd app && gunicorn -w 5 -b 0.0.0.0:8760 server:app &

echo "-- END entrypoint.adsb.sh --"

# Monitor for restart flag instead of tail -f
while true; do
    if [ -f /tmp/restart_needed ]; then
        echo "Restart flag detected - exiting container"
        exit 1
    fi
    sleep 2
done
