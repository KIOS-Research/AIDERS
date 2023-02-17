package com.jilk.ros.message;

import com.jilk.ros.rosbridge.implementation.JSON;

/**
 * Created by aanast01 on 21-06-19.
 */
@MessageType(string = "kios/Telemetry")
public class TelemetryMsg extends Message {
//    public Header header;

    public int rostime_secs;
    public int rostime_nsecs;
    public int flightTimeSecs;
    public int batteryThreashold;
    public int batteryPercentage;
    public int gpsSignal;
    public int satelliteNumber;
    public float altitude;
    public float heading;
    public double velocity;
    public double latitude;
    public double longitude;
    public double homeLatitude;
    public double homeLongitude;
    public String droneState;
    public float gimbalAngle;
    public int serialVersionUID;

    public TelemetryMsg(){
        rostime_secs=0;
        rostime_nsecs=0;
        flightTimeSecs=0;
        batteryThreashold=0;
        batteryPercentage=0;
        gpsSignal=0;
        satelliteNumber=0;
        altitude=(float)0.0;
        heading=(float)0.0;
        velocity=0.0;
        latitude=0.0;
        longitude=0.0;
        homeLatitude=0.0;
        homeLongitude=0.0;
        droneState="";
        gimbalAngle=(float)0.0;
        serialVersionUID=0;
    }

    public String toJSON() {
        return JSON.toJSON(this);
    }

}
