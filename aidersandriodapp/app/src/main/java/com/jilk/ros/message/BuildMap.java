package com.jilk.ros.message;

import com.jilk.ros.rosbridge.implementation.JSON;

public class BuildMap extends Message {
//    public Header header;

    public boolean buildmap;
    public int photoid;
    public float altitude;
    public float heading;
    public double latitude;
    public double longitude;

    public BuildMap(){
        buildmap = true;
        photoid = 0;
        altitude=(float)0.0;
        heading=(float)0.0;
        latitude=0.0;
        longitude=0.0;
    }

    public String toJSON() {
        return JSON.toJSON(this);
    }
}
