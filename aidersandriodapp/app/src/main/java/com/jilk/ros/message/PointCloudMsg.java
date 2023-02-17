package com.jilk.ros.message;

import com.jilk.ros.rosbridge.implementation.JSON;

/**
 * Created by xtheod01 on 18-10-22.
 */
@MessageType(string = "kios/PointCloud")
public class PointCloudMsg extends Message {

    public float x;
    public float y;
    public float z;
    public float red;
    public float green;
    public float blue;

//    public float getX() {
//        return x;
//    }
//
//    public void setX(float x) {
//        this.x = x;
//    }
//
//    public float getY() {
//        return y;
//    }
//
//    public void setY(float y) {
//        this.y = y;
//    }
//
//    public float getZ() {
//        return z;
//    }
//
//    public void setZ(float z) {
//        this.z = z;
//    }
//
//    public float getRed() {
//        return red;
//    }
//
//    public void setRed(float red) {
//        this.red = red;
//    }
//
//    public float getGreen() {
//        return green;
//    }
//
//    public void setGreen(float green) {
//        this.green = green;
//    }
//
//    public float getBlue() {
//        return blue;
//    }
//
//    public void setBlue(float blue) {
//        this.blue = blue;
//    }

    public PointCloudMsg(){
        x=(float)0.0;
        y=(float)0.0;
        z=(float)0.0;
        red=(float)0.0;
        green=(float)0.0;
        blue=(float)0.0;

    }

    public String toJSON() {
        return JSON.toJSON(this);
    }

}
