package com.jilk.ros.entities;

public class Point {

   public float x,y,z;
    public Point(float x,float y,float z){
        this.x=x;
        this.y=y;
        this.z=z;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }
}
