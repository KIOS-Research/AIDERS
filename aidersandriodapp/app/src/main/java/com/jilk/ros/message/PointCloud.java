package com.jilk.ros.message;

import com.jilk.ros.entities.Point;
import com.jilk.ros.rosbridge.implementation.JSON;

import java.util.LinkedList;
import java.util.List;

@MessageType(string = "kios/pointCloud")
public class PointCloud  extends Message {

  public   List<Point> points;
    public PointCloud(){
      points=new LinkedList<>();

    }
    public String toJSON() {
        return JSON.toJSON(this);
    }

}
