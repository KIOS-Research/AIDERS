package com.kios.rosDJI.Helpers;

/**
 * Created by the0s on 18/01/2018.
 */

public class ButtonEvent {
    public boolean msg;
    public String name;


    public ButtonEvent(String name, boolean msg) {
        this.name = name;
        this.msg = msg;
    }
    }
