package com.dji;

import com.kios.rosDJI.ui.ConnectDroneActivity;

public class FlightController {
    public static double getYaw() {

        try {
            return ConnectDroneActivity.getAircraft().getFlightController().getState().getAttitude().yaw;

        }
        catch (NullPointerException e){
            return 0;
        }

    }

}
