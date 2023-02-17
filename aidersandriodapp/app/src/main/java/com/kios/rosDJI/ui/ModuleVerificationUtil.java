package com.kios.rosDJI.ui;

import androidx.annotation.Nullable;

import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;

/**
 * Created by dji on 16/1/6.
 */
class ModuleVerificationUtil {

    @Nullable
    static FlightController getFlightController() {
        Aircraft aircraft = DJISampleApplication.getAircraftInstance();
        if (aircraft != null) {
            return aircraft.getFlightController();
        }
        return null;
    }

}
