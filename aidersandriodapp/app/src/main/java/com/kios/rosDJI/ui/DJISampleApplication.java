package com.kios.rosDJI.ui;

import android.annotation.SuppressLint;
import android.app.Application;
import android.content.Context;
import android.util.Log;
import androidx.multidex.MultiDex;
import com.google.firebase.crashlytics.FirebaseCrashlytics;
import com.squareup.otto.Bus;
import com.squareup.otto.ThreadEnforcer;
import dji.sdk.base.BaseProduct;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

/**
 * Main application
 */
@SuppressLint("Registered")
public class DJISampleApplication extends Application {

    public static final String TAG = DJISampleApplication.class.getName();

    private static BaseProduct product;
    private static Bus bus = new Bus(ThreadEnforcer.ANY);
    private static Application app = null;

    /**
     * Gets instance of the specific product connected after the
     * API KEY is successfully validated. Please make sure the
     * API_KEY has been added in the Manifest
     */
    public static synchronized BaseProduct getProductInstance() {
        if (null == product) {
            try {
                FirebaseCrashlytics.getInstance().log("getProductInstance");
                product = DJISDKManager.getInstance().getProduct();
            }catch (Exception e){
                FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                Log.e("DJISampleApplication", e.toString());
            }
        }
        return product;
    }

    public static boolean isAircraftConnected() {
        return getProductInstance() != null && getProductInstance() instanceof Aircraft;
    }

    public static synchronized Aircraft getAircraftInstance() {
        if (!isAircraftConnected()) {
            return null;
        }
        return (Aircraft) getProductInstance();
    }

    public static Application getInstance() {
        return DJISampleApplication.app;
    }

    public static Bus getEventBus() {
        return bus;
    }

    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        MultiDex.install(this);
        com.secneo.sdk.Helper.install(this);
        app = this;
    }

}