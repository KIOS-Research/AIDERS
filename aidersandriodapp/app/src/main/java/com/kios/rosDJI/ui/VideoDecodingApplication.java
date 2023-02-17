package com.kios.rosDJI.ui;

import android.annotation.SuppressLint;
import android.app.Application;
import android.content.Context;
import com.secneo.sdk.Helper;
import dji.sdk.base.BaseProduct;
import dji.sdk.sdkmanager.DJISDKManager;

@SuppressLint("Registered")
public class VideoDecodingApplication extends Application {

    private static BaseProduct mProduct;

    public static synchronized BaseProduct getProductInstance() {
        if (null == mProduct) {
            mProduct = DJISDKManager.getInstance().getProduct();
        }
        return mProduct;
    }

    @Override
    protected void attachBaseContext(Context base) {
        super.attachBaseContext(base);
        Helper.install(VideoDecodingApplication.this);
    }
}
