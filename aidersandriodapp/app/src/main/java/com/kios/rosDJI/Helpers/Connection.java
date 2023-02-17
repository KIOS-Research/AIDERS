package com.kios.rosDJI.Helpers;

import android.app.Activity;
import android.net.wifi.WifiManager;

import static android.content.Context.WIFI_SERVICE;

public class Connection {
    Activity activity;
    public Connection(Activity activity){
        this.activity=activity;
    }

    public boolean connectedToNework(){
        WifiManager wifiManager = (WifiManager) activity.getApplicationContext().getSystemService(WIFI_SERVICE);

        return  (wifiManager.getConnectionInfo().getIpAddress()!=0);
    }
}
