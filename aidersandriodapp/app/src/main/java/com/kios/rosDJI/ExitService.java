package com.kios.rosDJI;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import static com.kios.rosDJI.ui.ConnectDroneActivity.jMsg;

public class ExitService extends Service {

    public IBinder onBind(Intent intent){

        return null;
    }

    @Override
    public void onTaskRemoved(Intent rootIntent) {

        Log.d("ExitService","onTaskRemoved");


        stopSelf();
    }
}
