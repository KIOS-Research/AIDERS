package com.kios.rosDJI;

import android.app.Application;
import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Toast;

import com.google.firebase.crashlytics.FirebaseCrashlytics;
import com.jilk.ros.ROSClient;
import com.jilk.ros.rosbridge.ROSBridgeClient;
import com.kios.rosDJI.Helpers.ButtonEvent;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import com.kios.rosDJI.ui.MainActivity;
import com.secneo.sdk.Helper;

import dji.sdk.base.BaseProduct;
import dji.sdk.sdkmanager.DJISDKManager;

import static android.content.ContentValues.TAG;
import static com.kios.rosDJI.ui.ConnectDroneActivity.getActivity;
import static com.kios.rosDJI.ui.MainActivity.getRemoteLogger;
import static com.kios.rosDJI.ui.MainActivity.getStackTrace;

/**
 * Created by xxhong on 16-11-21.
 */

public class RCApplication extends Application {
    private ROSBridgeClient client;
    private static BaseProduct mProduct;
    boolean calledBefore = false;
    boolean tryReconect = true;

    @Override
    public void onCreate() {
        super.onCreate();


        // throw  new IllegalArgumentException("Testing");
        // 00222D6D-6D7C-448F-BA85-E80E965B96C3
    }

    @Override
    public void onTerminate() {
        super.onTerminate();
        if (client != null)
            client.disconnect();
    }

    //added for dji import classes
    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(RCApplication.this);
    }

    //dji specific
    public BaseProduct getBaseProduct() {
        return mProduct;
    }

    //    public Aircraft getAircraft() {return  (Aircraft) mProduct;}
    @SuppressWarnings("unused")
    public void setBaseProduct(BaseProduct mProduct) {
        RCApplication.mProduct = mProduct;
    }


    //ros client specific
    public ROSBridgeClient getRosClient() {
        return client;
    }
//    public void setRosClient(ROSBridgeClient client) {
//        this.client = client;
//    }

    //connect to ros client from anywhere
    public boolean rosClientConnect(String ip_, String port_) {
        try {
            client = new ROSBridgeClient("ws://" + ip_ + ":" + port_);
            return rosReConnect();
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    static int code = 0;


    public static int getRosMessage() {

        return code;
    }

    public static void resetRosMessage() {

        code = 0;

    }

    public void setTryconnect(boolean tryingConnect) {
        tryReconect = tryingConnect;
    }
    public boolean getTryconnect() {
       return tryReconect ;
    }
    public  void setCallBefore(boolean calledBefore){
        this.calledBefore=calledBefore;
    }
    @SuppressWarnings("UnnecessaryLocalVariable")
    public boolean rosReConnect() {
        try {

            final boolean connect = client.connect(new ROSClient.ConnectionStatusListener() {
                @Override
                public void onConnect() {
                    calledBefore = false;
                    getRemoteLogger().log(1, "rosReConnect: success");
                    Log.d(TAG, "ROS reconnecting");

                    code = 0;
                    client.setDebug(true);
                    showToast("ROS Connected");
                    Log.d("ROS CLIENT", "Connect ROS success");
//                EventBus.getDefault().post(new ButtonEvent("DJI",true));

                    try {
                        ConnectDroneActivity.m.setDisplayDataLeft("ROS CONNECTED");

                    }
                    catch (NullPointerException e){

                    }

                    MainActivity.buttonsEnable(new ButtonEvent("DJI", true));
                    MainActivity.buttonsEnable(new ButtonEvent("ROS", false));
                    MainActivity.setRosConnected(true);
                }

                @Override
                public void onDisconnect(boolean normal, String reason, int code) {
                    RCApplication.code = code;
                    Log.d(TAG, "ROS trying reconnect: "+tryReconect);

                    getRemoteLogger().log(1, "onDisconnect: normal: " + normal + " , " + " , reason: " + reason + " , code:" + code);
                    //  showToast("ROS Disconected " + reason + " " + code);
                    MainActivity.buttonsEnable(new ButtonEvent("DJI", false));
                    Log.d(TAG, "ROS Disconected " + reason + " ,code: " + code);

//                    showToast("Failed To connect to ROS");
                    Log.d("AMAL", getClass().getName());

                    if(code == 1006){
                        MainActivity.showAlert();
                    }



                    MainActivity.setRosConnected(false);
                    ConnectDroneActivity.setRosEnabled(false);

                    ConnectDroneActivity.beforeClose();

                    try {
                        if (!tryReconect) {
                            if (MainActivity.ConnectActivityActive && !ConnectDroneActivity.rosStop && DJISDKManager.getInstance().getProduct() != null) {
                                MainActivity.setRosConnected(false);
                                Log.d(TAG, "respondToRosDisconnect calling.. ");

                                MainActivity.respondToRosDisconnect();
                                Log.d(TAG, "respondToRosDisconnect called ");

                            } else {
                                Log.d(TAG, "respondToRosStop calling.. ");

                                MainActivity.respondToRosStop();
                                Log.d(TAG, "respondToRosStop called ");

                            }
                        } else if (!calledBefore){
                            try {
                                ConnectDroneActivity.m.setDisplayDataLeft("Ros Disconnected... Trying reconnecting");
                                ConnectDroneActivity.m.tryingConnecting();
                            }catch (NullPointerException e){

                            }
                           }
                        client.disconnect();

                    } catch (ExceptionInInitializerError | NoClassDefFoundError e) {

                        Log.d(TAG, e.toString());
                    }


                }



                @Override
                public void onError(Exception ex) {
                    RCApplication.code = -5;


                    //ERROR WHEN CONNECTING

                    getRemoteLogger().log(1, "onError: " + getStackTrace(ex));

                    Log.d("CONNECTERROR", getStackTrace(ex));

                    if((ex.toString()).equals("java.net.ConnectException: Network is unreachable")){
                       Log.d("CONNECTERROR", "HEREEEEEEEE");
                       ConnectDroneActivity.showNetworkReconnectError();
                    }
                    ex.printStackTrace();
                    showToast("ROS Error");
                    Log.d(TAG, "ROS communication error");
                    Log.d(TAG, "ROS trying reconnect: "+tryReconect);

                    MainActivity.buttonsEnable(new ButtonEvent("DJI", false));
                    try {
                        if (!tryReconect) {
                            if (MainActivity.ConnectActivityActive && !ConnectDroneActivity.rosStop && DJISDKManager.getInstance().getProduct() != null) {
                                MainActivity.setRosConnected(false);
                                MainActivity.respondToRosDisconnect();
                            } else {
                                MainActivity.respondToRosStop();
                            }
                        }
                        else if (!calledBefore){
                            ConnectDroneActivity.m.setDisplayDataLeft("Ros Disconnected... Trying reconnection");

                            ConnectDroneActivity .m.tryingConnecting();}
                        client.disconnect();
                    } catch (ExceptionInInitializerError | NoClassDefFoundError e) {
                        Log.d(TAG, e.toString());
                    } catch (Exception e) {
                        Log.d(TAG, e.toString());
                        FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                    }

                }
            });
            return connect;
        } catch (Exception e) {
            getRemoteLogger().log(1, "rosReConnect: " + getStackTrace(e));

            e.printStackTrace();
            return false;
        }
    }

    //show messages on screen
    private void showToast(final String toastMsg) {
        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_LONG).show();
            }
        });
    }


}
