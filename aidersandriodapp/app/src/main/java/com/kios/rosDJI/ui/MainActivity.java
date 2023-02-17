package com.kios.rosDJI.ui;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.ActivityManager;
import android.app.AlarmManager;
import android.app.AlertDialog;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.ContentResolver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.os.StrictMode;
import android.provider.Settings;
import android.text.format.Formatter;
import android.util.Log;
import android.view.GestureDetector;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.view.GestureDetectorCompat;
import dji.common.util.CommonCallbacks;
import com.dji.download.MediaDownload;
import com.google.firebase.crashlytics.FirebaseCrashlytics;
import com.kios.rosDJI.BuildConfig;
import com.kios.rosDJI.Helpers.ButtonEvent;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.Helpers.PhoneDetails;
import com.kios.rosDJI.Helpers.SharedPreferences;
import com.kios.rosDJI.Helpers.WebSocketError;
import com.kios.rosDJI.R;
import com.kios.rosDJI.RCApplication;
import com.remote.log.DeviceDetails;
import com.remote.log.FirebaseAuthentication;
import com.remote.log.TimberRemoteTree;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import butterknife.ButterKnife;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.log.DJILog;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import timber.log.Timber;
import static com.kios.rosDJI.ui.ConnectDroneActivity.sendError;


@SuppressWarnings("deprecation")
public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MainActivity";
    private static final int CONNECT_DRONE_ACTIVITY = 1;
    private static final boolean debug = false;
    private static final boolean clearFiles = false;
    private static boolean registered = false;// boolean to
    private boolean exitDialog = false;

    public static final String FLAG_CONNECTION_CHANGE = "dji_sdk_connection_change";
    public static String droneName = "DJI_Drone";
    private final AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private final List<String> missingPermission = new ArrayList<>();
    public static BaseProduct mProduct;
    public static BaseComponent mComponent;
    private static Handler mHandler;
    private int lastProcess = -1;
    @SuppressLint("StaticFieldLeak")
    public static Activity app;
    public static boolean ConnectActivityActive = false;
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            //Manifest.permission.READ_PHONE_STATE,
    };
    @SuppressLint("StaticFieldLeak")
    private static Button btn_connectRos;
    @SuppressLint("StaticFieldLeak")
    private static Button ros_connect;
    private static Button change_drone_name;
    //Spinner For Easier ROS Master IP selection
    private String[] masterIPs;
    private String ips;
    private final int customPosition = 0;
    private Spinner dropdown;
    private EditText etIP;
    private EditText etPort;
    private static Button download_photos;
    public static String phoneIP;
    public static String masterIP;

    private static boolean rosConnected = false;
    private static boolean connectedDrone = false;
    private boolean tryingreconnect = false;

    public static TimberRemoteTree remoteLogger;
    static Activity activity;
    //call back receiver on connection change between android device and remote controller
    protected BroadcastReceiver mReceiver1 = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {

            Log.d(TAG, "receiver called ");

            BaseProduct product = DJISDKManager.getInstance().getProduct();


            if (product == null || (product != null && !product.isConnected())) {
                Log.d(TAG, "stop timer update drone info");
                connectedDrone = false;
                buttonsEnable(new ButtonEvent("ALL", false));

            } else {
                if (!tryingreconnect/*||!Helper.getBooleanSharedPreference(activity,"user")*/) {
                    String fromRestart = readFromFile(app, "config.txt");
                    Log.d(TAG, "fromRestart: " + fromRestart);
                    getRemoteLogger().log(3, "reconnectingTOROS after registered");
                    reconnectingTOROS(fromRestart.equals("true"));

                }

                connectedDrone = true;
                buttonsEnable(new ButtonEvent("ROS", true));
            }
            RCApplication.resetRosMessage();


        }
    };

    private final BaseComponent.ComponentListener mDJIComponentListener = new BaseComponent.ComponentListener() {

        @Override
        public void onConnectivityChange(boolean isConnected) {
            Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
            notifyStatusChange();
        }
    };

    static boolean startedMain = false;

    public static void respondToRosDisconnect() {
        Log.d(TAG, "respondToRosDisconnect");

        try {
            writeToFile("true", app);

            FirebaseCrashlytics.getInstance().log(TAG + " in respondToRosDisconnect");
            ConnectDroneActivity.beforeClose();
            app.finishActivity(CONNECT_DRONE_ACTIVITY);


            Intent mStartActivity = new Intent(app, MainActivity.class);

            int mPendingIntentId = 123456;
            PendingIntent mPendingIntent = PendingIntent.getActivity(app, mPendingIntentId, mStartActivity,
                    PendingIntent.FLAG_CANCEL_CURRENT);
            AlarmManager mgr = (AlarmManager) app.getSystemService(Context.ALARM_SERVICE);
            Objects.requireNonNull(mgr).set(AlarmManager.RTC, System.currentTimeMillis() + 100, mPendingIntent);
            System.exit(0);
            startedMain = true;

        } catch (Exception e) {
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
        }
    }

    public static void respondToRosStop() {
        try {
            FirebaseCrashlytics.getInstance().log(TAG + " in respondToRosStop");
            ConnectDroneActivity.beforeClose();
            app.finishActivity(CONNECT_DRONE_ACTIVITY);
            writeToFile("false", app);
        } catch (Exception e) {
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
            Log.e(TAG, "respondToRosStop");
        }
    }

    static class MyGestureListener extends GestureDetector.SimpleOnGestureListener {

        @Override
        public boolean onDoubleTap(MotionEvent event) {
            ConnectDroneActivity.changeViewSize();
            return true;
        }
    }

    public static GestureDetectorCompat mDetector;
    static ContentResolver contentResolver;

    public static TimberRemoteTree getRemoteLogger() {
        if (remoteLogger == null) {
            String deviceId = Settings.Secure.getString(contentResolver, Settings.Secure.ANDROID_ID);

            DeviceDetails deviceDetails = new DeviceDetails(deviceId, Build.VERSION.RELEASE, Build.MANUFACTURER, Build.BRAND, Build.DEVICE, Build.MODEL, BuildConfig.VERSION_NAME, BuildConfig.VERSION_CODE);

            remoteLogger = new TimberRemoteTree(deviceDetails);
        }
        return remoteLogger;
    }

// ...
// Initialize Firebase Auth

    @SuppressLint("ClickableViewAccessibility")
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        startedMain = false;

        tryingreconnect = false;
        activity = this;

        super.onCreate(savedInstanceState);


        FirebaseAuthentication firebaseAuthentication = new FirebaseAuthentication(this);
        contentResolver = this.getContentResolver();
        String deviceId = Settings.Secure.getString(this.getContentResolver(), Settings.Secure.ANDROID_ID);
        DeviceDetails deviceDetails = new DeviceDetails(deviceId, Build.VERSION.RELEASE, Build.MANUFACTURER, Build.BRAND, Build.DEVICE, Build.MODEL, BuildConfig.VERSION_NAME, BuildConfig.VERSION_CODE);
        if (remoteLogger == null) {
            remoteLogger = new TimberRemoteTree(deviceDetails);
            Timber.plant(remoteLogger);
        }
        getRemoteLogger().log(3, "MainActivity", "onCreate:MainActivity", null);


        try {
            masterIP = readFromFile(this, "masterIp.txt");

            if (masterIP != null) {

            }

        } catch (Exception e) {
            getRemoteLogger().log(6, "ServerSocket: " + getStackTrace(e));

            e.printStackTrace();

        }


        checkAndRequestPermissions();
        FirebaseCrashlytics.getInstance().setCrashlyticsCollectionEnabled(true);
        setContentView(R.layout.activity_main);

        ButterKnife.bind(this);

        //Add strict policy for socket connection
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        btn_connectRos = findViewById(R.id.btn_startActivity);
        ros_connect = findViewById(R.id.ros_connect);
        download_photos = findViewById(R.id.getPhoto);

        change_drone_name = findViewById(R.id.change_drones_name_btn);
        etIP = findViewById(R.id.et_ip);
        etIP.setImeOptions(EditorInfo.IME_ACTION_DONE);
        etPort = findViewById(R.id.et_port);
        ImageView kiosLogo = findViewById(R.id.kiosLogo);
        Log.d(TAG, "oncreate");

        dropdown = findViewById(R.id.masterIPSpinner);

        dropdown.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {

            @Override
            public void onItemSelected(AdapterView<?> parent, View view,
                                       int position, long id) {
                if(position != 0){
                    etIP.setText(dropdown.getSelectedItem().toString());
                }

            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // TODO Auto-generated method stub

            }
        });

        dropdown.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View v) {
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                // Specify alert dialog is not cancelable/not ignorable
                builder.setCancelable(true);
                builder.setTitle("Clear IPs List");
                builder.setMessage("Are you sure you want to clear the IPs list?");
                builder.setIcon(getResources().getDrawable(R.drawable.warning));

                builder.setPositiveButton("YES", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        clearFiles();
                    }
                });
                builder.setNegativeButton("NO", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.dismiss();
                    }
                });
                // Create the alert dialog
                final AlertDialog dialog = builder.create();
                dialog.show();

                return false;
            }
        });

        mDetector = new GestureDetectorCompat(this, new MyGestureListener());

        mHandler = new Handler(Looper.getMainLooper());

        app = MainActivity.this;
        hideKeyboard();



        etIP.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
//                Log.d("AMAL", dropdown.getSelectedItem() + " " + dropdown.getSelectedItemPosition() + " ");
                dropdown.setSelection(customPosition, true);
            }
        });
        etPort.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                dropdown.setSelection(customPosition, true);
            }
        });


        //region Recover from ROS
        String fromRestart = readFromFile(app, "config.txt");
        // if (fromRestart.equals("true")) {
        Log.d("MainActivity", "From restart");


        //region Prepare txt Files
        if (clearFiles) {
            clearFiles();
        } else {
            writeToFile("false", app);
            try {
                ips = readFromFile(app, "masterIPs.txt");
                masterIPs = ips.split(",");
            } catch (NullPointerException e) {
                e.printStackTrace();
                ips = "Custom,192.168.1.3,192.168.1.4";
                createIPlist(app, ips);
                masterIPs = new String[]{"Custom", "192.168.1.3", "192.168.1.4"};
            }
            if (masterIPs.length == 1) {
                ips = "Custom,192.168.1.11,172.20.81.57,172.20.245.54";
                createIPlist(app, ips);
                masterIPs = new String[]{"Custom", "192.168.1.11", "172.20.81.57", "172.20.245.54"};
            }
            Log.d(TAG, "masterIPs: " + Arrays.toString(masterIPs));
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, masterIPs);

            dropdown = findViewById(R.id.masterIPSpinner);
            dropdown.setAdapter(adapter);

            try {
                int pos = Integer.parseInt(readPositionFromFile(app));
                dropdown.setSelection(pos);
            } catch (RuntimeException e) {
                int pos = 0;
                dropdown.setSelection(pos);
                writePositionToFile("0", app);
            }
        }
        //endregion Prepare txt Files

        IntentFilter intent = new IntentFilter();

        intent.addAction(FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver1, intent);
        if (!PhoneDetails.getDeviceName().equalsIgnoreCase("DJI pm430") && PhoneDetails.getAndroidVersion() > 10 && !Environment.isExternalStorageManager()) {

            Intent intent2 = new Intent(Settings.ACTION_MANAGE_ALL_FILES_ACCESS_PERMISSION);

            startActivity(intent2);
        }

    }


    CountDownTimer countDownTimer;

    private synchronized void reconnectingTOROS(boolean connectToRos) {
        tryingreconnect = false;
        int message = RCApplication.getRosMessage();
        boolean byuser = SharedPreferences.getBooleanSharedPreference(this, "byuser");
        Log.d("MainActivity", "reconnectingTOROS: message from ROS: " + message);
        Log.d("MainActivity", "reconnectingTOROS: by user: " + byuser);
        Log.d("MainActivity", "connectToRos: " + connectToRos);

        remoteLogger.log(6, "reconnectingTOROS-> byuser: " + byuser + ", error message: " + message + "connectToRos: " + connectToRos);
        if (isActivityRunning(ConnectDroneActivity.getActivity())) {
            Log.d("MainActivity", "reconnectingTOROS: activity already running ");
            SharedPreferences.setBooleanSharedPreference(this, "byuser", true);
            return;
        }


        if (!connectToRos && (message == 0 || byuser)) {
            SharedPreferences.setBooleanSharedPreference(this, "byuser", true);
            return;
        }

        if (byuser) {
            SharedPreferences.setBooleanSharedPreference(this, "byuser", true);
            return;
        }


        WebSocketError.Error error = WebSocketError.getDescription(message);

        if ( /*&& error.code != 1000 &&*/ DJISampleApplication.isAircraftConnected()) {
            exitDialog = false;

            final Handler handler = new Handler() {
                @Override
                public void handleMessage(Message mesg) {
                    throw new RuntimeException();
                }
            };

            final AlertDialog.Builder alertDialog = new AlertDialog.Builder(this);
            final AlertDialog alertDialog1 = alertDialog.create();

            alertDialog1.setTitle("ROS DISCONNECTED: " + error.toString());
            alertDialog1.setMessage("count down");
            alertDialog1.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    exitDialog = true;
                    countDownTimer.cancel();
                    dialog.dismiss();
                    handler.sendMessage(handler.obtainMessage());

                }
            });
            alertDialog1.show();

            for (int i = 0; i < 3; i++) {


                countDownTimer = new CountDownTimer(2 * 1000, 1000) {
                    @Override
                    public void onTick(long millisUntilFinished) {
                        Log.d("MainActivity", "reconnectingTOROS count down : " + ("00:" + (millisUntilFinished / 1000)));
                        alertDialog1.setMessage("Trying Reconnecting to ROS in 00:" + (millisUntilFinished / 1000) + " s");
                    }

                    @Override
                    public void onFinish() {
                        alertDialog1.setMessage("Reconnecting to ROS...");

                        connectToRos();

                        if (rosConnected) {
                            alertDialog1.setMessage("ROS CONNECTED");

                            Log.d("MainActivity", "reconnectingTOROS tried : " + rosConnected);

                            exitDialog = true;
                        } else {

                            Log.d("MainActivity", "reconnectingTOROS try again");


                        }
                        handler.sendMessage(handler.obtainMessage());

                    }
                };

                countDownTimer.start();
                // loop till a runtime exception is triggered.
                try {
                    Looper.loop();
                } catch (RuntimeException e2) {
                }
                if (exitDialog)
                    break;
            }
            alertDialog1.hide();

            if (rosConnected) {
                Intent intent2 = new Intent(MainActivity.this, ConnectDroneActivity.class);
                startActivityForResult(intent2, CONNECT_DRONE_ACTIVITY);
            } else {


                try {


                    ((Aircraft) mProduct).getFlightController().startGoHome(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                Log.e("GoHome", djiError.getDescription());
                                showToast("GoHome" + djiError.getDescription());
                            } else {
                                Log.e("GoHome", "Successful but returned Null");
                                showToast("GoHome Successful");
                            }
                        }
                    });

                    if (!((Aircraft) mProduct).getFlightController().getState().isGoingHome()) {
                        ((Aircraft) mProduct).getFlightController().startLanding(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                    Log.d(TAG, djiError.toString());
                                    showToast("Land" + djiError.getDescription());
                                } else {
                                    Log.e("Land", "Successful but returned Null");
                                    showToast("Land Successful");
                                }
                            }
                        });
                    }


                } catch (NullPointerException e) {
                    Log.e("RECOVERY THREAD", e.toString());
                }
            }


        } else {
            new Helper().showDialog(this, "ROS DISCONNECTED " + error.code, "REASON: " + error.reason, false);
        }
        SharedPreferences.setBooleanSharedPreference(this, "byuser", true);

    }

    @SuppressLint("MissingSuperCall")
    @Override
    protected void onNewIntent(@NonNull Intent intent) {
        String action = intent.getAction();
        if (UsbManager.ACTION_USB_ACCESSORY_ATTACHED.equals(action)) {
            Intent attachedIntent = new Intent();
            attachedIntent.setAction(DJISDKManager.USB_ACCESSORY_ATTACHED);
            sendBroadcast(attachedIntent);
        }
    }

    @Override
    protected void onDestroy() {
        Log.d("MainActivity", "ondestroy");
        super.onDestroy();
        unregisterReceiver(mReceiver1);
        SharedPreferences.setBooleanSharedPreference(this, "byuser", true);
    }


    @Override
    protected void onResume() {
        startedMain = false;
        Log.d(TAG, "onResume " + tryingreconnect);
        getRemoteLogger().log(3, "MainActivity", "onResume:MainActivity", null);

        if (!tryingreconnect && registered) {

            String fromRestart = readFromFile(app, "config.txt");
            Log.d(TAG, "onResume-> fromRestart: " + fromRestart);

            getRemoteLogger().log(3, "MainActivity", "onResume -> reconnectingTOROS", null);


            reconnectingTOROS(fromRestart.equals("true"));


        }

        ConnectActivityActive = btn_connectRos != null && btn_connectRos.isEnabled();
        super.onResume();

        try {
            WifiManager wm = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
            phoneIP = Formatter.formatIpAddress(Objects.requireNonNull(wm).getConnectionInfo().getIpAddress());
            String[] phoneIpParts = phoneIP.split("\\.");
            String customIPstart = phoneIpParts[0] + "." + phoneIpParts[1] + "." + phoneIpParts[2] + ".";
            masterIP = readFromFile(this, "masterIp.txt");


            etIP.setText(masterIP != null && masterIP.compareTo("") != 0 ? masterIP : customIPstart);
        } catch (NullPointerException e) {
            Log.e(TAG, "GET IP ERROR: " + e.toString());
        }

        if (debug)
            buttonsEnable(new ButtonEvent("ALL", true));
        else {

            if (connectedDrone && !rosConnected)
                buttonsEnable(new ButtonEvent("ROS", true));
            else if (connectedDrone && rosConnected) {
                buttonsEnable(new ButtonEvent("DJI", true));
                buttonsEnable(new ButtonEvent("ROS", false));
            } else
                buttonsEnable(new ButtonEvent("ROS", false));


        }

        ips = readFromFile(app, "masterIPs.txt");
        masterIPs = ips.split(",");
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, masterIPs);
        dropdown = findViewById(R.id.masterIPSpinner);
        dropdown.setAdapter(adapter);
        int pos = Integer.parseInt(readPositionFromFile(app));
        dropdown.setSelection(pos);

        hideKeyboard();
        getSupportActionBar().hide();
    }

    private void clearFiles() {
        writeToFile("false", app);

        ips = "Custom,192.168.1.11,172.20.81.57,172.20.245.54";
        createIPlist(app, ips);
        masterIPs = new String[]{"Custom", "192.168.1.11", "172.20.81.57", "172.20.245.54"};

        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, masterIPs);

        dropdown = findViewById(R.id.masterIPSpinner);
        dropdown.setAdapter(adapter);

        int pos = 0;
        dropdown.setSelection(pos);

        writePositionToFile("0", app);
    }

    private final boolean fromNodes = false;

    public void onClick(View view) {
        switch (view.getId()) {
            case R.id.getPhoto:

                try {
                    boolean isflying = ((Aircraft) Helper.getProductInstance()).getFlightController().getState().isFlying();
                    if (isflying) {
                        new Helper().showDialog(this, "Land aircraft to download photos", "WARNING", false);
                    } else {
                        MediaDownload mediaDownload = new MediaDownload(activity, null);
                        mediaDownload.MediaDownloadPhotos();
                    }
                } catch
                (Exception e) {

                }
                break;
            case R.id.tv_ros:
                showToast("Kios Center of Excellence");
                break;
            case R.id.ros_connect: //connect client to ros
                FirebaseCrashlytics.getInstance().log(TAG + " onClick ROS");
                if(droneName.contains(" ")){

                    activity.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            AlertDialog.Builder alertDialog = new AlertDialog.Builder(activity);
                            alertDialog.setTitle("Invalid Drone Name");
                            alertDialog.setMessage("There are spaces in the Drone's name, do you allow the name to be changed by replacing spaces with underscore");
                            AlertDialog accountDialog = alertDialog.create();

                            alertDialog.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    showToast(droneName);
                                    dialog.dismiss();
                                }
                            });

                            alertDialog.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    droneName = droneName.replaceAll(" ", "_");


                                    mProduct.setName(droneName, new CommonCallbacks.CompletionCallback() {
                                        @Override
                                        public void onResult(DJIError djiError) {
                                            if (djiError != null) {
                                                sendError(djiError.toString());
                                                showToast(djiError.toString());
                                            } else {
                                                showToast(droneName);
                                               //successfully changed name
                                            }
                                        }
                                    });

                                    connectToRos();
                                    dialog.dismiss();
                                }
                            });

                            alertDialog.setCancelable(true);
                            alertDialog.show();
                        }
                    });
                }else{
                    connectToRos();

                }


                break;
            case R.id.btn_startActivity: //start activity to connect to drone
                FirebaseCrashlytics.getInstance().log(TAG + " onClick ConnectRos");
                Intent intent2 = new Intent(MainActivity.this, ConnectDroneActivity.class);
                startActivityForResult(intent2, CONNECT_DRONE_ACTIVITY);
                break;
            case R.id.change_drones_name_btn:
                //region Change Drone's Name
                FirebaseCrashlytics.getInstance().log(TAG + " onClick ChangeDroneName");
                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                LayoutInflater inflater = getLayoutInflater();
                View dialogView = inflater.inflate(R.layout.edtxt_alert, null);

                // Specify alert dialog is not cancelable/not ignorable
                builder.setCancelable(true);

                // Set the custom layout as alert dialog view
                builder.setView(dialogView);

                // Get the custom alert dialog view widgets reference
                Button btn_positive = dialogView.findViewById(R.id.dialog_positive_btn);
                Button btn_negative = dialogView.findViewById(R.id.dialog_negative_btn);
                final EditText et_name = dialogView.findViewById(R.id.et_name);
                TextView tv_curr_name = dialogView.findViewById(R.id.dialog_drone_name);
                tv_curr_name.setText(droneName);

                // Create the alert dialog
                final AlertDialog dialog = builder.create();

                // Set positive/yes button click listener
                btn_positive.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        // Dismiss the alert dialog
                        dialog.cancel();
                        hideKeyboard();

                        final String name = et_name.getText().toString();
                        hideKeyboard();

                        mProduct.setName(name, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                    sendError(djiError.toString());
                                    showToast(djiError.toString());
                                } else {
                                    droneName = name;
                                    showToast("Successful!!!");
                                }
                            }
                        });
                        hideKeyboard();
                    }
                });

                // Set negative/no button click listener
                btn_negative.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        // Dismiss/cancel the alert dialog
                        //dialog.cancel();
                        dialog.dismiss();
                    }
                });

                // Display the custom alert dialog on interface
                if (mProduct != null) {
                    dialog.show();
                } else
                    showToast("Please Connect a Drone First");
                //endregion Change Drone's Name
                break;
            default:
                break;
        }
    }


    private String ip;
    private String port = "9090";

    public void connectToRos() {
        FirebaseCrashlytics.getInstance().log(TAG + " connectToRos");
        int position = dropdown.getSelectedItemPosition();

        writePositionToFile(position + "", app);

        if (position == customPosition) {
            ip = etIP.getText().toString();
            writeToFileMaster(ip, this);

            port = etPort.getText().toString();
        } else {
            ip = masterIPs[position];
            port = "9090";
        }
        try {
            if (!checkIP(ip)) {
                if (position == customPosition)
                    etIP.setError("IP address not Valid");
                else
                    showToast("Selected IP is Invalid");
            } else if (port.isEmpty()) {
                if (position == customPosition)
                    etPort.setError("Port number not Valid");
                else
                    showToast("Preconfigured Port is Invalid");
            } else {
                Thread connectionThread = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        rosConnected = ((RCApplication) getApplication()).rosClientConnect(ip, port);
                        FirebaseCrashlytics.getInstance().log(TAG + " rosConnected: " + rosConnected);
                        Log.d(TAG, "connectionThread: " + ip);
                        if (position == customPosition && rosConnected) {

                            if (!ips.contains(ip)) {
                                Log.d(TAG, "connectionThread save ip: " + ip);

                                ips = ips + "," + ip;
                                createIPlist(app, ips);
                            }
                        }
                    }
                });
                masterIP = ip;
                connectionThread.start();
                connectionThread.join();
            }

        } catch (Exception e) {
            Log.e(TAG, e.toString());
        }

    }

    private boolean checkIP(String ip) {
        try {
            if (ip == null || ip.isEmpty()) {
                return false;
            }

            String[] parts = ip.split("\\.");
            String[] phoneIpParts = phoneIP.split("\\.");
            if (parts.length != 4) {
                return false;
            }
            return !ip.endsWith(".");
        } catch (NumberFormatException nfe) {
            return false;
        }
    }

    //shw msgs on the screen!
    private void showToast(final String toastMsg) {
        MainActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    //    private boolean once=false;
    // enable buttons when event has happened
    public static void buttonsEnable(final ButtonEvent event) {
        if (!isActivityRunning(app))
            return;
        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (event.name.equals("ROS")) {
                    ros_connect.setEnabled(event.msg);
                    change_drone_name.setEnabled(event.msg);
                    download_photos.setEnabled(event.msg);

                    Log.d(TAG, "ButtonsEnable ROS " + event.msg);
                }
                if (event.name.equals("DJI")) {
                    btn_connectRos.setEnabled(event.msg);
                    download_photos.setEnabled(event.msg);

                    Log.d(TAG, "ButtonsEnable DJI " + event.msg);
                }
                if (event.name.equals("ALL")) {
                    download_photos.setEnabled(event.msg);
                    btn_connectRos.setEnabled(event.msg);
                    ros_connect.setEnabled(event.msg);
                    change_drone_name.setEnabled(event.msg);
                    Log.d(TAG, "ButtonsEnable ALL " + event.msg);
                }
            }
        });
    }

    private void startNextActivity() {
        FirebaseCrashlytics.getInstance().log(TAG + " startNextActivity");
        if (isActivityRunning(ConnectDroneActivity.getActivity())) {
            FirebaseCrashlytics.getInstance().log(TAG + " startNextActivity: ConnectDroneActivity is running");

            return;
        }
        try {
            connectToRos();
            if (rosConnected) {
                Intent intent = new Intent(MainActivity.this, ConnectDroneActivity.class);

                startActivityForResult(intent, 1);
            }

        } catch (Exception e) {
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
            Log.e(TAG, e.toString());
        }
    }

    public static void setRosConnected(boolean rosConnected) {
        MainActivity.rosConnected = rosConnected;
    }
    // DJI RELATED TO registration and product connection!

    /**
     * DJI
     * Result of runtime permission request
     */
    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            if (!registered) {
                startSDKRegistration();
            }
        } else {
            showToast("Missing permissions!!!");
        }
    }


    /**
     * DIJ
     * Checks if there is any missing permissions, and
     * requests runtime permission if needed.
     */
    private void checkAndRequestPermissions() {
        // Check for permissions
        for (String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (missingPermission.isEmpty()) {
            // if (!registered) {
            //   registered = true;
            if (!registered) {
                startSDKRegistration();
            }            //}
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            showToast("Need to grant the permissions!");
            ActivityCompat.requestPermissions(this,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }
    }

    /**
     * DJI
     * startSDKRegistration
     */
    public void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            buttonsEnable(new ButtonEvent("ALL", false));

            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    showToast(MainActivity.this.getString(R.string.sdk_registration_doing_message));
                    DJISDKManager.getInstance().registerApp(MainActivity.this.getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {

                                registered = true;

                                DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                DJISDKManager.getInstance().startConnectionToProduct();
                                showToast(MainActivity.this.getString(R.string.sdk_registration_success_message));


                            } else {
                                showToast(MainActivity.this.getString(R.string.sdk_registration_message));
                            }
                            Log.v(TAG, djiError.getDescription());
                        }

                        @Override
                        public void onProductDisconnect() {
                            SharedPreferences.setBooleanSharedPreference(activity, "byuser", true);

                            Log.d(TAG, "onProductDisconnect");
                            buttonsEnable(new ButtonEvent("ALL", false));
                            if (ConnectActivityActive) {
                                ConnectDroneActivity.rosStop = true;
//                                respondToRosStop();
                            }
                            notifyStatusChange();
                            connectedDrone = false;

                        }

                        @Override
                        public void onProductConnect(BaseProduct baseProduct) {
                            connectedDrone = true;


                            try {
                                Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                                //mProduct = baseProduct;
                                mProduct = DJISDKManager.getInstance().getProduct();
                                showToast(baseProduct.getModel().getDisplayName() + " connected");
                                notifyStatusChange();
                                constructDroneName();
                                buttonsEnable(new ButtonEvent("ROS", true));
                            } catch (NullPointerException e) {
                                FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                                Log.e(TAG, "exception: " + e.toString());
                            }
                        }

                        @Override
                        public void onProductChanged(BaseProduct baseProduct) {
                            Log.d(TAG, "onProductChanged");

                            constructDroneName();
                            buttonsEnable(new ButtonEvent("ALL", false));
                            if (ConnectActivityActive) {
                                ConnectDroneActivity.rosStop = true;
                                //  respondToRosStop();
                            }
                            notifyStatusChange();
                        }

                        @Override
                        public void onComponentChange(BaseProduct.ComponentKey componentKey,
                                                      BaseComponent oldComponent,
                                                      BaseComponent newComponent) {
                            if (newComponent != null) {
                                newComponent.setComponentListener(mDJIComponentListener);
                                mComponent = newComponent;
                            }
                            Log.d(TAG,
                                    String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                            componentKey,
                                            oldComponent,
                                            newComponent));

                            notifyStatusChange();
                        }

                        @Override
                        public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {
                        }

                        @Override
                        public void onDatabaseDownloadProgress(long current, long total) {
                            int process = (int) (100 * current / total);
                            if (process == lastProcess) {
                                return;
                            }
                            lastProcess = process;
//                            showProgress(process);
                            if (process % 25 == 0) {
                                showToast("DB load process : " + process);
                            } else if (process == 0) {
                                showToast("DB load begin");
                            }
                        }
                    });
                }
            });
        }
    }

    public static void notifyStatusChange() {
        mHandler.removeCallbacks(updateRunnable);
        mHandler.postDelayed(updateRunnable, 500);
    }

    private static final Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            Intent intent = new Intent(FLAG_CONNECTION_CHANGE);
            app.sendBroadcast(intent);
        }
    };

    private static void writeToFile(String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("config.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private static void writeToFileMaster(String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("masterIp.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private String readFromFile(Context context, String filename) {

        String ret = "";

        try {
            InputStream inputStream = context.openFileInput(filename);

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString;
                StringBuilder stringBuilder = new StringBuilder();

                while ((receiveString = bufferedReader.readLine()) != null) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }

    private static void createIPlist(Context context, String data) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("masterIPs.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private static void writePositionToFile(String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("position.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private String readPositionFromFile(Context context) {

        String ret = "";

        try {
            InputStream inputStream = context.openFileInput("position.txt");

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString;
                StringBuilder stringBuilder = new StringBuilder();

                while ((receiveString = bufferedReader.readLine()) != null) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }


    public static void hideKeyboard() {
        InputMethodManager imm = (InputMethodManager) app.getSystemService(Activity.INPUT_METHOD_SERVICE);
        //Find the currently focused view, so we can grab the correct window token from it.
        View view = app.getCurrentFocus();
        //If no view currently has focus, create a new one, just so we can grab a window token from it
        if (view == null) {
            view = new View(app);
        }
        if (imm != null) {
            imm.hideSoftInputFromWindow(view.getWindowToken(), 0);
        }
    }

    public static boolean isActivityRunning(Activity activity) {
        if (activity == null) {
            Log.d("MainActivity", "isActivityRunning: is null");

            return false;
        }
        ActivityManager activityManager = (ActivityManager) activity.getSystemService(Context.ACTIVITY_SERVICE);
        List<ActivityManager.RunningTaskInfo> activitys = activityManager.getRunningTasks(Integer.MAX_VALUE);
        boolean isActivityFound = false;
        for (int i = 0; i < activitys.size(); i++) {
            Log.d("MainActivity", "isActivityRunning: " + activitys.get(i));
            if (activitys.get(i).topActivity.equals(activity.getComponentName())) {
                Log.d("MainActivity", "isActivityRunning: found activity");

                isActivityFound = true;
                break;
            }
        }
        return isActivityFound;
    }


    public static String getStackTrace(Exception e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        return sw.toString(); // stack trace as a string
    }

    public void constructDroneName() {
        if (mProduct != null) {
            mProduct.getName(new CommonCallbacks.CompletionCallbackWith<String>() {
                @Override
                public void onSuccess(String s) {
                    getRemoteLogger().log(3, "MainActivity:constructDroneName, Name based on dji: " + s);
                    Log.d("MainActivity", "Name based on dji: " + s);
                    droneName = s;
                }

                @Override
                public void onFailure(DJIError djiError) {
                    try {
                        if (djiError != null) {
                            getRemoteLogger().log(3, "MainActivity: constructDroneName, failed to get name: " + djiError.toString());
                            Log.d("MainActivity", " failed to get name: " + djiError.toString());
                        } else {
                            getRemoteLogger().log(3, "MainActivity: constructDroneName, failed to get name: ");
                            Log.d("MainActivity", " failed to get name: ");
                        }
                    } catch (NullPointerException e) {
                        e.printStackTrace();

                    }

                }
            });
        }
    }

    @Override
    protected void onPause() {


        Log.d("MainActivity", "onPause");

        super.onPause();

    }

    public static void showAlert() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder alertDialog = new AlertDialog.Builder(activity);
                alertDialog.setTitle("Failed connecting to ROS");
                alertDialog.setMessage("- Ensure that you are connected to the same network as the Server\n\n - Ensure you have selected the correct IP Address above");
                AlertDialog accountDialog = alertDialog.create();

                alertDialog.setNegativeButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.dismiss();
                    }
                });

                alertDialog.setCancelable(true);
                alertDialog.show();
            }
        });
    }

    public static void showNetworkReconnectError() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder alertDialog = new AlertDialog.Builder(activity);
                alertDialog.setTitle("Network Not Reachable");
                alertDialog.setMessage("Unable to Connect to ROS due to Network Issue. Press Okay to Close the App, check your connection and try again!");
                AlertDialog accountDialog = alertDialog.create();

                alertDialog.setNegativeButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.dismiss();
                        activity.finish();
                        System.exit(0);
                    }
                });

                alertDialog.setCancelable(true);
                alertDialog.show();
            }
        });
    }


}
