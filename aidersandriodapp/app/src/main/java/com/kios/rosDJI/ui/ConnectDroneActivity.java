package com.kios.rosDJI.ui;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.PointF;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.ActionBarDrawerToggle;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.core.view.GestureDetectorCompat;
import androidx.core.view.GravityCompat;
import androidx.drawerlayout.widget.DrawerLayout;
import android.text.format.Formatter;
import android.util.Log;
import android.view.Display;
import android.view.GestureDetector;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.inputmethod.InputMethodManager;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.RelativeLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import com.dji.cameras.CameraDrone;
import com.dji.download.MediaDownload;
import com.dji.download.SendPhotosToROS;
import com.dji.waypoint.Mission;
import com.google.android.material.navigation.NavigationView;
import com.google.android.material.snackbar.Snackbar;
import com.google.firebase.crashlytics.FirebaseCrashlytics;
import com.google.firebase.database.annotations.NotNull;
import com.google.gson.Gson;
import com.jilk.ros.Service;
import com.jilk.ros.message.PointCloudMsg;
import com.jilk.ros.message.TelemetryMsg;
import com.jilk.ros.message.BuildMap;
import com.jilk.ros.message.TimePrimitive;
import com.jilk.ros.rosapi.message.Empty;
import com.jilk.ros.rosapi.message.GetTime;
import com.jilk.ros.rosbridge.ROSBridgeClient;
import com.kios.rosDJI.ExitService;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.Helpers.MessageEvent;
import com.kios.rosDJI.Helpers.Notifications;
import com.kios.rosDJI.Helpers.SharedPreferences;
import com.kios.rosDJI.Helpers.VideoFeedView;
import com.kios.rosDJI.Helpers.VideoFeedViewPrimary;
import com.kios.rosDJI.R;
import com.kios.rosDJI.RCApplication;
import com.kios.rosDJI.entity.PublishEvent;
import com.tensorFlow.DetectorActivity;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.RandomAccessFile;
import java.nio.channels.NotYetConnectedException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.Timer;
import java.util.TimerTask;
import butterknife.ButterKnife;
import de.greenrobot.event.EventBus;
import dji.common.airlink.PhysicalSource;
import dji.common.battery.BatteryState;
import dji.common.camera.CameraVideoStreamSource;
import dji.common.camera.LaserMeasureInformation;
import dji.common.camera.SettingsDefinitions;
import dji.common.camera.StorageState;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
import dji.common.flightcontroller.VisionDetectionState;
import dji.common.flightcontroller.simulator.InitializationData;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.hotpoint.HotpointHeading;
import dji.common.mission.hotpoint.HotpointMission;
import dji.common.mission.hotpoint.HotpointStartPoint;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionGotoWaypointMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.model.LocationCoordinate2D;
import dji.common.perception.DJILidarPointCloudRecord;
import dji.common.perception.DJILidarPointCloudSamplingRate;
import dji.common.perception.DJILidarPointCloudVisibleLightPixelMode;
import dji.common.perception.RecordingStatus;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.keysdk.callback.ActionCallback;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.camera.Camera;
import dji.sdk.camera.Lens;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.lidar.Lidar;
import dji.sdk.lidar.processor.DJILidarLiveViewDataProcessor;
import dji.sdk.lidar.reader.PointCloudLiveViewData;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.hotpoint.HotpointMissionOperator;
import dji.sdk.mission.hotpoint.HotpointMissionOperatorListener;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.sdkmanager.LiveStreamManager;
import dji.sdk.sdkmanager.LiveVideoBitRateMode;
import dji.sdk.sdkmanager.LiveVideoResolution;
import dji.ux.panel.PreFlightCheckListPanel;
import dji.ux.widget.RadarWidget;
import dji.ux.workflow.CompassCalibratingWorkFlow;
import com.squareup.otto.Subscribe;
import static com.kios.rosDJI.ui.MainActivity.getRemoteLogger;
import static com.kios.rosDJI.ui.MainActivity.getStackTrace;
import static com.kios.rosDJI.ui.MainActivity.masterIP;
import static com.kios.rosDJI.ui.MainActivity.setRosConnected;
import static com.kios.rosDJI.ui.ModuleVerificationUtil.getFlightController;
import static dji.ux.workflow.CompassCalibratingWorkFlow.startCalibration;
import static java.lang.Math.sqrt;
import static java.lang.Thread.sleep;

public class ConnectDroneActivity extends AppCompatActivity implements NavigationView.OnNavigationItemSelectedListener {
    public static boolean active = false;
    public static boolean rosStop = false;
    @SuppressLint("StaticFieldLeak")
    public static boolean isDetectionActive = false;
    public static DetectorActivity detectorActivity;
    public static int detectCounter = 0;

    public boolean showDiagnostics = false;

    private static final String TAG = ConnectDroneActivity.class.getSimpleName();
    @SuppressLint("StaticFieldLeak")
    public static Activity act;
    private static boolean rtmpStream = false;
    @SuppressLint("StaticFieldLeak")
    private static TextView liveTxt;
    static FlightController flightController = getFlightController();
    protected static boolean DEBUG = false; // start fake telemetry data to be send
    public static double imgRateInHz = 3;
    public static double detectRateinHz = 30;
    static double telemetryRateInHz = 3;
    static double pictureRateInHz = 1;
    static double updateScreenTextRateInHz = 1.0;
    static int maxNumOfRetrialsConnect = 3;
    private static HotpointMissionOperator hotpointMissionOperator;
    private HotpointMission hpMission;
    private static HotpointMissionOperatorListener hpListener;
    private String picServerIP = masterIP;
    private int picServerPort = 5001;
    private Boolean activityPaused = false;

    static Boolean handshakeconnected = false;
    static TimerTask timerConnectDroneROS = null;

    //Lidar Variables
    private dji.sdk.lidar.Lidar currentLidar = null;
    private RecordingStatus recordingStatus = null;
    private int recordingTime = 0;
    private Boolean startLidar = true;
    private Boolean startRangeFinder = true;
    private Lidar.DJIPointCloudLiveDataListener LidarLiveListener;


    //handlers
    public static Handler handler;
    static Handler handlerDisplay;
    @SuppressLint("StaticFieldLeak")
    private static VideoFeedView videoFeedViewFPV;
    private static boolean fpvViewFullscreen = false;
    private LiveStreamManager.LiveStreamVideoSource currentVideoSource = LiveStreamManager.LiveStreamVideoSource.Primary;

    //UI binders
    private Button btn_service;
    private Button activity_main_screen_surface;
    private View telemetryScrollView;
    private TextView textQualityRate;

    int camera_mode = 1;
    private String pointCloudString = "";

    //Ros Related
    private static ROSBridgeClient client;
    private static Service<Empty, GetTime> timeService = null;
    private static Boolean rosEnabled = false;
    private static String serialNum = Math.random() * 10 + "";
    private static String droneStart = "DJI_Aircraft";
    private static String droneName = droneStart + serialNum;
    private static String droneIp;
    private String streamURl = "";
    private String phoneIp;
    public static final String[] topics = {"Detection", "Telemetry", "Mission", "Input", "Error",
            "Release", "Stream", "Zoom", "HpMission", "ObstacleReport",
            "BuildMapRequest", "BuildMapResponse", "PicServerSetup", "handshake", "PointCloud",
            "StartOrStopPointCloud", "RangeFinder", "StartOrStopRangeFinder"};
    private static String[] how = {"advertise", "advertise", "subscribe", "subscribe", "advertise",
            "subscribe", "subscribe", "subscribe", "subscribe", "advertise",
            "subscribe", "advertise", "subscribe", "subscribe", "advertise",
            "subscribe", "advertise", "subscribe"};
    private static String[] types = {"std_msgs/String", "kios/Telemetry", "kios/MissionDji", "kios/InputDJI",
            "std_msgs/String", "kios/Release", "kios/Stream", "kios/Zoom", "kios/HpMissionDji",
            "std_msgs/String", "kios/BuildMap", "kios/BuildMap", "kios/PicServer", "std_msgs/String",
            "std_msgs/String", "std_msgs/String", "std_msgs/String", "std_msgs/String"};


    String startOrStopPointCloud = "";
    String startOrStopRangeFinder = "";

    public MenuItem item1;

    boolean toggleImgSent = true;
    int imgQuality = 10;
    double imgRate = 0.0;
    double teleRate = 0.0;
    static volatile boolean telemetry = true;
    static volatile boolean battery = true;
    static volatile boolean detection = true;
    private String dataToDisplay = "";
    private String dataToDisplayLeft = "";
    public static int count;
    public static int videoViewWidth;
    public static int videoViewHeight;

    // DJI Stuff
    private static BaseProduct mProduct = MainActivity.mProduct;
    private VisionDetectionState visionDetectionState = null;
    private GimbalState gimbalState = null;
    private Camera mCamera;
    private SystemState mCameraState;
    static CameraDrone cameraDrone;

    private boolean isLaserEnabled = true;

    SettingsDefinitions.StorageLocation storageLocation;

    //    private Button recordBtn;
//    private Switch cameraSwitch;
    private BatteryState mBattery;
    private final int remainingFlightTimeLimit = 120; //remaining flight time limit in seconds
    private File missionFile;
    private View zoomLayout;
    private Button zoomIn;
    private Button zoomOut;
    private ImageView drawerImage;
    private TextView drawerName;
    private TextView drawerIP;
    private Boolean webViewShowing = false;
    private WebView wv;
    private Boolean inMission = false;
    @SuppressLint("StaticFieldLeak")
    private static TextView msg_det;
    @SuppressLint("StaticFieldLeak")
    private static TextView sub_info;
    private int alertState = 0;
    private static int displayWidth;
    private RadarWidget radarWidget;

    @SuppressLint("StaticFieldLeak")
    public static ConnectDroneActivity m;

    private File dir;
    private File dir2;
    private File dir3;

    private CompassCalibratingWorkFlow calibratingWorkFlow;
    private boolean obstacleThreadRunning = false;
    private boolean virtualSticksThread = false;
    private boolean keepOriginalPhoto = false;

    static Timer timerTelemetry, timerPictures, timerObstacle, timerSticks, timerConnectToRos;//timers sending data to master ros
    static Timer timerDisplaydata;
    static Activity activityConnect;

    boolean dataLeft = false, dataRight = false; //whether there ara messages to dispay right or left on the screen ros

    public static Activity getActivity() {
        return activityConnect;
    }

    public File getDir() {

        return dir;
    }

    protected void onCreate(Bundle savedInstanceState) {
        calledBeforeclose = false;

        super.onCreate(savedInstanceState);
        SharedPreferences.setBooleanSharedPreference(this, "byuser", false);
        activityConnect = this;
        Log.d(TAG, "masterIP: " + masterIP);

        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.drawer_layout);
        FirebaseCrashlytics.getInstance().setCustomKey("TAG", TAG);
        FirebaseCrashlytics.getInstance().log("ConnectDroneActivity OnCreate");
        mProduct = DJISDKManager.getInstance().getProduct();
        ((RCApplication) getApplication()).setTryconnect(true);
        if (!MainActivity.droneName.contains("DJI_Drone"))
            droneName = MainActivity.droneName;

        handler = new Handler(Looper.getMainLooper());
        handlerDisplay = new Handler(Looper.getMainLooper());

        WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        String ip = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());

        Log.d("AMAL", ip);


        if (getBaseProduct().getModel().equals(Model.MATRICE_300_RTK) ) {

            Camera camera = Helper.getCameraInstance();
            if(camera != null){
                camera.exitPlayback(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {

                    }
                });
                cameraPitchToZero();
            }

        }

        btn_service = findViewById(R.id.btn_service);
        activity_main_screen_surface = findViewById(R.id.activity_main_screen_surface);
        telemetryScrollView = findViewById(R.id.telemetryScrollView);
        textQualityRate = findViewById(R.id.textQualityRate);

        //Nav Drawer Related
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        DrawerLayout drawer = findViewById(R.id.drawer_layout);
        NavigationView navigationView = findViewById(R.id.nav_view);
        final Menu navMenu = navigationView.getMenu();
        ActionBarDrawerToggle toggle = new ActionBarDrawerToggle(
                this, drawer, toolbar, R.string.navigation_drawer_open, R.string.navigation_drawer_close) {
            @Override
            public void onDrawerClosed(View drawerView) {
                Log.d(TAG, "drawer close");

            }

            @Override
            public void onDrawerOpened(View drawerView) {

                if (getRosEnabled()) {
                    navMenu.findItem(R.id.nav_stop).setTitle("Stop ROS");
                } else {
                    navMenu.findItem(R.id.nav_stop).setTitle("Start ROS");

                }
                // Code here will be triggered once the drawer open as we dont want anything to happen so we leave this blank
                super.onDrawerOpened(drawerView);
            }

            @Override
            public void onDrawerStateChanged(int newState) {
                Log.d(TAG, "drawer state changed");

                super.onDrawerStateChanged(newState);
            }


        };
        drawer.addDrawerListener(toggle);
        toggle.syncState();

        calibratingWorkFlow = findViewById(R.id.calibrationWorkflow);
        radarWidget = findViewById(R.id.radarWidget);
        navigationView.setNavigationItemSelectedListener(this);
        View headerView = navigationView.getHeaderView(0);
        drawerImage = headerView.findViewById(R.id.drawerImageView);
        drawerName = headerView.findViewById(R.id.drawer_name);
        drawerIP = headerView.findViewById(R.id.drawer_ip);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        m = ConnectDroneActivity.this;
        act = ConnectDroneActivity.this;
        hideKeyboard();

        detectorActivity = new DetectorActivity();
        phoneIp = MainActivity.phoneIP;

        mDetector = new GestureDetectorCompat(this, new MyGestureListener());

        initUi();

        try {
            getRosClient();
        } catch (NotYetConnectedException e) {
            Log.e(TAG, e.toString());
            FirebaseCrashlytics.getInstance().recordException(new Exception(e.toString()));
            finish();
        } catch (NullPointerException e) {
            Log.e(TAG, e.toString());
            MainActivity.respondToRosDisconnect();
            FirebaseCrashlytics.getInstance().recordException(new Exception(e.toString()));
        }


        DJISampleApplication.getEventBus().register(this); //get responses from connectedROS ROS
        EventBus.getDefault().register(this);
        ButterKnife.bind(this); //helps with xml binding

        msgToLeftScreenText("Incoming Input");
        msgToRightScreenText("Outgoing Telemetry");

        //region Set HomeLocation
        if ((getAircraft().getFlightController().getState().getGPSSignalLevel() != GPSSignalLevel.LEVEL_1 &&
                getAircraft().getFlightController().getState().getGPSSignalLevel() != GPSSignalLevel.LEVEL_0 &&
                getAircraft().getFlightController().getState().getGPSSignalLevel() != GPSSignalLevel.NONE) && !(getAircraft().getFlightController().getState().isFlying())) {
            double firstLat = getAircraft().getFlightController().getState().getAircraftLocation().getLatitude();
            double firstLon = getAircraft().getFlightController().getState().getAircraftLocation().getLongitude();
            LocationCoordinate2D firstLocation = new LocationCoordinate2D(firstLat, firstLon);

            getAircraft().getFlightController().setHomeLocation(firstLocation, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    handleDjiError(djiError);
                    if (djiError == null) {
                        sendError("HomePoint set to current location");
                    }
                }
            });
        }
        //endregion

        notifyStatusChange();

        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        displayWidth = size.x;
        if (displayWidth > 800) {
            ConstraintLayout.LayoutParams params = (ConstraintLayout.LayoutParams) videoFeedViewFPV.getLayoutParams();
            params.height = 400;
            params.width = 600;
            videoFeedViewFPV.setLayoutParams(params);
        }
        radarWidget.initView(act.getApplicationContext(), null, 0);

//        printCpuUsages("Before Starting Lidar PC CPU");

        //region Create Directory for Mission Logging
        SimpleDateFormat sdf = new SimpleDateFormat("ddMMyyHHmmSS");
        String currentDate = sdf.format(new Date());
        dir2 = new File(Environment.getExternalStorageDirectory() + "/RosDJI/MapPics/");
        if (!dir2.isDirectory()) {
            try {
                if (dir2.mkdirs()) {
                    FirebaseCrashlytics.getInstance().log("MapPics Directory created");
                    System.out.println("Directory created");
                } else {
                    FirebaseCrashlytics.getInstance().log("MapPics Directory is not created");
                    System.out.println("Directory is not created");
                }
            } catch (Exception e) {
                e.printStackTrace();
                FirebaseCrashlytics.getInstance().recordException(new Exception(e.toString()));
            }
        }
        dir3 = new File(Environment.getExternalStorageDirectory() + "/RosDJI/MapPics2/");
        if (!dir3.isDirectory()) {
            try {
                if (dir3.mkdirs()) {
                    FirebaseCrashlytics.getInstance().log("MapPics2 Directory created");
                    System.out.println("Directory created");
                } else {
                    FirebaseCrashlytics.getInstance().log("MapPics2 Directory not created");
                    System.out.println("Directory is not created");
                }
            } catch (Exception e) {
                e.printStackTrace();
                FirebaseCrashlytics.getInstance().recordException(new Exception(e.toString()));
            }
        }
        clearPreviousBuildMapFiles();
        byuser = false;
        Intent intent2 = new Intent(this, ExitService.class);
        startService(intent2);
        Log.d("ConnectDroneActivity", "start oncreate");

        BaseProduct product = Helper.getProductInstance();
        if(product instanceof Aircraft){
            if(((Aircraft) product).getLidars() != null){
                if(((Aircraft) product).getLidars().get(0).isConnected()){
                    showToast("Connected");
                    currentLidar = ((Aircraft) product).getLidars().get(0);
                }
            }else{

                if (getBaseProduct().getModel().equals(Model.MATRICE_300_RTK) ) {
                    showToast("No camera has been detected!");
                }

            }
        }

        if(currentLidar != null){
            currentLidar.setPointCloudReturnMode(SettingsDefinitions.DJILidarPointCloudReturnMode.TRIPLE_RETURN, new CommonCallbacks.CompletionCallback(){
                @Override
                public void onResult(DJIError djiError)
                {
                    if (djiError == null) {
//                        showToast("Set Return Mode: success");
                    }else {
                        showToast(djiError.getDescription());
                    }
                }
            });

            currentLidar.setPointCloudScanMode(SettingsDefinitions.DJILidarPointCloudScanMode.REPETITIVE, new CommonCallbacks.CompletionCallback(){
                @Override
                public void onResult(DJIError djiError)
                {
                    if (djiError == null) {
//                        showToast("Set Scan Mode: success");
                    }else {
                        showToast(djiError.getDescription());
                    }
                }
            });

            currentLidar.setPointCloudSamplingRate(DJILidarPointCloudSamplingRate.RATE_160K_HZ, new CommonCallbacks.CompletionCallback(){
                @Override
                public void onResult(DJIError djiError)
                {
                    if (djiError == null) {
//                        showToast("Set Sampling Rate: success");
                    }else {
                        showToast(djiError.getDescription());
                    }
                }
            });

            currentLidar.setPointCloudVisibleLightPixel(DJILidarPointCloudVisibleLightPixelMode.VISIBLE_LIGHT_PIXEL_ON_TIMED_SHOT_ON, new CommonCallbacks.CompletionCallback(){
                @Override
                public void onResult(DJIError djiError)
                {
                    if (djiError == null) {
//                        showToast("Set RGB color On: success");
                    }else {
                        showToast(djiError.getDescription());
                    }
                }
            });

            startLidarListener();
        }
    }



    public void printCpuUsages(String tag)
    {
        try
        {
            RandomAccessFile reader = new RandomAccessFile("/proc/stat", "r");
            String load = reader.readLine();
            while (load != null)
            {
                Log.d(tag, "CPU usage: " + load);
                load = reader.readLine();

                String[] toks = load.split(" ");

                long idle1 = Long.parseLong(toks[5]);
                long cpu1 = Long.parseLong(toks[2]) + Long.parseLong(toks[3])
                        + Long.parseLong(toks[4]) + Long.parseLong(toks[6])
                        + Long.parseLong(toks[7]) + Long.parseLong(toks[8]);

                Log.d(TAG, "line1 -- " + load);
                Log.d(TAG, "idle1 -- " + idle1);
                Log.d(TAG, "cpu1 -- " + cpu1);

                try {
                    Thread.sleep(360);
                } catch (Exception e) {
                }

                reader.seek(0);
                load = reader.readLine();
                reader.close();

                toks = load.split(" ");

                long idle2 = Long.parseLong(toks[5]);
                long cpu2 = Long.parseLong(toks[2]) + Long.parseLong(toks[3])
                        + Long.parseLong(toks[4]) + Long.parseLong(toks[6])
                        + Long.parseLong(toks[7]) + Long.parseLong(toks[8]);

                Log.d(TAG, "line2 -- " + load);
                Log.d(TAG, "idle2 -- " + idle2);
                Log.d(TAG, "cpu2 -- " + cpu2);

                Log.d(TAG,
                        "usage -- "
                                + Long.toString((cpu2 - cpu1) * 100
                                / ((cpu2 + idle2) - (cpu1 + idle1))));

                long usage = cpu2 - cpu1;
                long total = (cpu2 + idle2) - (cpu1 + idle1);

                Log.d("Return cpu", " " + (int) (usage * 100 / total));
            }
        }
        catch (IOException ex)
        {
            ex.printStackTrace();
        }
    }


    public void startLidarListener(){

        LidarLiveListener = new Lidar.DJIPointCloudLiveDataListener() {

            @RequiresApi(api = Build.VERSION_CODES.N)
            @Override
            public void onReceiveLiveViewData(PointCloudLiveViewData[] pointCloudLiveViewData, int i) {
                System.out.println("Enter method");
//                    i = 1050;
                DJILidarLiveViewDataProcessor.getInstance().addPointCloudLiveViewData(pointCloudLiveViewData, i);
                Log.d("LIVE", pointCloudLiveViewData.toString());

                try {

                    PointCloudMsg pointCloudMsg = new PointCloudMsg();

                    for (int j = 0; j < i; j++) {

                        pointCloudString += pointCloudLiveViewData[j].getX() + "," + pointCloudLiveViewData[j].getZ() + ","
                                +pointCloudLiveViewData[j].getY() + "," + pointCloudLiveViewData[j].getRed()
                                + "," + pointCloudLiveViewData[j].getGreen() + "," + pointCloudLiveViewData[j].getBlue() + "|";

                        CameraDrone cameraDrone = new CameraDrone();
                        cameraDrone.cameras.get(0).setStorageStateCallBack(new StorageState.Callback() {
                            @Override
                            public void onUpdate(@NonNull @NotNull StorageState storageState) {
                                storageLocation = storageState.getStorageLocation();

                            }
                        });

                        File file = new File(Environment.getExternalStoragePublicDirectory(
                                String.valueOf(storageLocation)).getAbsolutePath());

                        Log.d("Deleted: " ,   " from " + file);

                    }

//                    printCpuUsages("Before Sending PC CPU");
                    sendPointCloud(pointCloudString);
//                    printCpuUsages("After Sending PC CPU");
                    Log.d("XYZ", pointCloudString);
                    pointCloudString = "";

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onError(DJIError djiError) {
                Log.d("dji error",djiError.getDescription());

            }
        };
    }


    public static void sendPointCloud(final String values) {
        //if (getRosEnabled()) {
            Thread thread = new Thread() {
                @Override
                public void run() {
                    try {
                        // send msg
                        String data = jStart() + jMsg("PointCloud:", values) + jEnd();
                        String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[14]) + "\",\"msg\":{" + data + "}}";
                        send(msg);

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            };
            thread.start();
    }


    public static void sendRangeFinder(final String values) {
        Thread thread = new Thread() {
            @Override
            public void run() {
                try {
                    // send msg
                    String data = jStart() + jMsg("RangeFinder:", values) + jEnd();
                    String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[16]) + "\",\"msg\":{" + data + "}}";
                    Log.d("RangeFinder: ", values);
                    send(msg);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        };
        thread.start();
    }


    @Override
    public boolean onNavigationItemSelected(@NonNull MenuItem item) {
        FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected");
        int id = item.getItemId();
        final MenuItem item1 = item;
        switch (id) {
            case R.id.download:

                try {
                    boolean isflying = ((Aircraft) Helper.getProductInstance()).getFlightController().getState().isFlying();
                    if (isflying) {
                        new Helper().showDialog(this, "Land aircraft to download photos", "WARNING", false);
                    } else {
                        MediaDownload mediaDownload = new MediaDownload(this, null);
                        mediaDownload.MediaDownloadPhotos();

                    }
                } catch
                (Exception e) {

                }
                break;

            case R.id.nav_start_stream:
                //region LiveStream
                if (rtmpStream) {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected stopStream");
                    DJISDKManager.getInstance().getLiveStreamManager().stopStream();
                    liveTxt.setVisibility(View.INVISIBLE);
                    rtmpStream = false;
                    item1.setTitle("Start Stream");
                } else {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected startStream");
                    if (DJISDKManager.getInstance().getLiveStreamManager() != null) {
                        AlertDialog.Builder builder = new AlertDialog.Builder(this);
                        LayoutInflater inflater = getLayoutInflater();
                        View dialogView = inflater.inflate(R.layout.edtxt_stream_alert, null);
                        Spinner resolution = dialogView.findViewById(R.id.resolution);

                        List<String> resolutions = Arrays.asList("1920 x 1080", "1440 x 1080", "1280 x 720", "1280 x 960", "960 x 720", "960 x 540", "720 x 540", "480 x 360");
                        List<LiveVideoResolution> resolutionsDJI = Arrays.asList(LiveVideoResolution.VIDEO_RESOLUTION_1920_1080, LiveVideoResolution.VIDEO_RESOLUTION_1440_1080, LiveVideoResolution.VIDEO_RESOLUTION_1280_720, LiveVideoResolution.VIDEO_RESOLUTION_1280_960, LiveVideoResolution.VIDEO_RESOLUTION_960_720, LiveVideoResolution.VIDEO_RESOLUTION_960_540, LiveVideoResolution.VIDEO_RESOLUTION_720_540, LiveVideoResolution.VIDEO_RESOLUTION_480_360);

                        HashMap<String, LiveVideoResolution> mapping = new HashMap<>();
                        int counter = 0;
                        for (String itemR : resolutions) {
                            mapping.put(itemR, resolutionsDJI.get(counter));
                            counter++;
                        }


                        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_dropdown_item, resolutions);

                        resolution.setAdapter(adapter);

                        // Specify alert dialog is not cancelable/not ignorable
                        builder.setCancelable(true);

                        // Set the custom layout as alert dialog view
                        builder.setView(dialogView);

                        // Get the custom alert dialog view widgets reference
                        Button btn_positive = dialogView.findViewById(R.id.streamdialog_positive_btn);
                        Button btn_negative = dialogView.findViewById(R.id.streamdialog_negative_btn);
                        final EditText et_name = dialogView.findViewById(R.id.streamUrl);
                        et_name.setText("rtmp://" + masterIP + "/live/" + droneName);
                        // Create the alert dialog
                        final AlertDialog dialog = builder.create();

                        // Set CUSTOM RTMP button click listener
                        btn_positive.setOnClickListener(new View.OnClickListener() {
                            @Override
                            public void onClick(View v) {


                                final String url = et_name.getText().toString();

                                hideKeyboard();

                                streamURl = url;
                                LiveStreamManager liveStreamManager = DJISDKManager.getInstance().getLiveStreamManager();
                                liveStreamManager.setLiveUrl(streamURl);
                                liveStreamManager.setAudioStreamingEnabled(true);
                                liveStreamManager.setAudioMuted(true);

                                ///testing
                                Log.d("ConnectDroneActivity", "strwaming resolution: " + resolution.getSelectedItem().toString());
                                liveStreamManager.setLiveVideoResolution(mapping.get(resolution.getSelectedItem().toString()));
                                liveStreamManager.setLiveVideoBitRateMode(LiveVideoBitRateMode.AUTO);
                                liveStreamManager.setAudioStreamingEnabled(false);

                                int result = liveStreamManager.startStream();
                                DJISDKManager.getInstance().getLiveStreamManager().setStartTime();
                                View actView = findViewById(R.id.activity_connect_drone);
                                final Snackbar snackbar = Snackbar.make(actView, "startLive:" + result +
                                        " isVideoStreamSpeedConfigurable:" + liveStreamManager.isVideoStreamSpeedConfigurable() +
                                        " isLiveAudioEnabled:" + liveStreamManager.isLiveAudioEnabled() + " resolution:" + liveStreamManager.getLiveVideoResolution().getWidth() + "x" + liveStreamManager.getLiveVideoResolution().getHeight(), Snackbar.LENGTH_LONG);
                                snackbar.show();

                                showToast(liveStreamManager.isStreaming() + "");
                                if (result == 0) {
                                    rtmpStream = true;
                                    item1.setTitle("Stop Stream");
                                }
                                hideKeyboard();
                                dialog.dismiss();
                            }
                        });

                        // Set YOUTUBE button click listener
                        btn_negative.setOnClickListener(new View.OnClickListener() {
                            @Override
                            public void onClick(View v) {
                                if (!rtmpStream) {
                                    streamURl = "rtmp://a.rtmp.youtube.com/live2/6awr-yeqm-k6a2-5f6x";

                                    DJISDKManager.getInstance().getLiveStreamManager().setLiveUrl(streamURl);
                                    DJISDKManager.getInstance().getLiveStreamManager().setVideoSource(LiveStreamManager.LiveStreamVideoSource.Primary);
                                    DJISDKManager.getInstance().getLiveStreamManager().setAudioMuted(true);
                                    int result = DJISDKManager.getInstance().getLiveStreamManager().startStream();
                                    DJISDKManager.getInstance().getLiveStreamManager().setVideoEncodingEnabled(true);
                                    DJISDKManager.getInstance().getLiveStreamManager().setStartTime();

                                    View actView = findViewById(R.id.activity_connect_drone);
                                    final Snackbar snackbar = Snackbar.make(actView, "startLive:" + result +
                                            " isVideoStreamSpeedConfigurable:" + DJISDKManager.getInstance().getLiveStreamManager().isVideoStreamSpeedConfigurable() +
                                            " isLiveAudioEnabled:" + DJISDKManager.getInstance().getLiveStreamManager().isLiveAudioEnabled(), Snackbar.LENGTH_LONG);
                                    snackbar.show();
                                    showToast(DJISDKManager.getInstance().getLiveStreamManager().isStreaming() + "");
                                    if (result == 0) {
                                        rtmpStream = true;
                                        item1.setTitle("Stop Stream");
                                    }
                                } else {
                                    DJISDKManager.getInstance().getLiveStreamManager().stopStream();
                                    rtmpStream = false;
                                    item1.setTitle("Start Stream");
                                }
                                dialog.dismiss();
                            }
                        });

                        dialog.show();
                    }
                }
                //endregion LiveStream
                break;

            case R.id.nav_camera_source:
                if (!getBaseProduct().getModel().equals(Model.MATRICE_300_RTK) && !getBaseProduct().getModel().equals(Model.MAVIC_2_ENTERPRISE_ADVANCED) ){
                    showToast("Cannot Change source for this Drone");
                }else{
                    BaseProduct product = Helper.getProductInstance();
                    if (product != null && product.isConnected()) {
                        if (product instanceof Aircraft)
                        {

                            Camera camera = product.getCamera();
                            showToast("Camera: " + camera.getDisplayName());

                            if(camera_mode == 1)
                            {
                                camera.setCameraVideoStreamSource(CameraVideoStreamSource.ZOOM, null);
                                camera_mode = 2;
                            }
                            else if(camera_mode == 2)
                            {
                                camera.setCameraVideoStreamSource(CameraVideoStreamSource.WIDE, null);
                                camera_mode = 3;
                            }
                            else if(camera_mode == 3)
                            {
                                camera.setCameraVideoStreamSource(CameraVideoStreamSource.INFRARED_THERMAL, null);
                                camera_mode = 1;
                            }
                        }
                    }
                }
                break;
            case R.id.nav_start_lidar:

                if (currentLidar == null){
                    showToast("Lidar is not detected!");
                }else{
                    if(startLidar){
                        currentLidar.addPointCloudLiveViewDataListener(LidarLiveListener);

                        currentLidar.pointCloudRecord(DJILidarPointCloudRecord.START_POINT_CLOUD_RECORDING, new CommonCallbacks.CompletionCallback(){
                            @Override
                            public void onResult(DJIError djiError)
                            {
                                if (djiError == null) {
                                    recordingStatus = RecordingStatus.STARTED;
                                    showToast("Record Lidar: Success");
                                }else {
                                    showToast(djiError.getDescription());
                                }
                                Log.d("Recording", String.valueOf(recordingStatus));
                            }

                        });

                        currentLidar.startReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback(){
                            @Override
                            public void onResult(DJIError djiError)
                            {
                                if (djiError == null) {
                                    showToast("Record Lidar: Success");
                                }else {
                                    Log.d("Error", djiError.getDescription());
                                    showToast(djiError.getDescription());
                                }
                            }
                        });
                        startLidar = false;
                        item1.setTitle("Stop Point Cloud");
                    }else{

                        currentLidar.removePointCloudLiveViewDataListener(LidarLiveListener);

                        currentLidar.pointCloudRecord(DJILidarPointCloudRecord.STOP_POINT_CLOUD_RECORDING, new CommonCallbacks.CompletionCallback(){
                            @Override
                            public void onResult(DJIError djiError)
                            {
                                if (djiError == null) {
                                    recordingStatus = RecordingStatus.STOPPED;
                                    showToast("Stop Record Lidar: Success");
                                    Log.d("Stop Record Lidar", String.valueOf(recordingStatus));

                                }else {
                                    showToast(djiError.getDescription());
                                }
                                Log.d("Recording", String.valueOf(recordingStatus));
                            }
                        });

                        currentLidar.stopReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback(){
                            @Override
                            public void onResult(DJIError djiError)
                            {
                                if (djiError == null) {
                                    showToast("Stop Record Lidar: Success");

                                }else {
                                    showToast(djiError.getDescription());
                                }
                            }
                        });
                        startLidar = true;
                        item1.setTitle("Start Point Cloud");
                    }
                }


                break;
                //End of Lidar Option


                //Start of Range Finder Option
            case R.id.nav_start_range_finder:

                BaseProduct product = FPVDemoApplication.getProductInstance();
                Camera camera = product.getCamera();


                if(startRangeFinder){
                    if(camera == null){
                        showToast("Camera is not detected!");
                    }else{
                        camera.setLaserEnabled(true, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if(djiError == null)
                                {
                                    Lens lens = camera.getLens(0);      // 0-> Zoom , 1-> WIDE , 2->THERMAL

                                    lens.setLaserMeasureInformationCallback(new LaserMeasureInformation.Callback() {
                                        @Override
                                        public void onUpdate(LaserMeasureInformation laserMeasureInformation) {

                                            DecimalFormat coord = new DecimalFormat("###.######");
                                            DecimalFormat alt = new DecimalFormat("####.#");
                                            DecimalFormat dist = new DecimalFormat("#####.###");

                                            try {

                                                PointF pointF = laserMeasureInformation.getTargetPoint();

                                                String targetDistance = dist.format(laserMeasureInformation.getTargetDistance());
                                                sleep(1000);
                                                Log.d("Before Send: ", targetDistance);
                                                //publish
                                                sendRangeFinder(targetDistance);

                                            } catch (Exception e) {
                                                e.printStackTrace();
                                            }
                                        }
                                    });

                                    showToast("Start Range Finder: Success");
                                }
                                else
                                {
                                    startRangeFinder = true;
                                    item1.setTitle("Start Range Finder");
//                                    showToast("Error Getting Lenses: " + djiError.getDescription());
                                    showToast("Camera is not detected!");
                                    Log.d("ERROR GETTING LENSES", djiError.getDescription());
                                }
                            }
                        });

                        startRangeFinder = false;
                        item1.setTitle("Stop Range Finder");
                    }


                }else{
                    camera.setLaserEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError == null)
                            {
                                showToast("Stop Range Finder: Success");
                            }
                            else
                            {
                                showToast("Error Getting Lenses: " + djiError.getDescription());
                                Log.d("ERROR GETTING LENSES", djiError.getDescription());

                            }
                        }
                    });
                    startRangeFinder = true;
                    item1.setTitle("Start Range Finder");
                }
                break;
                //End of Lidar Option


            case R.id.calibrate:
                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected calibrate");
                calibratingWorkFlow.setVisibility(View.VISIBLE);
                startCalibration(new ActionCallback() {
                    @Override
                    public void onSuccess() {
                    }

                    @Override
                    public void onFailure(@NonNull DJIError djiError) {
                    }
                });
                break;

            case R.id.nav_change_stream:
                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected changeStreamSource");
                if (!isLiveStreamManagerOn()) {
                    return true;
                }
                if (!isSupportSecondaryVideo()) {
                    return true;
                }
                if (DJISDKManager.getInstance().getLiveStreamManager().isStreaming()) {
                    showToast("Before change live source, you should stop live stream!");
                    return true;
                }
                currentVideoSource = (currentVideoSource == LiveStreamManager.LiveStreamVideoSource.Primary) ?
                        LiveStreamManager.LiveStreamVideoSource.Secoundary :
                        LiveStreamManager.LiveStreamVideoSource.Primary;
                DJISDKManager.getInstance().getLiveStreamManager().setVideoSource(currentVideoSource);

                showToast("Change Success ! Video Source : " + currentVideoSource.name());
                break;
            case R.id.nav_mute_audio:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected muteAudio");
                if (!isLiveStreamManagerOn()) {
                    return true;
                }
                if (DJISDKManager.getInstance().getLiveStreamManager().isAudioMuted()) {
                    DJISDKManager.getInstance().getLiveStreamManager().setAudioMuted(false);

                    showToast("Audio Unmuted");
                    item1.setTitle("Mute Audio");
                } else {
                    DJISDKManager.getInstance().getLiveStreamManager().setAudioMuted(true);
                    showToast("Audio Muted");
                    item1.setTitle("Unmute Audio");
                }
                break;
            case R.id.nav_detect:


                if (isDetectionActive) {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected stopDetector");
                    isDetectionActive = false;
                    DetectorActivity.trackingOverlay.setVisibility(View.GONE);
                    item.setTitle("Start Detection");
                } else {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected startDetector");
                    isDetectionActive = true;
                    DetectorActivity.trackingOverlay.setVisibility(View.VISIBLE);
                    DetectorActivity.trackingOverlay.bringToFront();
                    item.setTitle("Stop Detection");
                }
                break;
            case R.id.nav_stop:

                if (getRosEnabled())
                    dialogOnCloseActivity();
                else {
                    tryingConnecting();
                }


                break;
            case R.id.showMsgs: {
                if (msg_det.getVisibility() == View.VISIBLE) {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected hideReceivedCommands");
                    msg_det.setVisibility(View.GONE);
                    item.setTitle("Show Received Commands");
                } else {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected showReceivedCommands");
                    msg_det.setVisibility(View.VISIBLE);
                    item.setTitle("Hide Received Commands");
                }
            }
            case R.id.showTelemetry:

                if (telemetryScrollView.getVisibility() == View.VISIBLE) {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected hideTelemetry");
                    telemetryScrollView.setVisibility(View.GONE);
                    item.setTitle("Show Telemetry");
                } else {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected showTelemetry");
                    telemetryScrollView.setVisibility(View.VISIBLE);
                    item.setTitle("Hide Telemetry");
                }
                break;
            case R.id.tipsReset:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected resetTips");
                writeToFile("false", act);
                showToast("Tips Have Been Reset");
                break;
            case R.id.about:


                if (!webViewShowing) {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected showNewKiosWeb");
                    webViewShowing = true;
                    wv.setWebViewClient(new WebViewClient());
                    wv.loadUrl("https://www.kios.ucy.ac.cy");
                    addContentView(wv, new WindowManager.LayoutParams());
                } else {
                    FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected showOldKiosWeb");
                    wv.loadUrl("https://www.kios.ucy.ac.cy");
                    wv.setVisibility(View.VISIBLE);
                }
                break;

            case R.id.virtualSticks:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected setVirtualSticks");
                //Control Mode Sets
                getAircraft().getFlightController().setVerticalControlMode(VerticalControlMode.VELOCITY);
                getAircraft().getFlightController().setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                getAircraft().getFlightController().setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                getAircraft().getFlightController().setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
                showToast(getAircraft().getFlightController().getRollPitchCoordinateSystem().toString());


                if (!obstacleThreadRunning && !virtualSticksThread) {
                    virtualSticksThread = true;
                    obstacleThreadRunning = true;
                    rosPublishObstacleSticksThread();
                }
                break;
            case R.id.btn_simulator:


                if (getFlightController() != null) {
                    if (flightController.getSimulator().isSimulatorActive()) {
                        FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected stopSimulator");
                        flightController.getSimulator().stop(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                handleDjiError(djiError);
                                if (djiError == null) {
                                    ConnectDroneActivity.this.runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            item1.setTitle("Start Simulator");
                                        }
                                    });
                                }
                            }
                        });
                    } else {
                        FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected startSimulator");
                        flightController.getSimulator()
                                .start(InitializationData.createInstance(new LocationCoordinate2D(35.143751, 33.412524), 10, 10),
                                        new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {
                                                handleDjiError(djiError);
                                                if (djiError == null) {
                                                    ConnectDroneActivity.this.runOnUiThread(new Runnable() {
                                                        @Override
                                                        public void run() {
                                                            item1.setTitle("Stop Simulator");
                                                        }
                                                    });
                                                }
                                            }
                                        });
                    }

                }
                break;
            case R.id.mapPicServer:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected setPicSercerIP");
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                LayoutInflater inflater = getLayoutInflater();
                View dialogView = inflater.inflate(R.layout.edtxt_pic_server_alert, null);

                // Specify alert dialog is not cancelable/not ignorable
                builder.setCancelable(true);

                // Set the custom layout as alert dialog view
                builder.setView(dialogView);

                // Get the custom alert dialog view widgets reference
                Button btn_positive = dialogView.findViewById(R.id.save_pic_btn);

                final EditText et_ip = dialogView.findViewById(R.id.et_pic_ip);
                final EditText et_port = dialogView.findViewById(R.id.et_pic_port);
                et_ip.setText(picServerIP);
                et_port.setText(picServerPort + "");
                // Create the alert dialog
                final AlertDialog dialog = builder.create();

                // Set CUSTOM RTMP button click listener
                btn_positive.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {

                        picServerIP = et_ip.getText().toString();
                        picServerPort = Integer.parseInt(et_port.getText().toString());
                        hideKeyboard();
                        dialog.dismiss();
                    }
                });

                dialog.show();
                break;
            case R.id.takeMapPic:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected keepOriginalPhoto");
                keepOriginalPhoto = !keepOriginalPhoto;
                showToast("Keeping original photos while mapping: " + Boolean.toString(keepOriginalPhoto));
                item.setTitle("Save Map Photos: " + Boolean.toString(keepOriginalPhoto));
                break;
            case R.id.showDiagnostics:


                FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onNavigationItemSelected showDiagnostics");
                showDiagnostics = !showDiagnostics;
                showToast("Show Diagnostics: " + Boolean.toString(showDiagnostics));
                item.setTitle("Show Diagnostics: " + Boolean.toString(showDiagnostics));
                break;
            case R.id.toggleRtk:


                getAircraft().getFlightController().getRTK().getRtkEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean) {
                        if (aBoolean) {
                            Objects.requireNonNull(getAircraft().getFlightController().getRTK()).setRtkEnabled(false, new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    handleDjiError(djiError);
                                    if (djiError != null)
                                        sendError(djiError.toString());
                                    else
                                        ConnectDroneActivity.this.runOnUiThread(new Runnable() {
                                            @Override
                                            public void run() {
                                                item.setTitle("Enable RTK");
                                            }
                                        });
                                }
                            });
                        } else {
                            Objects.requireNonNull(getAircraft().getFlightController().getRTK()).setRtkEnabled(true, new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    handleDjiError(djiError);
                                    if (djiError != null)
                                        sendError(djiError.toString());
                                    else
                                        ConnectDroneActivity.this.runOnUiThread(new Runnable() {
                                            @Override
                                            public void run() {
                                                item.setTitle("Disable RTK");
                                            }
                                        });
                                }
                            });
                        }
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        handleDjiError(djiError);
                        if (djiError != null)
                            sendError(djiError.toString());
                    }
                });
                break;
            case R.id.stream_camera:

                final LinearLayout linearLayout2 = (LinearLayout) activityConnect.getLayoutInflater().inflate(R.layout.radio_group, null);
                final AlertDialog.Builder builder2 = new AlertDialog.Builder(activityConnect);

                RadioGroup radioGroup = linearLayout2.findViewById(R.id.radio_group);

                final RadioButton radioButton[] = new RadioButton[3];
                radioButton[0] = new RadioButton(activityConnect);
                radioButton[0].setText("WIDE");

                radioGroup.addView(radioButton[0]);

                radioButton[1] = new RadioButton(activityConnect);
                radioButton[1].setText("ZOOM");

                radioGroup.addView(radioButton[1]);

                radioButton[2] = new RadioButton(activityConnect);
                radioButton[2].setText("THERMAL");

                radioGroup.addView(radioButton[2]);
                radioButton[0].setChecked(true);

                builder2.setView(linearLayout2);
                builder2.setCancelable(true);
                builder2.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        CameraDrone cameraDrone = new CameraDrone();

                        RadioGroup radioGroup1 = linearLayout2.findViewById(R.id.radio_group);
                        int id = radioGroup1.getCheckedRadioButtonId();
                        RadioButton radioButton1 = linearLayout2.findViewById(id);
                        if (radioButton1.getText().toString().equalsIgnoreCase("WIDE")) {
                            cameraDrone.setCameraVideoStreamSource(CameraVideoStreamSource.WIDE, item);

                        } else if (radioButton1.getText().toString().equalsIgnoreCase("ZOOM"))
                            cameraDrone.setCameraVideoStreamSource(CameraVideoStreamSource.ZOOM, item);
                        else
                            cameraDrone.setCameraVideoStreamSource(CameraVideoStreamSource.INFRARED_THERMAL, item);

                    }
                });
                builder2.show();

                break;


        }

        DrawerLayout drawer = findViewById(R.id.drawer_layout);
        drawer.closeDrawer(GravityCompat.START);
        return true;
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    private boolean isLiveStreamManagerOn() {
        if (DJISDKManager.getInstance().getLiveStreamManager() == null) {
            showToast("No live stream manager!");
            return false;
        }
        return true;
    }

    private boolean isSupportSecondaryVideo() {
        try {
            if (!Helper.isMultiStreamPlatform()) {
                showToast("No secondary video!");
                return false;
            }
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return true;
    }

    private void handleStreamRequest(boolean start) {
        if (!start) {
            DJISDKManager.getInstance().getLiveStreamManager().stopStream();
            liveTxt.setVisibility(View.INVISIBLE);
            rtmpStream = false;
        } else {
            if (DJISDKManager.getInstance().getLiveStreamManager() != null) {
                streamURl = "rtmp://" + masterIP + "/live/" + droneName;

                DJISDKManager.getInstance().getLiveStreamManager().setLiveUrl(streamURl);
                DJISDKManager.getInstance().getLiveStreamManager().setAudioStreamingEnabled(true);
                DJISDKManager.getInstance().getLiveStreamManager().setAudioMuted(true);

                ///testing
                DJISDKManager.getInstance().getLiveStreamManager().setLiveVideoResolution(LiveVideoResolution.VIDEO_RESOLUTION_960_720);
                DJISDKManager.getInstance().getLiveStreamManager().setLiveVideoBitRateMode(LiveVideoBitRateMode.AUTO);
                DJISDKManager.getInstance().getLiveStreamManager().setAudioStreamingEnabled(false);

                int result = DJISDKManager.getInstance().getLiveStreamManager().startStream();
                DJISDKManager.getInstance().getLiveStreamManager().setStartTime();
                View actView = findViewById(R.id.activity_connect_drone);

                if (result == 0) {
                    rtmpStream = true;
                }
            }
        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        rosSubscribe(true); //subscribe to the topics
        buttonEnable(true);
        loopForProductConnection(); //get connection to aircraft
        hideKeyboard();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                handleStreamRequest(true);
            }
        }, 2000);
        try {
            getAircraft().getCamera().setStorageLocation(SettingsDefinitions.StorageLocation.SDCARD, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    handleDjiError(djiError);
                }
            });
        } catch (NullPointerException e) {
            Log.d(TAG, e.toString());
        }
    }

    private void tryingConnectingToROS() {

        cameraDrone.getCameraVideoStreamSource();
        handshakeconnected = false;
        TimerTask timerTask = new TimerTask()//wait until  received confirmed message from ros
        {
            public void run() {
                if (!handshakeconnected && getRosEnabled()) {
                    //  Log.d("ConnectDroneActivity", "resend to master that drone is connected");

                    if (!cameraDrone.isH20T())
                        sendDroneIds(true, cameraDrone);
                    else if (cameraDrone.getCameraSourceMode() != null)
                        sendDroneIds(true, cameraDrone);

                } else {
                    getRemoteLogger().log(3, "tryingConnectingToROS: connected successfully to ROS: " + droneName);

                    Log.d("ConnectDroneActivity", "droneIds publisher abort");
                    cancel();
                }

            }
        };

        timerConnectToRos = new Timer();
        timerConnectToRos.schedule(timerTask, 0, 500);
    }

    @Override
    protected void onResume() {
        getRemoteLogger().log(Log.DEBUG, "ConnectDroneActivity: onResume");
        getRemoteLogger().log(Log.DEBUG, "ConnectDroneActivity: activityPaused: " + activityPaused);

        Log.d("ConnectDroneActivity", "onResume: onResume");

        MainActivity.ConnectActivityActive = true;
        active = true;
        if (!getBaseProduct().getModel().equals(Model.MATRICE_300_RTK))
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
        notifyStatusChange();
        enableServices();

        Display display = ((WindowManager) getApplicationContext().getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay();
        int rotation = display.getRotation();

        Log.d("ORIENTATION", rotation + "");

        if(rotation == 3){
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
        }

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        rosStop = false;
        hideKeyboard();
        if (!activityPaused) {}
        super.onResume();

    }

    @Override
    protected void onDestroy() {
        Log.d("ConnectDroneActivity", "on: onDestroy");

        super.onDestroy();
        sendDroneIds(false, null);
        Log.d("ConnectDroneActivity", "onDestroy");


        active = false;
        EventBus.getDefault().unregister(this);
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        client.disconnect();
    }

    @Override
    protected void onStop() {
        getRemoteLogger().log(Log.DEBUG, "ConnectDroneActivity: onstop");

        Log.d("ConnectDroneActivity", "onstop: " + this.isDestroyed());
        super.onStop();
        handleStreamRequest(false);
        active = false;
        telemetry = false; //to stop the thread that sends telemetry
        detection = false;
        battery = false;
        activityPaused = false;

        rosSubscribe(false);
        //remove all callback from handlers
        handler.removeCallbacksAndMessages(null);
        handlerDisplay.removeCallbacksAndMessages(null);
        //disconnect certain aircraft callbacks
        if (getAircraft() != null) {
            FlightAssistant intelligentFlightAssistant = getAircraft().getFlightController().getFlightAssistant();
            if (intelligentFlightAssistant != null) {
                intelligentFlightAssistant.setVisionDetectionStateUpdatedCallback(null);
            }

        }
    }

    private void dialogOnCloseActivity() {
        ((RCApplication) getApplication()).setTryconnect(false);

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("Exit App?")
                .setCancelable(false)
                .setPositiveButton("Yes", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        FirebaseCrashlytics.getInstance().log("ConnectDroneActivity onBackPressed stopROS");
                        SharedPreferences.setBooleanSharedPreference(ConnectDroneActivity.getActivity(), "byuser", true);

                        new Handler().postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                closeActivity();
                                ConnectDroneActivity.this.onBackPressed();

                            }
                        }, 500);
                        dialog.dismiss();
                    }
                })
                .setNegativeButton("No", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                    }
                });
        AlertDialog alert = builder.create();
        alert.show();
    }

    @Override
    public void onBackPressed() {
        dialogOnCloseActivity();
    }
    public static void hideKeyboard() {
        InputMethodManager imm = (InputMethodManager) m.getSystemService(Activity.INPUT_METHOD_SERVICE);
        //Find the currently focused view, so we can grab the correct window token from it.
        View view = m.getCurrentFocus();
        //If no view currently has focus, create a new one, just so we can grab a window token from it
        if (view == null) {
            view = new View(m);
        }
        try {
            if (imm != null) {
                imm.hideSoftInputFromWindow(view.getWindowToken(), 0);
            }
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }

    static boolean byuser = true;
    static boolean calledBeforeclose = false;

    public static void beforeClose() {
        Log.d(TAG, "beforeClose(): " + calledBeforeclose);
        if (calledBeforeclose) {
            return;
        }

        SharedPreferences.setIntSharedPreference(activityConnect, "photos_mission", Mission.getNum_media());

        Mission.inProgress = false;
        calledBeforeclose = true;
        getRemoteLogger().log(1, "beforeClose run");

        if (cameraDrone != null)
            cameraDrone.stopReadPointCloudLiveViewData();
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    sendError("Ros closing...");
                    active = false;

                    sendDroneIds(false, null);
                    Log.d(TAG, "Sending Disconnected Message to ROS ");
                    setRosEnabled(false);//to stop threads from publishing
                    telemetry = false; //to stop the thread that sends telemetry

                    if (timerConnectToRos != null) {
                        timerConnectToRos.cancel();
                    }
                    if (timerTelemetry != null)
                        timerTelemetry.cancel();
                    if (timerPictures != null)
                        timerPictures.cancel();
                    if (timerObstacle != null)
                        timerObstacle.cancel();

                    battery = false; //to stop the thread that updates the battery txt
                    detection = false; //to stop the thread that sends detection msgs
                    rosSubscribe(false);//to unsubscribe from ros topics
                    setRosEnabled(false);
                    setRosConnected(false);

                    //remove all callback from handlers
                    handler.removeCallbacksAndMessages(null);
                    handlerDisplay.removeCallbacksAndMessages(null);

                    Mission.removeWaypointMissionOperator();

                    //disconnect certain aircraft callbacks
                    if (getAircraft() != null) {
                        FlightAssistant intelligentFlightAssistant = getAircraft().getFlightController().getFlightAssistant();
                        if (intelligentFlightAssistant != null) {
                            intelligentFlightAssistant.setVisionDetectionStateUpdatedCallback(null);
                        }

                    }

                    active = false;
                    EventBus.getDefault().unregister(this);
                    if (client != null)
                        client.disconnect();

                } catch (Exception e) {
                    try {
                        getRemoteLogger().log(6, "beforeClose: " + getStackTrace(e));
                    } catch (Exception e2) {

                    }
                    FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                    Log.e(TAG, e.toString());
                }

            }
        });
        try {
            thread.start();
        } catch (ExceptionInInitializerError | NullPointerException e) {
            getRemoteLogger().log(6, "beforeClose: " + getStackTrace((Exception) e));

            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
            e.printStackTrace();
        }
        Log.d(TAG, "stopping threads");

    }


    public static void closeActivity() {
        if (timerDisplaydata != null) {
            timerDisplaydata.cancel();
            timerDisplaydata = null;
        }
        Log.d("ConnectDroneActivity", "closeActivity");
        getRemoteLogger().log(1, "closeActivity run-> byuser:" + SharedPreferences.getBooleanSharedPreference(getActivity(), "byuser"));
        beforeClose();
    }

    class MyGestureListener extends GestureDetector.SimpleOnGestureListener {

        @Override
        public boolean onDoubleTap(MotionEvent event) {
            showToast("Double Tapped");
            return true;
        }
    }

    private GestureDetectorCompat mDetector;
    private int currentZoomFocalLenght;
    private int zoomFocalLenghtStep;
    private int maxZoomFocalLenght;
    private int minZoomFocalLenght;
    private VideoFeedViewPrimary videoFeedViewPrimary;

    @SuppressLint("ClickableViewAccessibility")
    private void initUi() {

        msg_det = findViewById(R.id.msg_det);
        sub_info = findViewById(R.id.sub_info);
        liveTxt = findViewById(R.id.liveTxt);
        try {
            if (getBaseProduct().getModel().equals(Model.MATRICE_300_RTK))
                getAircraft().getAirLink().getOcuSyncLink().assignSourceToPrimaryChannel(PhysicalSource.LEFT_CAM, PhysicalSource.FPV_CAM, new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        handleDjiError(djiError);
                    }
                });
        } catch (NullPointerException e) {

        }
        videoFeedViewPrimary = findViewById(R.id.livestream_video_feed);
        videoFeedViewPrimary.registerLiveVideo(VideoFeeder.getInstance().getPrimaryVideoFeed(), true);

        videoFeedViewFPV = findViewById(R.id.livestream_fpv_feed);
        videoFeedViewFPV.registerLiveVideo(VideoFeeder.getInstance().getSecondaryVideoFeed(), false);
        if (Helper.isMultiStreamPlatform()) {
            videoFeedViewFPV.setVisibility(View.VISIBLE);
            videoFeedViewFPV.bringToFront();
        }
        videoFeedViewFPV.setClickable(true);
        videoFeedViewFPV.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                MainActivity.mDetector.onTouchEvent(event);
                return false;
            }
        });

        //region zoom_button_listeners
        zoomLayout = findViewById(R.id.zoomLayout);
        zoomIn = findViewById(R.id.zoomIn);
        zoomOut = findViewById(R.id.zoomOut);
        final boolean[] zoom = {true};
        zoomIn.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, final MotionEvent event) {
                handler.post(new Runnable() {
                    @SuppressWarnings("deprecation")
                    @Override
                    public void run() {
                        switch (event.getAction()) {
                            case MotionEvent.ACTION_DOWN:
                                zoomIn.setBackground(getResources().getDrawable(R.drawable.round_btn_normal));
                                if (mCamera != null && mCamera.isHybridZoomSupported()) {
                                    mCamera.getHybridZoomFocalLength(new CommonCallbacks.CompletionCallbackWith<Integer>() {
                                        @Override
                                        public void onSuccess(Integer integer) {
                                            currentZoomFocalLenght = integer;
                                        }

                                        @Override
                                        public void onFailure(DJIError djiError) {
                                            handleDjiError(djiError);
                                        }
                                    });
                                    handler.postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            int newZoomLength = currentZoomFocalLenght + zoomFocalLenghtStep;
                                            if (newZoomLength >= maxZoomFocalLenght) {
                                                newZoomLength = maxZoomFocalLenght;
                                                showToast("Max Zoom");
                                            }
                                            mCamera.setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                                @Override
                                                public void onResult(DJIError djiError) {
                                                    handleDjiError(djiError);
                                                }
                                            });
                                            zoomIn.setBackground(getResources().getDrawable(R.drawable.round_btn_disable));
                                        }
                                    }, 50);
                                } else if (mCamera.getDisplayName().equals(Camera.DisplayNameZenmuseH20T)) {
                                    mCamera.getLens(0).getHybridZoomFocalLength(new CommonCallbacks.CompletionCallbackWith<Integer>() {
                                        @Override
                                        public void onSuccess(Integer integer) {
                                            currentZoomFocalLenght = integer;
                                        }

                                        @Override
                                        public void onFailure(DJIError djiError) {
                                            handleDjiError(djiError);
                                        }
                                    });
                                    handler.postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            int newZoomLength = currentZoomFocalLenght + zoomFocalLenghtStep;
                                            if (newZoomLength >= maxZoomFocalLenght) {
                                                newZoomLength = maxZoomFocalLenght;
                                                showToast("Max Zoom");
                                            }
                                            mCamera.getLens(0).setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                                @Override
                                                public void onResult(DJIError djiError) {
                                                    handleDjiError(djiError);
                                                }
                                            });
                                            zoomIn.setBackground(getResources().getDrawable(R.drawable.round_btn_disable));
                                        }
                                    }, 50);
                                }
                                break;
                            default:
                                break;
                        }
                    }
                });
                return false;
            }
        });

        zoomOut.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, final MotionEvent event) {
                handler.post(new Runnable() {
                    @SuppressWarnings("deprecation")
                    @Override
                    public void run() {
                        switch (event.getAction()) {
                            case MotionEvent.ACTION_DOWN:
                                if (mCamera != null && mCamera.isHybridZoomSupported()) {
                                    zoomOut.setBackground(getResources().getDrawable(R.drawable.round_btn_normal));
                                    mCamera.getHybridZoomFocalLength(new CommonCallbacks.CompletionCallbackWith<Integer>() {
                                        @Override
                                        public void onSuccess(Integer integer) {
                                            currentZoomFocalLenght = integer;
                                        }

                                        @Override
                                        public void onFailure(DJIError djiError) {
                                            handleDjiError(djiError);
                                        }
                                    });
                                    handler.postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            int newZoomLength = currentZoomFocalLenght - zoomFocalLenghtStep;
                                            if (newZoomLength <= minZoomFocalLenght) {
                                                newZoomLength = minZoomFocalLenght;
                                                showToast("Min Zoom");
                                            }
                                            mCamera.setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                                @Override
                                                public void onResult(DJIError djiError) {
                                                    handleDjiError(djiError);
                                                }
                                            });
                                            zoomOut.setBackground(getResources().getDrawable(R.drawable.round_btn_disable));
                                        }
                                    }, 50);
                                } else if (mCamera.getDisplayName().equals(Camera.DisplayNameZenmuseH20T)) {
                                    zoomOut.setBackground(getResources().getDrawable(R.drawable.round_btn_normal));
                                    mCamera.getLens(0).getHybridZoomFocalLength(new CommonCallbacks.CompletionCallbackWith<Integer>() {
                                        @Override
                                        public void onSuccess(Integer integer) {
                                            currentZoomFocalLenght = integer;
                                        }

                                        @Override
                                        public void onFailure(DJIError djiError) {
                                            handleDjiError(djiError);
                                        }
                                    });
                                    handler.postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            int newZoomLength = currentZoomFocalLenght - zoomFocalLenghtStep;
                                            if (newZoomLength <= minZoomFocalLenght) {
                                                newZoomLength = minZoomFocalLenght;
                                                showToast("Min Zoom");
                                            }
                                            mCamera.getLens(0).setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                                @Override
                                                public void onResult(DJIError djiError) {
                                                    handleDjiError(djiError);
                                                }
                                            });
                                            zoomOut.setBackground(getResources().getDrawable(R.drawable.round_btn_disable));
                                        }
                                    }, 50);
                                }
                                break;
                            default:
                                break;
                        }
                    }
                });
                return false;
            }
        });
        //endregion

        setBatteryCallback();
        setRemainingFlightTimeChecker();

        //region DrawerImage
        drawerName.setText("Drone ID: " + droneName);
        drawerName.setClickable(true);
        drawerName.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (detectRateinHz == 20) {
                    detectRateinHz = 30;
                    showToast("Detection rate changed to 30");
                } else if (detectRateinHz == 30) {
                    detectRateinHz = 40;
                    showToast("Detection rate changed to 40");
                } else if (detectRateinHz == 40) {
                    detectRateinHz = 20;
                    showToast("Detection rate changed to 20");
                }
            }
        });
        drawerIP.setText("Phone IP: " + phoneIp);
        try {
            switch (getAircraft().getModel()) {
                case Spark:
                    drawerImage.setImageResource(R.drawable.spark);
                    break;
                case MAVIC_PRO:
                    drawerImage.setImageResource(R.drawable.mavicp);
                    break;
                case MATRICE_100:
                    drawerImage.setImageResource(R.drawable.m100);
                    break;
                case MAVIC_2_PRO:
                    drawerImage.setImageResource(R.drawable.mavic2p);
                    break;
                case MAVIC_2_ZOOM:
                    drawerImage.setImageResource(R.drawable.mavic2z);
                    break;
                case MAVIC_2_ENTERPRISE:
                    drawerImage.setImageResource(R.drawable.mavic2e);
                    break;
                case MAVIC_2_ENTERPRISE_DUAL:
                    drawerImage.setImageResource(R.drawable.mavic2ed);
                    break;
                case MATRICE_210:
                    drawerImage.setImageResource(R.drawable.m210);
                    break;
                case MATRICE_210_RTK:
                    drawerImage.setImageResource(R.drawable.m210rtk);
                    break;
                case MATRICE_300_RTK:
                    drawerImage.setImageResource(R.drawable.m300rtk);
                    break;
                default:
                    drawerImage.setImageResource(R.drawable.kiosfb1);
                    break;
            }
        } catch (NullPointerException e) {
            e.printStackTrace();
            drawerImage.setImageResource(R.drawable.kiosfb1);
        }
        //endregion DrawerImage
        wv = new WebView(getApplicationContext());

        //region Tips Alert Dialog Builder
        handler.post(new Runnable() {
            @Override
            public void run() {
                if (!readFromFile(act).equals("true")) {
                    AlertDialog.Builder builder = new AlertDialog.Builder(act);
                    LayoutInflater inflater = getLayoutInflater();
                    View dialogView = inflater.inflate(R.layout.checkbox_alert, null);
                    // Specify alert dialog is not cancelable/not ignorable
                    builder.setCancelable(true);

                    // Set the custom layout as alert dialog view
                    builder.setView(dialogView);

                    // Get the custom alert dialog view widgets reference
                    final Button btn_positive = dialogView.findViewById(R.id.dialog_positive_btn);
                    final TextView alertTxt = dialogView.findViewById(R.id.dialog_title);
                    final CheckBox checkBox = dialogView.findViewById(R.id.checkbox);

                    // Create the alert dialog
                    final AlertDialog dialog = builder.create();

                    // Set positive/yes button click listener
                    btn_positive.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            hideKeyboard();
                            if (alertState == 0) {
                                alertState = 1;
                                btn_positive.setText("Dismiss");
                                alertTxt.setText(getResources().getString(R.string.tips2));
                            } else if (alertState == 1) {
                                alertState = 2;
                                final boolean ischecked = checkBox.isChecked();

                                if (ischecked) {
                                    writeToFile("true", act);
                                }
                                dialog.dismiss();
                            }
                        }
                    });


                    dialog.show();
                }
            }
        });
        //endregion Tips Alert Dialog Builder

        try {
            FirebaseCrashlytics.getInstance().log(TAG + " set virtual sticks modes");
            getAircraft().getFlightController().setVerticalControlMode(VerticalControlMode.VELOCITY);
            getAircraft().getFlightController().setYawControlMode(YawControlMode.ANGLE);
            getAircraft().getFlightController().setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            getAircraft().getFlightController().setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        } catch (Exception e) {
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
            Log.e(TAG, e.toString());
        }

        if (getBaseProduct() != null && getBaseProduct().getModel().equals(Model.MATRICE_300_RTK)) {
            NavigationView navigationView = findViewById(R.id.nav_view);
            Menu navMenu = navigationView.getMenu();
            //  MenuItem lensItem = navMenu.findItem(R.id.changeH20Tlens);
            MenuItem rtkItem = navMenu.findItem(R.id.toggleRtk);
            // lensItem.setVisible(true);
            rtkItem.setVisible(true);
            zoomLayout.setVisibility(View.VISIBLE);
        }

    }

    public static void changeViewSize() {
        if (!fpvViewFullscreen) {
            RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) videoFeedViewFPV.getLayoutParams();
            params.height = RelativeLayout.LayoutParams.MATCH_PARENT;
            params.width = RelativeLayout.LayoutParams.MATCH_PARENT;
            videoFeedViewFPV.setLayoutParams(params);
            fpvViewFullscreen = true;
            msg_det.bringToFront();
            sub_info.bringToFront();
        } else {
            if (displayWidth > 800) {
                RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) videoFeedViewFPV.getLayoutParams();
                params.height = 400;
                params.width = 600;
                videoFeedViewFPV.setLayoutParams(params);
            } else {
                RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) videoFeedViewFPV.getLayoutParams();
                params.height = 200;
                params.width = 300;
                videoFeedViewFPV.setLayoutParams(params);
            }
            fpvViewFullscreen = false;
            msg_det.bringToFront();
            sub_info.bringToFront();
        }
    }

    private static void writeToFile(String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("config2.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private String readFromFile(Context context) {
        String ret = "";
        try {
            InputStream inputStream = context.openFileInput("config2.txt");

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

    private void notifyStatusChange() {


        final BaseProduct product = getBaseProduct();

        Log.d(TAG, "notifyStatusChange: " + (product == null ? "Disconnect" : (product.getModel() == null ? "null model" : product.getModel().name())));

        if (null == product || !product.isConnected()) {
            mCamera = null;
            showToast("Disconnected");
        } else {

            try {
                cameraDrone = new CameraDrone();
                cameraDrone.aspectratio();
                cameraDrone.setShootPhotoMode(SettingsDefinitions.ShootPhotoMode.SINGLE);
                cameraDrone.switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0);
                cameraDrone.setStorageLocation(SettingsDefinitions.StorageLocation.INTERNAL_STORAGE);
                cameraDrone.cameraCallBack();
            } catch (Exception e) {

            }

            if (product.getModel().equals(Model.MATRICE_210) || product.getModel().equals(Model.MATRICE_210_RTK) || product.getModel().equals(Model.MATRICE_300_RTK)) {
                List<Camera> cameras = product.getCameras();
                if (cameras != null) {
                    boolean cameraZero = false;
                    boolean cameraOne = false;
                    boolean cameraTwo = false;
                    for (int i = 0; i < cameras.size(); i++) {
                        if (cameras.get(i).getIndex() == 0) {
                            cameraZero = true;
                        } else if (cameras.get(i).getIndex() == 1) {
                            cameraOne = true;
                        } else if (cameras.get(i).getIndex() == 2) {
                            cameraTwo = true;
                        }
                    }
                    if (cameraZero) {
                        mCamera = product.getCameraWithComponentIndex(0);
                    } else if (cameraOne) {
                        mCamera = product.getCameraWithComponentIndex(1);
                    } else if (cameraTwo) {
                        mCamera = product.getCameraWithComponentIndex(2);
                    }
                    if (mCamera != null) {
                        mCamera.setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                    showToast("can't change mode of camera, error:" + djiError.getDescription());
                                }
                            }
                        });
                    }
                }
            } else if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                mCamera = product.getCamera();
                if (mCamera != null) {
                    mCamera.setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                showToast("can't change mode of camera, error:" + djiError.getDescription());
                                Log.d("ConnectDroneActivity", "error to set shoot photo mode");

                            } else {
                                Log.d("ConnectDroneActivity", "success set shoot photo mode");

                            }
                        }
                    });
                }
            }
            if (mCamera != null) {
                if (mCamera.isHybridZoomSupported()) {
//                    zoomLayout.setVisibility(View.VISIBLE);
                    zoomIn.setEnabled(true);
                    zoomOut.setEnabled(true);
                    mCamera.getHybridZoomSpec(new CommonCallbacks.CompletionCallbackWith<SettingsDefinitions.HybridZoomSpec>() {
                        @Override
                        public void onSuccess(SettingsDefinitions.HybridZoomSpec hybridZoomSpec) {
                            zoomFocalLenghtStep = hybridZoomSpec.getFocalLengthStep();
                            maxZoomFocalLenght = hybridZoomSpec.getMaxHybridFocalLength();
                            minZoomFocalLenght = hybridZoomSpec.getMinHybridFocalLength();
                        }

                        @Override
                        public void onFailure(DJIError djiError) {
                            handleDjiError(djiError);
                        }
                    });
                } else if (mCamera.getDisplayName().equals(Camera.DisplayNameZenmuseH20T)) {
                    mCamera.setCameraVideoStreamSource(CameraVideoStreamSource.WIDE, null);
                    List<Lens> lenses = mCamera.getLenses();
                    for (int i = 0; i < lenses.size(); i++)
                        if (lenses.get(i).getType() == SettingsDefinitions.LensType.ZOOM)
                            if (lenses.get(i).isHybridZoomSupported()) {
                                zoomIn.setEnabled(true);
                                zoomOut.setEnabled(true);
                                lenses.get(i).getHybridZoomSpec(new CommonCallbacks.CompletionCallbackWith<SettingsDefinitions.HybridZoomSpec>() {
                                    @Override
                                    public void onSuccess(SettingsDefinitions.HybridZoomSpec hybridZoomSpec) {
                                        zoomFocalLenghtStep = hybridZoomSpec.getFocalLengthStep();
                                        maxZoomFocalLenght = hybridZoomSpec.getMaxHybridFocalLength();
                                        minZoomFocalLenght = hybridZoomSpec.getMinHybridFocalLength();
                                    }

                                    @Override
                                    public void onFailure(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            } else if (lenses.get(i).isOpticalZoomSupported()) {
                                zoomIn.setEnabled(true);
                                zoomOut.setEnabled(true);
                                lenses.get(i).getOpticalZoomSpec(new CommonCallbacks.CompletionCallbackWith<SettingsDefinitions.OpticalZoomSpec>() {
                                    @Override
                                    public void onSuccess(SettingsDefinitions.OpticalZoomSpec opticalZoomSpec) {
                                        zoomFocalLenghtStep = opticalZoomSpec.getFocalLengthStep();
                                        maxZoomFocalLenght = opticalZoomSpec.getMaxFocalLength();
                                        minZoomFocalLenght = opticalZoomSpec.getMinFocalLength();
                                    }

                                    @Override
                                    public void onFailure(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            }
                } else {
                    zoomLayout.setVisibility(View.GONE);
                }

            }
        }


    }

    private void setRemainingFlightTimeChecker() {
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                int remainingFlightTime = getAircraft().getFlightController().getState().getGoHomeAssessment().getRemainingFlightTime();
                if (remainingFlightTime <= remainingFlightTimeLimit && !getAircraft().getFlightController().getSimulator().isSimulatorActive()) {
                    sendError("Aircraft Must Return Home NOW!!!");
                }
                handler.postDelayed(this, 30000);
            }
        }, 30000);
    }

    private void setBatteryCallback() {

        if (getBaseProduct().getModel() == Model.MATRICE_210 || getBaseProduct().getModel() == Model.MATRICE_210_RTK) {

            List<Battery> batteries = getAircraft().getBatteries();

            if (batteries != null) {
                for (int i = 0; i < batteries.size(); i++) {
                    final int index = i;
                    getAircraft().getBatteries().get(i).setStateCallback(new BatteryState.Callback() {
                        @SuppressLint("SetTextI18n")
                        @Override
                        public void onUpdate(BatteryState batteryState) {
                            mBattery = batteryState;

                        }
                    });
                }
            }


        } else {
            Battery battery = getAircraft().getBattery();
            if (battery != null) {
                battery.setStateCallback(new BatteryState.Callback() {
                    @SuppressLint("SetTextI18n")
                    @Override
                    public void onUpdate(BatteryState batteryState) {
                        mBattery = batteryState;

                    }
                });
            }

        }
    }

    public void getRosClient() {
        //get Ros client
        client = ((RCApplication) getApplication()).getRosClient();
        // setup the time service that will be used for the msgs
        timeService = new Service<>("/rosapi/get_time", Empty.class, GetTime.class, client);
        //get available nodes, services, topics (not used for any particular reason, needed by verifyUniqueDroneID())
    }


    //subscribe or not to the ros nodes
    public static void rosSubscribe(boolean subscribe) {
        if (!subscribe) {
            Log.i(TAG, "RosInit: Unsubscribing/Unadvertising Topics");
            for (int i = 0; i < topics.length; i++) {
                send("{\"op\":\"un" + how[i] + "\",\"topic\":\"" + getFullTopicName(topics[i]) + "\"}");
            }
            // send("{\"op\":\"unadvertise\",\"topic\":\"/droneIds\",\"type\":\"std_msgs/String\"}");
        } else {
            Log.i(TAG, "RosInit: Subscribing/Advertising Topics");
            for (int i = 0; i < topics.length; i++) {
                send("{\"op\":\"" + how[i] + "\",\"topic\":\"" + getFullTopicName(topics[i]) + "\",\"type\":\"" + types[i] + "\"}");
            }
            send("{\"op\":\"advertise\",\"topic\":\"/droneIds\",\"type\":\"std_msgs/String\"}");

        }
    }

    //loops until it finds a connection to the product
    public void loopForProductConnection() {
        final Runnable r = new Runnable() {
            public void run() {
                setBaseProduct(MainActivity.mProduct);
                if (mProduct != null) {
                    buttonEnable(true); //enable button for Ros to drone connection
                    handler.removeCallbacksAndMessages(this);
                    sendError("Product Connected:" + mProduct.toString());
                } else {
                    sendError("No Product Available");
                    buttonEnable(false);
                    handler.postDelayed(this, 1000);
                }
            }
        };
        handler.post(r);
    }

    //enable the button on the screen that starts ros transfer
    private void buttonEnable(final boolean enable) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                btn_service.setEnabled(enable);
                activity_main_screen_surface.setEnabled(enable);
            }
        });
    }

    public void enableServices() {
        telemetry = true;
        flightController = getAircraft().getFlightController();

        tryingConnectingToROS();
        setRosEnabled(true); //enable ros for threads interested
        setToggleImgSent(true); //enable for allowing surface update to start img sent thread
        loopForGimbalConnection(maxNumOfRetrialsConnect); //start thread to connect to gimbal push info
        loopForAssistantConnectionThread(maxNumOfRetrialsConnect); //start thread to connect to assistant push info
        rosPublishTelemetryThread(); //start publishing telemetry in different thread
        displayData((int) (1.0 / updateScreenTextRateInHz)); //start updating screen data every X milliseconds
        btn_service.setText("Stop ROS");

        if (getAircraft().getModel() == Model.MATRICE_210_RTK || getAircraft().getModel() == Model.MATRICE_300_RTK) {
            Objects.requireNonNull(getAircraft().getFlightController().getRTK()).setRtkEnabled(false, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    handleDjiError(djiError);
                    if (djiError != null)
                        sendError(djiError.toString());
                }
            });
        }

    }

    public void loopForAssistantConnectionThread(final int retrials) {
        Thread thread = new Thread() {
            @Override
            public void run() {
                int trials = retrials;
                while (trials >= 0) {
                    try {
                        if (startFrontObstacleDetection()) {
                            sendError("Assistant Available");
                            radarWidget.setVisibility(View.VISIBLE);
                            return;
                        } else {
                            sendError("NO Assistant, trial: " + trials);
                            radarWidget.setVisibility(View.INVISIBLE);
                            sleep(2000);
                        }
                        trials--;
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        thread.start();
    }

    public void loopForGimbalConnection(final int retrials) {
        Thread thread = new Thread() {
            @Override
            public void run() {
                int trials = retrials;
                while (trials >= 0) {
                    try {
                        if (startGimbalCallback()) {
                            sendError("Gimbal Available");
                            return;
                        } else {
                            sendError("NO Gimbal, trial: " + trials);
                            sleep(2000);
                        }
                        trials--;
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        thread.start();
    }

    public static String getFullTopicName(String name) {
        return "/" + droneName + "/" + name;
    }

    public void rosPublishTelemetryThread() {
        telemetry = true;
       // thread loops and sends telemetry continously at the fastest possible rate
        sendError("Starting Telemetry Thread");

        timerTelemetry = new Timer();
        long sleepTime = (long) (((1.0 / telemetryRateInHz)) * 1000);
        Log.d(TAG, "telemetry: " + telemetry);
        timerTelemetry.schedule(new TimerTask() {
            @Override
            public void run() {
                if (telemetry) {
                    if (getRosEnabled()) {
                        if (DEBUG) {
                            testPublishTelemetry();
                        } else {
                            runTelemetryPublisher();
                        }
                    }
                } else {
                    cancel();
                }
            }
        }, 1000, sleepTime);
    }

    boolean timerPublish = false;
    private int period = 1000; // ms

    public void rosPublishPicturesThread() {

        // thread loops and sends pictures continuously at the fastest possible rate
        sendError("Starting Pictures Thread");

        Log.d("ConnectDroneActivity", "rosPublishPicturesThread starting");
        timerPictures = new Timer();
        if (mCamera.isMultiLensCameraSupported())
            mCamera.getCameraVideoStreamSource(new CommonCallbacks.CompletionCallbackWith<CameraVideoStreamSource>() {
                @Override
                public void onSuccess(CameraVideoStreamSource cameraVideoStreamSource) {
                    if (cameraVideoStreamSource != CameraVideoStreamSource.WIDE)
                        mCamera.setCameraVideoStreamSource(CameraVideoStreamSource.WIDE, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                DJIError temp = djiError;
                                if (temp != null && temp.getDescription() != null)
                                    djiError.setDescription("getCameraVideoStreamSource: " + temp.getDescription());
                                handleDjiError(djiError);
                            }
                        });
                }

                @Override
                public void onFailure(DJIError djiError) {
                    DJIError temp = djiError;
                    if (temp != null && temp.getDescription() != null)
                        djiError.setDescription("getCameraVideoStreamSource: " + temp.getDescription());
                    handleDjiError(djiError);
                }
            });

        final SendPhotosToROS sendPhotosToROS = new SendPhotosToROS(this, MainActivity.masterIP + ":8000");

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                long sleepTime;

                try {
                    while (getRosEnabled() && timerPublish) {
                        double speed =Math.abs( flightController.getState().getVelocityX());
                        double height = flightController.getState().getAircraftLocation().getAltitude();
                        double distance = 2.0 * height * Math.tan(Math.toRadians(CameraDrone.getFov() / 2.0));
                        if (speed>2) {
                            sleepTime = (long) (distance / (speed*1.0) ) * 1000;
                            Log.d("ConnectDroneActivity", "rosPublishPicturesThread sleepTime: " + sleepTime);
                            Log.d("ConnectDroneActivity", "rosPublishPicturesThread distance: " + distance);
                            Log.d("ConnectDroneActivity", "rosPublishPicturesThread getFov: " + CameraDrone.getFov());
                            Log.d("ConnectDroneActivity", "rosPublishPicturesThread speed: " + speed);
                            Log.d("ConnectDroneActivity", "rosPublishPicturesThread height: " + height);

                            sleep(sleepTime);
                        }
                        else {
                            sleep(5000);

                        }
                        getPicture(sendPhotosToROS);


                    }
                    sendError("Pictures Thread Stopping");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
        thread.start();
    }

    private int counterSticks = 0;

    public void rosPublishObstacleSticksThread() {
        timerObstacle = new Timer();
        counterSticks = 0;
        timerObstacle.schedule(new TimerTask() {
            @Override
            public void run() {
                if (getRosEnabled() || telemetry) {
                    if (visionDetectionState != null) {
                        try {
                            Log.d(TAG, "rosPublishObstacleThread: running");
                            if (visionDetectionState.getPosition().toString().contains("NOSE")) {
                                FirebaseCrashlytics.getInstance().log(TAG + "rosPublishObstacleThread Nose");
                                Log.d(TAG, "rosPublishObstacleThread: Nose send data");

                                double dist1 = Math.min(visionDetectionState.getDetectionSectors()[0].getObstacleDistanceInMeters(), visionDetectionState.getDetectionSectors()[1].getObstacleDistanceInMeters());
                                double dist2 = Math.min(visionDetectionState.getDetectionSectors()[2].getObstacleDistanceInMeters(), visionDetectionState.getDetectionSectors()[3].getObstacleDistanceInMeters());
                                double dist3 = Math.min(dist1, dist2);
                                reportObstacle("" + dist3);
                            }

                        } catch (Exception e) {
                            Log.d(TAG, "rosPublishObstacleThread: Exception");

                            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                            e.printStackTrace();
                        }
                    }


                    if (flightControlData != null) {
                        if (enableVirtualControlStick) {
                            getAircraft().getFlightController().sendVirtualStickFlightControlData(flightControlData, new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null)
                                        handleDjiError(djiError);
                                }
                            });
                            counterSticks++;
                        }
                        if (counterSticks >= times) {
                            flightControlData = new FlightControlData(0.0f, 0.0f, flightControlData.getYaw(), 0.0f);
                            enableVirtualControlStick = false;
                            counterSticks = times = 0;
                        }
                    }

                } else {
                    cancel();
                }

            }
        }, 500, 300);
    }

    public void rosPublishObstacleThread() {
        timerObstacle = new Timer();

        timerObstacle.schedule(new TimerTask() {
            @Override
            public void run() {
                if (getRosEnabled() || telemetry) {
                    if (visionDetectionState != null) {
                        try {
                            Log.d(TAG, "rosPublishObstacleThread: running");
                            if (visionDetectionState.getPosition().toString().contains("NOSE")) {
                                FirebaseCrashlytics.getInstance().log(TAG + "rosPublishObstacleThread Nose");
                                double dist1 = Math.min(visionDetectionState.getDetectionSectors()[0].getObstacleDistanceInMeters(), visionDetectionState.getDetectionSectors()[1].getObstacleDistanceInMeters());
                                double dist2 = Math.min(visionDetectionState.getDetectionSectors()[2].getObstacleDistanceInMeters(), visionDetectionState.getDetectionSectors()[3].getObstacleDistanceInMeters());
                                double dist3 = Math.min(dist1, dist2);
                                reportObstacle("" + dist3);
                            }

                        } catch (Exception e) {
                            Log.d(TAG, "rosPublishObstacleThread: Exception");

                            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                            e.printStackTrace();
                        }
                    }
                } else {
                    cancel();
                }

            }
        }, 500, 300);
    }

    public int getIndexOfLargest(int[] array) {
        if (array == null || array.length == 0) return -1; // null or empty

        int largest = 0;
        for (int i = 1; i < array.length; i++) {
            if (array[i] > array[largest]) largest = i;
        }
        return largest; // position of the first largest found
    }

    public static void rosPublishDetectionThread(final String object, final String confidence) {
        detection = true;
        runDetectionPublisher(object, confidence);
    }

    private synchronized void runTelemetryPublisher() {
        if (getRosEnabled()) {
            Log.d(TAG, "runTelemetryPublisher");

            try {
                //telemetry display helpers
                int nsecs = 0;
                float altitude = 0;
                double lon = 0.0;
                double lat = 0.0;

                TelemetryMsg telemetryMsg = new TelemetryMsg();

                if (getAircraft() != null) {
                    TimePrimitive time;
                    Log.d(TAG, "runTelemetryPublisher 2");

                    try {
                        time = getRosTime();
                    } catch (Exception e) {
                        Log.d(TAG, "runTelemetryPublisher 4");

                        e.printStackTrace();
                        return;

                    }
                    Log.d(TAG, "runTelemetryPublisher 3");

                    telemetryMsg.rostime_secs = time.secs;
                    telemetryMsg.rostime_nsecs = time.nsecs;
                    nsecs = time.nsecs;

                    if (getAircraft().getFlightController() != null) {
                        telemetryMsg.flightTimeSecs = getAircraft().getFlightController().getState().getFlightTimeInSeconds();
                        telemetryMsg.batteryThreashold = getAircraft().getFlightController().getState().getBatteryThresholdBehavior().value();
                        if (mBattery != null) {

                            telemetryMsg.batteryPercentage = mBattery.getChargeRemainingInPercent();
                        } else {
                            setBatteryCallback();
                        }
                    }

                    if (flightController != null) {
                        FlightControllerState flightControllerState = getAircraft().getFlightController().getState();


                        altitude = flightControllerState.getAircraftLocation().getAltitude();

                        telemetryMsg.gpsSignal = flightControllerState.getGPSSignalLevel().value();
                        telemetryMsg.satelliteNumber = flightControllerState.getSatelliteCount();
                        telemetryMsg.altitude = altitude;
                        telemetryMsg.heading = getAircraft().getFlightController().getCompass().getHeading();
                        float xSpeed = flightControllerState.getVelocityX();
                        float ySpeed = flightControllerState.getVelocityY();
                        float zSpeed = flightControllerState.getVelocityZ();
                        double xySpeed = sqrt((xSpeed * xSpeed) + (ySpeed * ySpeed));
                        telemetryMsg.velocity = sqrt((xySpeed * xySpeed) + (zSpeed * zSpeed));
                        lon = flightControllerState.getAircraftLocation().getLongitude();
                        lat = flightControllerState.getAircraftLocation().getLatitude();
                        if (!Double.isNaN(lon) && !Double.isNaN(lat)) {
                            telemetryMsg.longitude = lon;
                            telemetryMsg.latitude = lat;
                        }
                        double hlat = flightControllerState.getHomeLocation().getLatitude();
                        double hlon = flightControllerState.getHomeLocation().getLongitude();
                        if (!Double.isNaN(hlon) && !Double.isNaN(hlat)) {
                            telemetryMsg.homeLatitude = hlat;
                            telemetryMsg.homeLongitude = hlon;
                        }
                        WaypointMissionOperator waypointMissionOperator = Mission.getWaypointMissionOperator();
                        if (waypointMissionOperator != null && waypointMissionOperator.getCurrentState() == WaypointMissionState.EXECUTING) {
                            telemetryMsg.droneState = "In_Mission";
                        } else if (waypointMissionOperator != null && waypointMissionOperator.getCurrentState() == WaypointMissionState.EXECUTION_PAUSED) {
                            telemetryMsg.droneState = "Paused_Mission";
                        } else if (flightControllerState.isFlying()) {
                            telemetryMsg.droneState = "Flying";
                        } else if (flightControllerState.isGoingHome()) {
                            telemetryMsg.droneState = "Going_Home";
                        } else if (flightControllerState.areMotorsOn()) {
                            telemetryMsg.droneState = "Armed";
                        } else {
                            telemetryMsg.droneState = "Landed";
                        }
                        if(gimbalState == null){
                            telemetryMsg.gimbalAngle = 999;
                        }else{
                            telemetryMsg.gimbalAngle = gimbalState.getAttitudeInDegrees().getPitch();
                        }
                    }


                    //send telemetry data
                    String telemetry = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[1]) + "\",\"msg\":" + telemetryMsg.toJSON() + "}";
                    Log.d(TAG, "send telemtery...");

                    send(telemetry);

                }
                DecimalFormat df2 = new DecimalFormat(".##########");

                if (getAircraft() != null && getAircraft().getFlightController() != null) {
                    if(gimbalState == null){
                        String displayData = "RosTime:\n\t" + nsecs + "\n\nLongitude:\n\t" + df2.format(lon) + "\n\nLatitude:\n\t" + df2.format(lat) + "\n\nHeading:\n\t" + getAircraft().getFlightController().getCompass().getHeading() + "\n\nState:\n\t" + telemetryMsg.droneState + "\n\nGimbal Angle:\n\t" + 999;
                        setDisplayData(displayData); //set display data to be updated on their next loop

                    }else{
                        String displayData = "RosTime:\n\t" + nsecs + "\n\nLongitude:\n\t" + df2.format(lon) + "\n\nLatitude:\n\t" + df2.format(lat) + "\n\nHeading:\n\t" + getAircraft().getFlightController().getCompass().getHeading() + "\n\nState:\n\t" + telemetryMsg.droneState + "\n\nGimbal Angle:\n\t" + gimbalState.getAttitudeInDegrees().getPitch();
                        setDisplayData(displayData); //set display data to be updated on their next loop
                    }
                }
                System.gc();
            } catch (Exception e) {
                e.printStackTrace();
                sendError("TELEMETRY EXCEPTION ERROR");
            }
        }
    }
    public static Boolean getTelemetry() {
        return telemetry;
    }

    //construct Detection json msg
    private static void runDetectionPublisher(final String object, final String confidence) {
        if (getRosEnabled()) {
            try {
                String detectData = jStart();
                TimePrimitive time = getRosTime();
                detectData += jMsg("rostime_secs", time.secs) + ",";
                detectData += jMsg("rostime_nsecs", time.nsecs) + ",";
                DecimalFormat df2 = new DecimalFormat(".##########");
                if (getAircraft().getFlightController() != null) {
                    detectData += jMsg("alt", flightController.getState().getAircraftLocation().getAltitude()) + ",";
                    detectData += jMsg("long", df2.format(flightController.getState().getAircraftLocation().getLongitude())) + ",";
                    detectData += jMsg("lat", df2.format(flightController.getState().getAircraftLocation().getLatitude())) + ",";
                }
                detectData += jMsg("Object ", object) + jMsg("Confidence", confidence) + jEnd();
                String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[0]) + "\",\"msg\":{" + detectData + "}}";
                send(msg);

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static String getFileTimestamp() {
        SimpleDateFormat sdf = new SimpleDateFormat("HHmmss");
        String currentDateandTime = sdf.format(new Date());


        DecimalFormat df2 = new DecimalFormat(".##########");
        String timestamp = currentDateandTime;

        if (getAircraft().getFlightController() != null) {
            getAircraft().getFlightController().getState();
            timestamp += "_lat_" + df2.format(getAircraft().getFlightController().getState().getAircraftLocation().getLatitude());
            timestamp += "_long_" + df2.format(getAircraft().getFlightController().getState().getAircraftLocation().getLongitude());
            timestamp += "_alt_" + getAircraft().getFlightController().getState().getAircraftLocation().getAltitude();
        }
        return timestamp;
    }

    public synchronized void msgToRightScreenText(final String msg) {
        activityConnect.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Log.d(TAG, "Update text Right: " + msg);
                sub_info.setText(msg);
            }
        });

    }

    public synchronized void msgToLeftScreenText(final String msg) {
        activityConnect.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //Log.d(TAG, "Update Text Left: " + msg);
                msg_det.setText(msg);
            }
        });
    }

    public void updateTexts() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                textQualityRate.setText(getImgQuality() + "% | " + String.format("%.3f", getRateQuality()) + "s, (Fix: " + String.format("%.1f", 1.0 / imgRateInHz) + "Hz)");
//                textTelemetryRate.setText(String.format("%.3f", getRateTelemetry()) + "s, (Fix: " + String.format("%.1f", telemetryRateInHz) +"Hz)");
            }
        });
    }

    public void handleDjiError(DJIError djiError) {
        if (djiError == null) {
            djiError = DJIError.COMMON_UNKNOWN;
            djiError.setDescription("Successful (But returned null)");
            sendError("DjiError: " + djiError.getDescription());
            Log.d(TAG, "DjiError: " + djiError.getDescription());
        } else {
            Log.d(TAG, "DjiError: " + djiError.getDescription());
        }
    }


    private WaypointMission createRandomWaypointMission(int numberOfWaypoint, int numberOfAction) {
        WaypointMission.Builder builder = new WaypointMission.Builder();

        final float baseAltitude = 30.0f;
        builder.autoFlightSpeed(15f);
        builder.maxFlightSpeed(15f);
        builder.setExitMissionOnRCSignalLostEnabled(false);
        builder.finishedAction(WaypointMissionFinishedAction.GO_FIRST_WAYPOINT);
        builder.flightPathMode(WaypointMissionFlightPathMode.NORMAL);
        builder.gotoFirstWaypointMode(WaypointMissionGotoWaypointMode.SAFELY);
        builder.headingMode(WaypointMissionHeadingMode.AUTO);
        builder.repeatTimes(1);
        List<Waypoint> waypointList = new ArrayList<>();

        Waypoint eachWaypoint = new Waypoint(35.144159, 33.412992, baseAltitude);
        waypointList.add(eachWaypoint);
        eachWaypoint = new Waypoint(35.143789, 33.413099, baseAltitude);
        waypointList.add(eachWaypoint);
        eachWaypoint = new Waypoint(35.143363, 33.412861, baseAltitude);
        waypointList.add(eachWaypoint);
        eachWaypoint = new Waypoint(35.143756, 33.412630, baseAltitude);
        waypointList.add(eachWaypoint);
        eachWaypoint = new Waypoint(35.144159, 33.412992, baseAltitude);
        waypointList.add(eachWaypoint);

        builder.waypointList(waypointList).waypointCount(waypointList.size());
        return builder.build();
    }

    private void hotpointMissionDji(String hpMissionDji) {
        try {
            hotpointMissionOperator = MissionControl.getInstance().getHotpointMissionOperator();
            if (getRosEnabled()) {
                if (mProduct != null) {
                    JSONParser parser = new JSONParser();
                    JSONObject jsonObj = (JSONObject) parser.parse(hpMissionDji);
                    JSONObject gpsInput = (JSONObject) jsonObj.get("hpGpsInput");
                    String missionName = (String) jsonObj.get("name");
                    JSONObject missionCommand = (JSONObject) jsonObj.get("missionCommand");
                    int command = (int) ((long) missionCommand.get("missionCommand"));
                    switch (command) {
                        case 0:
                            Log.d(TAG, missionName);
                            double latitude = (double) gpsInput.get("latitude");
                            double longitude = (double) gpsInput.get("longitude");
                            double speed = (double) gpsInput.get("speed");
                            double altitude = (double) gpsInput.get("altitude");
                            double radius = (double) gpsInput.get("radius");
                            boolean clockwise = (boolean) gpsInput.get("clockwise");

                            hpMission = new HotpointMission(new LocationCoordinate2D(latitude, longitude), altitude, radius, (float) speed, clockwise, HotpointStartPoint.NEAREST, HotpointHeading.TOWARDS_HOT_POINT);

                            if (hotpointMissionOperator != null && hpListener != null) {
                                hotpointMissionOperator.addListener(hpListener);
                            }

                            try {
                                showToast("Start HotpointMissionDJI");
                                hotpointMissionOperator.startMission(hpMission, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        handleDjiError(djiError);
                                        if (djiError == null) {}
                                    }
                                });
                            } catch (NullPointerException e) {
                                Log.e("HOTPOINT", e.toString());
                            }

                            break;
                        case 1:
                            hotpointMissionOperator.stop(new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    handleDjiError(djiError);
                                }
                            });
                            return;
                        case 2:
                            hotpointMissionOperator.pause(new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    handleDjiError(djiError);
                                    if (djiError == null) {}
                                }
                            });
                            return;
                        case 3:
                            hotpointMissionOperator.resume(new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    handleDjiError(djiError);
                                    if (djiError == null) {
                                    }
                                }
                            });
                    }

                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private JSONParser parser = new JSONParser();
    private JSONObject jsonObj;

    static boolean enableVirtualControlStick = false;

    private void controlDrone(String input) {
        try {
            if (getRosEnabled()) {
                if (mProduct != null) {
                    Aircraft p = (Aircraft) mProduct;
                    try {
                        jsonObj = (JSONObject) parser.parse(input);
                    } catch (Exception e) {
                        FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                        Log.e(TAG, e.toString());
                    }

                    boolean isFlightControl = (boolean) jsonObj.get("isFlightControlData");
                    enableVirtualControlStick = (boolean) jsonObj.get("enableVirtualControlStick");
                    boolean isBasicCommand = (boolean) jsonObj.get("isBasicCommand");
                    boolean isGimbalData = (boolean) jsonObj.get("isGimbalData");

                    if (isFlightControl) {
                        getAircraft().getFlightController().setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                handleDjiError(djiError);
                            }
                        });
                        JSONObject flightControl = (JSONObject) jsonObj.get("flightControlData");
                        Log.d(TAG, flightControl.toJSONString());
                        float pitch = (float) ((double) flightControl.get("pitch"));
                        float roll = (float) ((double) flightControl.get("roll"));
                        float yaw = (float) ((double) flightControl.get("yaw"));
                        float throttle = (float) ((double) flightControl.get("verticalThrottle"));
                        int iterations = (int) ((long) flightControl.get("verticalControlMode"));
                        djiManualControl(p, pitch, roll, yaw, throttle, iterations);
                        Log.d(TAG, "Manual Control");
                    }
                    if (isBasicCommand) {
                        JSONObject basicCommand = (JSONObject) jsonObj.get("basicCommand");
                        Log.d(TAG, basicCommand.toJSONString());
                        int command = (int) ((long) basicCommand.get("command"));
                        switch (command) {
                            case 0:
                                djiTakeOff();
                                Log.d(TAG, "TakeOff");
                                showToast("Takeoff");
                                break;
                            case 1:
                                djiLand();
                                Log.d(TAG, "Land");
                                showToast("Land");
                                break;
                            case 2:
                                djiMotorOn(p);
                                Log.d(TAG, "Motors ON");
                                break;
                            case 3:
                                djiMotorOff(p);
                                Log.d(TAG, "Motors Off");
                                break;
                            case 4:
                                djiGoHome();
                                Log.d(TAG, "GoHome");
                            default:
                                break;
                        }

                    }
                    if (isGimbalData) {
                        JSONObject gimbalControlData = (JSONObject) jsonObj.get("gimbalControlData");
                        Log.d(TAG, gimbalControlData.toJSONString());
                        float gimbalRoll = (float) ((double) gimbalControlData.get("roll"));
                        float gimbalPitch = (float) ((double) gimbalControlData.get("pitch"));
                        float gimbalYaw = (float) ((double) gimbalControlData.get("roll"));
                        float gimbalTime = (float) ((double) gimbalControlData.get("time"));
                        djiGimbalControl(p, gimbalRoll, gimbalPitch, gimbalYaw, gimbalTime);
                        Log.d(TAG, "Gimbal Control");
                    }

                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private void releaseDji(String input) {
        try {
            if (getRosEnabled()) {
                if (mProduct != null) {
                    jsonObj = (JSONObject) parser.parse(input);

                    int releaseCommand = (int) ((long) jsonObj.get("command"));
                    switch (releaseCommand) {
                        case 0:
                            Log.d(TAG, "Release:Collect");
                            showToast("Release Mechanism: Collect");
                            break;
                        case 1:
                            Log.d(TAG, "Release:Hold");
                            showToast("Release Mechanism: Hold");
                            break;
                        case 2:
                            Log.d(TAG, "Release:Release");
                            showToast("Release Mechanism: Release");
                            break;
                        default:
                            break;
                    }

                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void djiGimbalControl(Aircraft p, float roll, float pitch, float yaw, float time) {
        Rotation.Builder rb = new Rotation.Builder();
        rb.mode(RotationMode.ABSOLUTE_ANGLE);
        if (roll != -1)
            rb.roll(roll);
        if (pitch != -1)
            rb.pitch(pitch);
        if (yaw != -1)
            rb.yaw(yaw);
        if (time != -1) {
            rb.time(time);
        }
        Rotation rotation = rb.build();

        if (p.getGimbal().isConnected()) {

            p.getGimbal().rotate(rotation, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    DJIError temp = djiError;
                    if (temp != null && temp.getDescription() != null)
                        djiError.setDescription("djiGimbalControl: " + temp.getDescription());
                    handleDjiError(djiError);
                }
            });
        }
    }

    private void djiMotorOn(Aircraft p) {
        if (!p.getFlightController().getState().areMotorsOn()) {
            p.getFlightController().turnOnMotors(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    handleDjiError(djiError);
                }
            });
        }
    }

    private void djiMotorOff(Aircraft p) {
        if (p.getFlightController().getState().areMotorsOn()) {
            p.getFlightController().turnOffMotors(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    handleDjiError(djiError);
                }
            });
        }
    }

    private void djiLand() {
        flightController.startLanding(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                handleDjiError(djiError);
            }
        });
    }

    private void djiTakeOff() {

        flightController.startTakeoff(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                handleDjiError(djiError);
            }
        });
    }

    private void djiGoHome() {

        flightController.startGoHome(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                handleDjiError(djiError);
            }
        });
    }

    private void controlZoomDJI(String input) {
        try {
            if (getRosEnabled()) {
                jsonObj = (JSONObject) parser.parse(input);
                mCamera.getHybridZoomFocalLength(new CommonCallbacks.CompletionCallbackWith<Integer>() {
                    @Override
                    public void onSuccess(Integer integer) {
                        currentZoomFocalLenght = integer;
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        handleDjiError(djiError);
                    }
                });

                int zoomCommand = (int) ((long) jsonObj.get("command"));
                final int zoomSteps = (int) ((long) jsonObj.get("zoomSteps"));

                switch (zoomCommand) {
                    case 0:
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                int newZoomLength = currentZoomFocalLenght + (zoomFocalLenghtStep * zoomSteps);
                                if (newZoomLength >= maxZoomFocalLenght) {
                                    newZoomLength = maxZoomFocalLenght;
                                    showToast("Max Zoom");
                                }
                                mCamera.setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            }
                        }, 5);
                        break;
                    case 1:
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                int newZoomLength = currentZoomFocalLenght - (zoomFocalLenghtStep * zoomSteps);
                                if (newZoomLength <= minZoomFocalLenght) {
                                    newZoomLength = minZoomFocalLenght;
                                    showToast("Min Zoom");
                                }
                                mCamera.setHybridZoomFocalLength(newZoomLength, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            }
                        }, 5);
                        break;
                    case 2:
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                mCamera.setHybridZoomFocalLength(maxZoomFocalLenght, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            }
                        }, 5);
                        break;
                    case 3:
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                mCamera.setHybridZoomFocalLength(minZoomFocalLenght, new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        handleDjiError(djiError);
                                    }
                                });
                            }
                        }, 5);
                        break;
                    default:
                        break;
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    static int times = 0;
    static FlightControlData flightControlData;

    private void djiManualControl(final Aircraft p, float pitch, float roll, float yaw, float throttle, int iterations) {
        flightControlData = new FlightControlData(roll, pitch, yaw, throttle);
        times = iterations;
    }

    private void setupPicServer(String setupMsg) {
        try {
            Log.d(TAG, "setup Pic Server receivced: " + setupMsg);
            JSONParser parser = new JSONParser();
            JSONObject jsonObj = (JSONObject) parser.parse(setupMsg);
            picServerIP = (String) jsonObj.get("serverIP");
            picServerPort = (int) ((long) jsonObj.get("serverPort"));


        } catch (ParseException e) {
            Log.e(TAG, e.toString());
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
        }
    }

    static Bitmap b;

    private BuildMap getPicture(final SendPhotosToROS sendPhotosToROS) {
        File file;


        if (gimbalState.getAttitudeInDegrees().getPitch() >= -89.5f) {
            djiGimbalControl(getAircraft(), 0.0f, -90.0f, 0.0f, 0);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if (keepOriginalPhoto == true) {
            Thread takePhotoThread = new Thread(new Runnable() {
                @Override
                public void run() {

                    getAircraft().getCamera().startShootPhoto(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null)
                                Log.d("ConnectDroneActivity", "getCamera shoot photo error : " + djiError.toString());
                            else
                                Log.d("ConnectDroneActivity", "getCamera shoot photo success ");

                            handleDjiError(djiError);
                        }
                    });
                }
            });
            takePhotoThread.start();
        }
        BuildMap buildMap = new BuildMap();

        if (getAircraft().getFlightController() != null) {
            Log.d("ConnectDroneActivity", "getPicture: save picture");

            b = videoFeedViewPrimary.getBitmap().copy(Bitmap.Config.RGB_565, true);
            double lat = flightController.getState().getAircraftLocation().getLatitude();
            double lon = flightController.getState().getAircraftLocation().getLongitude();
            float alt = flightController.getState().getAircraftLocation().getAltitude();
            float heading = getAircraft().getFlightController().getCompass().getHeading();
            buildMap.buildmap = false;
            buildMap.photoid = getRosTime().secs;
            buildMap.altitude = alt;
            buildMap.heading = heading;
            if (!Double.isNaN(lon)) {
                buildMap.latitude = lat;
                buildMap.longitude = lon;
            }
            String msg1 = buildMap.toJSON();
            SimpleDateFormat sdf = new SimpleDateFormat("HHmmss");
            String currentDateandTime = sdf.format(new Date());

            String filename = "pictureMap_" + currentDateandTime + ".jpeg";
            FirebaseCrashlytics.getInstance().log("BUILDMAP " + msg1);
            Log.d("BUILDMAP", msg1);
            file = new File(dir2.getPath(), filename);
            try (FileOutputStream out = new FileOutputStream(file)) {
                FirebaseCrashlytics.getInstance().log("BUILDMAP create jpeg file");
                b.compress(Bitmap.CompressFormat.JPEG, 100, out);


            } catch (IOException e) {
                FirebaseCrashlytics.getInstance().recordException(new Throwable(e.getMessage()));
                e.printStackTrace();
                showToast(e.toString());
            }

            try {
                Log.d("ConnectDroneActivity", "sendPicture dir2: " + (dir2 != null ? dir2.listFiles().length : "empty file"));

                Gson gson = new Gson();
                String msg = gson.toJson(buildMap);
                try {
                    setDisplayDataLeft(msg);
                    sendPhotosToROS.sendPhotoBuildMap(file.getPath(), buildMap);
                } catch (Exception e) {
                    e.printStackTrace();

                }
                setDisplayDataLeft(msg);
                Log.d("ConnectDroneActivity", "sendPicture: " + msg);

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return buildMap;
    }

    private boolean isCompletelyWritten(File file) {
        RandomAccessFile stream = null;
        try {
            stream = new RandomAccessFile(file, "rw");
            return true;
        } catch (Exception e) {
            Log.i("ConnectDroneActivity", "Skipping file " + file.getName() + " for this iteration due it's not completely written");
        } finally {
            if (stream != null) {
                try {
                    stream.close();
                } catch (IOException e) {
                    Log.e("ConnectDroneActivity", "Exception during closing file " + file.getName());
                }
            }
        }
        return false;
    }

    static DataOutputStream dos;

    private void clearPreviousBuildMapFiles() {
        try {
            for (final File fileEntry : dir2.listFiles()) {
                String msgPath = dir3.getPath() + "/" + fileEntry.getName().split("\\.")[0] + ".txt";
                String msg = readFromTxtFile(msgPath);
                setDisplayDataLeft(msg);
                File txt = new File(msgPath);
                FirebaseCrashlytics.getInstance().log("clearPreviousBuildMapFiles " + msgPath);
                fileEntry.delete();
                txt.delete();
            }
        } catch (Exception e) {
            FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
            e.printStackTrace();
        }
    }

    private String readFromTxtFile(String filename) {
        String line = "";
        String msgLine = "";
        try {
            FileReader inputStream = new FileReader(filename);
            BufferedReader bufferedReader = new BufferedReader(inputStream);

            while ((line = bufferedReader.readLine()) != null) {
                msgLine = line;
            }
            inputStream.close();
        } catch (FileNotFoundException e) {
            Log.e("ConnectDroneActivity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("ConnectDroneActivity", "Can not read file: " + e.toString());
            showToast(e.toString());
        }

        return msgLine;
    }


    //shw msgs on the screen!
    public void showToast(final String toastMsg) {
        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), toastMsg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    //displayData Looper for screen update
    public void displayData(final int every) {
        try {
            if (timerDisplaydata != null)
                return;
            timerDisplaydata = new Timer();
            Log.d(TAG, "start display data: " + every);
            timerDisplaydata.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (dataRight) {
                        msgToRightScreenText(getDisplayData());
                        dataRight = false;
                    }
                    if (dataLeft) {
                        dataLeft = false;
                        msgToLeftScreenText(getDisplayDataLeft());
                    }
                }
            }, 500, every * 1000);
        } catch (Exception e) {

        }
    }


    // ON EVENT LISTENERS
    //Receive data from ROS server, send from ROSBridgeWebSocketClient onMessage()
    @Subscribe
    public void onEvent(final PublishEvent event) {
        try {
            Log.d(TAG, "On Event: " + event.name + " , " + event.id + " , " + event.msg);

            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    if (getRosEnabled()) {
                        setDisplayDataLeft(event.name + " : " + event.msg);
                        if (getFullTopicName(topics[3]).equals(event.name)) {
                            controlDrone(event.msg);
                            Log.d(TAG, "Control Drone: " + event.msg);
                        } else if (getFullTopicName(topics[2]).equals(event.name)) {
                            Log.d(TAG, "Mission.inProgress: " + Mission.inProgress);

                            Mission mission = new Mission(getActivity(), msg_det, m, false, 4);
                            mission.missionDji(event.msg);
                            Log.d(TAG, "Waypoint Mission DJI: " + event.msg);
                        } else if (getFullTopicName(topics[5]).equals(event.name)) {
                            releaseDji(event.msg);
                            Log.d(TAG, "Release DJI: " + event.msg);
                        } else if (getFullTopicName(topics[6]).equals(event.name)) {
                            try {
                                jsonObj = (JSONObject) parser.parse(event.msg);
                                int command = (int) ((long) jsonObj.get("command"));
                                if (command == 0)
                                    handleStreamRequest(true);
                                else if (command == 1)
                                    handleStreamRequest(false);
                            } catch (ParseException | NullPointerException e) {
                                e.printStackTrace();
                                FirebaseCrashlytics.getInstance().log("Handle ROS stream request: " + e.getClass().toString());
                                FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                            }
                            Log.e(TAG, "Stream: " + event.name);
                        } else if (getFullTopicName(topics[7]).equals(event.name)) {
                            controlZoomDJI(event.msg);
                            Log.e(TAG, "Zoom: " + event.name);
                        } else if (getFullTopicName(topics[8]).equals(event.name)) {
                            hotpointMissionDji(event.msg);
                            Log.d(TAG, "Hotpoint Mission DJI: " + event.msg);
                        } else if (getFullTopicName(topics[10]).equals(event.name)) {
                            try {
                                JSONParser parser = new JSONParser();
                                JSONObject jsonObj = (JSONObject) parser.parse(event.msg);
                                boolean active = (boolean) (jsonObj.get("buildmap"));
                                Log.d(TAG, "buildmap: active : " + active);
                                Log.d(TAG, "buildmap: overlap 2 : " + jsonObj.get("overlap"));

                                int overlap = Integer.parseInt (jsonObj.get("overlap").toString());
                                Log.d(TAG, "buildmap: overlap : " + active);

                                CameraDrone.setFovOverlap(overlap);

                                if (active) {
                                    if (!timerPublish) {
                                        timerPublish = true;
                                        rosPublishPicturesThread();
                                    }



                                } else {
                                    timerPublish = false;
                                }


                            } catch (Exception e) {
                                Log.d(TAG, "buildmap: exception");

                                e.printStackTrace();
                            }
                            Log.d(TAG, "Build Map: " + event.msg);
                        } else if (getFullTopicName(topics[12]).equals(event.name)) {
                            setupPicServer(event.msg);
                            Log.d(TAG, "buildmap: Setup Pic Server: " + event.msg);
                        } else if (getFullTopicName("handshake").equals(event.name)) {
                            Log.d(TAG, "On Event handshake: " + event.msg);

                            try {
                                JSONParser parser = new JSONParser();
                                JSONObject jsonObj = (JSONObject) parser.parse(event.msg);
                                if (timerConnectDroneROS != null)
                                    timerConnectDroneROS.cancel();
                                int connected = Integer.parseInt(jsonObj.get("data").toString());
                                handshakeconnected = (connected == 1);
                                Log.d(TAG, "handshake result: " + handshakeconnected);


                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                            Log.d(TAG, "Setup Pic Server: " + event.msg);
                        }
                        //Subscribe to StartOrStopPointCloud
                        else if (getFullTopicName(topics[15]).equals(event.name)) {
                            try {
                                JSONParser parser = new JSONParser();
                                JSONObject jsonObject = (JSONObject) parser.parse(event.msg);
                                startOrStopPointCloud = (String) jsonObject.get("data");
                                Log.d(TAG, "PointCloud: " + startOrStopPointCloud);


                                if (startOrStopPointCloud.equals("START")){
                                    currentLidar.addPointCloudLiveViewDataListener(LidarLiveListener);

                                    currentLidar.pointCloudRecord(DJILidarPointCloudRecord.START_POINT_CLOUD_RECORDING, new CommonCallbacks.CompletionCallback(){
                                        @Override
                                        public void onResult(DJIError djiError)
                                        {
                                            if (djiError == null) {
                                                recordingStatus = RecordingStatus.STARTED;
                                                showToast("Record Lidar: Success");
                                            }else {
                                                showToast(djiError.getDescription());
                                            }
                                            Log.d("Recording", String.valueOf(recordingStatus));
                                        }

                                    });

                                    currentLidar.startReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback(){
                                        @Override
                                        public void onResult(DJIError djiError)
                                        {
                                            if (djiError == null) {
                                                showToast("Record Lidar: Success");

                                            }else {
                                                Log.d("Error", djiError.getDescription());
                                                showToast(djiError.getDescription());
                                            }
                                        }
                                    });
                                    startLidar = false;
                                    //item1.setTitle("Stop Lidar");
                                }else{
                                    currentLidar.removePointCloudLiveViewDataListener(LidarLiveListener);

                                    currentLidar.pointCloudRecord(DJILidarPointCloudRecord.STOP_POINT_CLOUD_RECORDING, new CommonCallbacks.CompletionCallback(){
                                        @Override
                                        public void onResult(DJIError djiError)
                                        {
                                            if (djiError == null) {
                                                recordingStatus = RecordingStatus.STOPPED;
                                                showToast("Stop Record Lidar: Success");
                                                Log.d("Stop Record Lidar", String.valueOf(recordingStatus));

                                            }else {
                                                showToast(djiError.getDescription());
                                            }
                                            Log.d("Recording", String.valueOf(recordingStatus));
                                        }
                                    });

                                    currentLidar.stopReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback(){
                                        @Override
                                        public void onResult(DJIError djiError)
                                        {
                                            if (djiError == null) {
                                                showToast("Stop Record Lidar: Success");

                                            }else {
                                                showToast(djiError.getDescription());
                                            }
                                        }
                                    });
                                    startLidar = true;
                                    //item1.setTitle("Start Lidar");
                                }

                            } catch (ParseException e) {
                                e.printStackTrace();
                            }
                        }
                        else if (getFullTopicName(topics[17]).equals(event.name)) {
                            try {
                                JSONParser parser = new JSONParser();
                                JSONObject jsonObject = (JSONObject) parser.parse(event.msg);
                                startOrStopRangeFinder = (String) jsonObject.get("data");
                                Log.d(TAG, "StartOrStopRangeFinder: " + startOrStopRangeFinder);

                                BaseProduct product = FPVDemoApplication.getProductInstance();
                                Camera camera = product.getCamera();

                                if (startOrStopRangeFinder.equals("START")){

                                        camera.setLaserEnabled(true, new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {
                                                if(djiError == null)
                                                {
                                                    Lens lens = camera.getLens(0);      // 0-> Zoom , 1-> WIDE , 2->THERMAL

                                                    lens.setLaserMeasureInformationCallback(new LaserMeasureInformation.Callback() {
                                                        @Override
                                                        public void onUpdate(LaserMeasureInformation laserMeasureInformation) {

                                                            DecimalFormat coord = new DecimalFormat("###.######");
                                                            DecimalFormat alt = new DecimalFormat("####.#");
                                                            DecimalFormat dist = new DecimalFormat("#####.###");

                                                            try {

                                                                PointF pointF = laserMeasureInformation.getTargetPoint();

                                                                String targetDistance = dist.format(laserMeasureInformation.getTargetDistance());
                                                                sleep(1000);
                                                                Log.d("Before Send: ", targetDistance);
                                                                //publish
                                                                sendRangeFinder(targetDistance);

                                                            } catch (Exception e) {
                                                                e.printStackTrace();
                                                            }
                                                        }
                                                    });

                                                    showToast("Start Range Finder: Success");
                                                }
                                                else
                                                {
                                                    showToast("Error Getting Lenses: " + djiError.getDescription());
                                                    Log.d("ERROR GETTING LENSES", djiError.getDescription());
                                                }
                                            }
                                        });

                                        startRangeFinder = false;
                                        item1.setTitle("Stop Range Finder");

                                    }else{
                                        camera.setLaserEnabled(false, new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {
                                                if(djiError == null)
                                                {
                                                    showToast("Stop Range Finder: Success");
                                                }
                                                else
                                                {
                                                    showToast("Error Getting Lenses: " + djiError.getDescription());
                                                    Log.d("ERROR GETTING LENSES", djiError.getDescription());
                                                }
                                            }
                                        });
                                        startRangeFinder = true;
                                        item1.setTitle("Start Range Finder");
                                    }

                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }
            });
            thread.start();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //Receive data from Event Bus
    @Subscribe
    public void onEvent(MessageEvent event) {
        Log.d(TAG, "On Event: " + event.getMessage());
        if (event.getMessage().equals("dji_product")) {
            mProduct = ((RCApplication) getApplication()).getBaseProduct();
            sendError("Connection to dji has changed");
            if (mProduct != null) {
                buttonEnable(true);
            } else {
                buttonEnable(false);
            }
        }
    }

    // region GETTER AND SETTERS SYNCHRONISED
    public static synchronized void setRosEnabled(Boolean data) {
        rosEnabled = data;
    }

    public static synchronized Boolean getRosEnabled() {
        return rosEnabled;
    }

    //
    public synchronized void setDisplayData(String data) {
        dataToDisplay = data;

        LiveStreamManager liveStreamManager = DJISDKManager.getInstance().getLiveStreamManager();
        if (rtmpStream && liveStreamManager.isStreaming()) {
            long streamTime = liveStreamManager.getStreamTime();
            dataToDisplay += "\n\nStream Time:\n\t" + streamTime;
            float streamFPS = liveStreamManager.getLiveVideoFps();
            dataToDisplay += "\n\nStream FPS:\n\t" + streamFPS;
            int bitrate = liveStreamManager.getLiveVideoBitRate();
            dataToDisplay += "\n\nStreaming BitRate:\n\t" + bitrate;
            boolean autioMute = liveStreamManager.isAudioMuted();
            dataToDisplay += "\n\nAudio Muted:\n\t" + autioMute;
            if (liveTxt.getVisibility() != View.VISIBLE) {
                activityConnect.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        liveTxt.setVisibility(View.VISIBLE);

                    }
                });
            }
        }
        dataRight = true;

    }

    public synchronized String getDisplayData() {
        return dataToDisplay;
    }

    //
    public synchronized void setDisplayDataLeft(String data) {
        dataLeft = true;

        TimePrimitive time = getRosTime();

        if (data.contains("Input")) {
            dataToDisplayLeft = "Input Command Received:" + "rostime_secs" + time.secs;
        } else {
            dataToDisplayLeft = data;
        }

    }

    public synchronized void setDisplayDataLeft2(String data) {
        dataLeft = true;

        // TimePrimitive time = getRosTime();
        if (data.contains("Input")) {
            dataToDisplayLeft = "Input Command Received:" + "rostime_secs" /*+ time.secs*/;
        } else {
            dataToDisplayLeft = data;
        }
    }

    public synchronized String getDisplayDataLeft() {
        return dataToDisplayLeft;
    }

    public synchronized void setVisionDetection(VisionDetectionState odb) {
        visionDetectionState = odb;
    }

    public synchronized void setGimbalState(GimbalState gmb) {
        gimbalState = gmb;
    }

    public static synchronized TimePrimitive getRosTime() {
        TimePrimitive time = new TimePrimitive();
        try {
            time = timeService.callBlocking(new Empty()).time;
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return time;
    }

    public synchronized int getImgQuality() {
        return imgQuality;
    }


    public synchronized double getRateQuality() {
        return imgRate;
    }

    public synchronized double getRateTelemetry() {
        return teleRate;
    }

    public synchronized void setRateTelemetry(double rate) {
        teleRate = rate;
    }

    public synchronized void setToggleImgSent(boolean t) {
        toggleImgSent = t;
    }

    public synchronized void setBaseProduct(BaseProduct t) {
        mProduct = t;
    }

    public static Aircraft getAircraft() {

        return (Aircraft) DJISDKManager.getInstance().getProduct();
    }

    public BaseProduct getBaseProduct() {
        return DJISDKManager.getInstance().getProduct();
    }
    //endregion GETTER AND SETTERS SYNCHRONISED

    //send String/JSON to ROS
    public static synchronized void send(String msg) {
        client.send(msg);
    }

    //Obstacle Detection
    public Boolean startFrontObstacleDetection() {
        boolean result = false;
        if (getAircraft() != null && getAircraft().getFlightController() != null) {
            FlightAssistant intelligentFlightAssistant = getAircraft().getFlightController().getFlightAssistant();
            if (intelligentFlightAssistant != null) {
                result = true;
                intelligentFlightAssistant.setVisionDetectionStateUpdatedCallback(new VisionDetectionState.Callback() {
                    @Override
                    public void onUpdate(@NonNull VisionDetectionState visionDetectionState) {
                        setVisionDetection(visionDetectionState);
                    }
                });
                intelligentFlightAssistant.getActiveObstacleAvoidanceEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean) {
                        sendError("ActiveObstacleAvoidanceEnabled");
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        handleDjiError(djiError);
                    }
                });
                intelligentFlightAssistant.getCollisionAvoidanceEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean) {
                        sendError("CollisionAvoidanceEnabled");
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        handleDjiError(djiError);
                    }
                });

            } else {
                sendError("Flight Assistant not available");
            }
        }
        return result;
    }

    //Gimball Callback
    public Boolean startGimbalCallback() {
        boolean result = false;
        if (getAircraft() != null) {
            Gimbal gimbal = getAircraft().getGimbal();
            if (gimbal != null) {
                result = true;
                gimbal.setStateCallback(new GimbalState.Callback() {
                    @Override
                    public void onUpdate(@NonNull GimbalState gimbalState) {
                        setGimbalState(gimbalState);
                    }
                });
            } else {
                sendError("Gimbal NOT Available");
            }
        }
        return result;
    }

    //JSON HELPERS
    public static String jStart() {
        return "\"data\":\"{";
    }

    public static String jEnd() {
        return "}\"";
    }

    public static String jMsg(String topic, String msg) {
        if (!msg.equals("NaN")) {
            return "\\\"" + topic + "\\\":\\\"" + msg + "\\\"";
        } else {
            return "\\\"" + topic + "\\\":\\\"NotAvailable\\\"";
        }
    }

    public static String jMsg(String topic, int msg) {
        return "\\\"" + topic + "\\\":" + msg;
    }

    public static String jMsg(String topic, double msg) {
        if (!Double.isNaN(msg)) {
            return "\\\"" + topic + "\\\":" + String.format("%.3f", msg);
        } else {
            return "\\\"" + topic + "\\\": -1";
        }
    }

    //sends information messages to ROS (used primarily for debugging)
    public static void sendError(final String error_msg) {
        Thread thread = new Thread() {
            @Override
            public void run() {
                try {
                    // send msg
                    String data = jStart() + jMsg("Details:", error_msg) + jEnd();
                    String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[4]) + "\",\"msg\":{" + data + "}}";
                    Log.d(TAG, error_msg);
                    send(msg);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        };
        thread.start();
    }

    public static void reportObstacle(String msg) {
        Thread thread = new Thread() {
            @Override
            public void run() {
                try {
                    // send msg
                    String data = jStart() + jMsg("Details:", msg) + jEnd();
                    String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[9]) + "\",\"msg\":{" + data + "}}";
                    Log.d(TAG, "Obstacle Detected");
                    send(msg);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        };
        thread.start();
    }

    public void testPublishTelemetry() {
        String data;
        data = jStart() + jMsg("top1", "hello") + "," + jMsg("top2", 3.55) + jEnd();
        String msg = "{\"op\":\"publish\",\"topic\":\"" + getFullTopicName(topics[1]) + "\",\"msg\":{" + data + "}}";
        send(msg);
        setDisplayData(data);
    }

    public void onClick(View view) {
        if (view.getId() == R.id.btn_service) {
            rosStop = true;
            closeActivity();
        } else if (view.getId() == R.id.preFlightWidget) {
            ConnectDroneActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    PreFlightCheckListPanel panel = findViewById(R.id.preFlightPanel);
                    if (panel.getVisibility() == View.VISIBLE)
                        panel.setVisibility(View.GONE);
                    else
                        panel.setVisibility(View.VISIBLE);
                }
            });
        }

    }

    public static void sendDroneIds(boolean connected, CameraDrone camera) {
        try {

            Long tsLong = System.currentTimeMillis() / 1000;
            String ts = tsLong.toString();

            String data = jStart() + jMsg("Timestamp", ts) + "," + jMsg("DroneName", droneName) + "," + jMsg("Connected", connected ? "True" : "False") + "," + jMsg("Model", mProduct != null ? (mProduct.getModel() != null ? mProduct.getModel().toString() : "Unknown") : "Unknown");
            data += "," + jMsg("cameraVideoStreamSource", camera != null ? (camera.getCameraSourceMode() != null ? camera.getCameraSourceMode() : "unknown") : "unknow");

            data += "," + jMsg("cameraName", camera != null ? (camera.getCameraName() != null ? camera.getCameraName() : "unknown") : "unknown");

            data += jEnd();
            String helloMsg = "{\"op\":\"publish\",\"topic\":\"/droneIds\",\"msg\":{" + data + "}}";
            send(helloMsg);
            Log.d(TAG, "Sending Connected Message to ROS " + ts + "\n " + data);
        } catch (Exception e) {
            e.printStackTrace();
            Log.d("ConnectDroneActivity", "sendDroneIds error");
        }
    }

    public void tryingConnecting() {
        ((RCApplication) getApplication()).setCallBefore(true);
        final long timestart = System.currentTimeMillis();
        TimerTask timerTask = new TimerTask() {
            @Override
            public void run() {
                final long timecurrent = System.currentTimeMillis();

                if ((timecurrent - timestart) / 1000 > 20 || !((RCApplication) getApplication()).getTryconnect()) {
                    Notifications.show_notification("Failed to reconnect in 20 Seconds.Go from menu and select start ROS", activityConnect, true);
                    Log.d(TAG, "failed to trying reconnect");

                    cancel();
                } else {
                    boolean connected = ((RCApplication) getApplication()).rosReConnect();
                    Log.d(TAG, "trying reconnecting: " + connected);
                    if (connected) {

                        calledBeforeclose = false;
                        Log.d(TAG, "reconnecting successfully ");


                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                handleStreamRequest(false);
                            }
                        }, 1000);

                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                handleStreamRequest(true);
                            }
                        }, 2000);

                        ((RCApplication) getApplication()).setCallBefore(false);
                        ConnectDroneActivity.setRosEnabled(true);
                        MainActivity.setRosConnected(true);
                        rosSubscribe(true);

                        enableServices();
                        if (timerPublish)
                            rosPublishPicturesThread();
                        cancel();
                    }
                }

            }
        };
        Timer timer = new Timer();
        timer.schedule(timerTask, 10000, 500);
    }

    public static void showNetworkReconnectError() {
        activityConnect.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder alertDialog = new AlertDialog.Builder(activityConnect);
                alertDialog.setTitle("Network Not Reachable");
                alertDialog.setMessage("Unable to Connect to ROS due to Network Issue. Press Okay to Close the App, check your connection and try again!");
                AlertDialog accountDialog = alertDialog.create();

                alertDialog.setNegativeButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.dismiss();
                        activityConnect.finish();
                        System.exit(0);
                    }
                });

                alertDialog.setCancelable(true);
                alertDialog.show();
            }
        });
    }

    private void cameraPitchToZero() {
        Camera camera = Helper.getCameraInstance();
        Gimbal gimbal = Helper.getProductInstance().getGimbal();
        Rotation.Builder builder = new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(0).yaw(Rotation.NO_ROTATION).roll(Rotation.NO_ROTATION).time(2);
        Rotation rotation = builder.build();
        gimbal.rotate(rotation, null);
    }


}
