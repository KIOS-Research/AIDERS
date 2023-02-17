package com.dji.waypoint;

import android.app.Activity;
import android.util.Log;
import android.widget.TextView;
import com.dji.cameras.CameraDrone;
import com.google.firebase.crashlytics.FirebaseCrashlytics;
import com.google.gson.Gson;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import com.kios.rosDJI.ui.MainActivity;
import com.loopj.android.http.AsyncHttpResponseHandler;
import com.loopj.android.http.RequestParams;
import com.loopj.android.http.SyncHttpClient;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import coordinates.Coordinates;
import cz.msebera.android.httpclient.Header;
import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.error.DJIWaypointV2Error;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointAction;
import dji.common.mission.waypoint.WaypointActionType;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionGotoWaypointMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.mission.waypointv2.Action.ActionTypes;
import dji.common.mission.waypointv2.Action.WaypointActuator;
import dji.common.mission.waypointv2.Action.WaypointCameraActuatorParam;
import dji.common.mission.waypointv2.Action.WaypointGimbalActuatorParam;
import dji.common.mission.waypointv2.Action.WaypointReachPointTriggerParam;
import dji.common.mission.waypointv2.Action.WaypointTrigger;
import dji.common.mission.waypointv2.Action.WaypointV2Action;
import dji.common.mission.waypointv2.Action.WaypointV2AssociateTriggerParam;
import dji.common.mission.waypointv2.WaypointV2;
import dji.common.mission.waypointv2.WaypointV2Mission;
import dji.common.mission.waypointv2.WaypointV2MissionTypes;
import dji.common.model.LocationCoordinate2D;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointV2MissionOperator;
import dji.sdk.sdkmanager.DJISDKManager;
import static com.kios.rosDJI.ui.ConnectDroneActivity.act;
import static com.kios.rosDJI.ui.ConnectDroneActivity.getRosEnabled;
import static com.kios.rosDJI.ui.MainActivity.mProduct;
import static java.lang.Thread.sleep;

public class Mission {

    enum MissionType {
        GRID, NORMAL;

    }


    static Activity activity;
    private static WaypointMissionOperator waypointMissionOperator;
    private static WaypointV2MissionOperator waypointMissionOperatorv2;


    private WaypointMission mission;
    private int WAYPOINT_COUNT = 0;
    static ConnectDroneActivity connectDroneActivity;
    int speed = 4;
    protected static int num_media;
    static boolean photoremote = false;
    double altitude = 30;
    String TAG = "Mission";
    TextView msg_det;
    static dji.common.mission.waypoint.WaypointMission.Builder mWaypointMission;
    static WaypointV2Mission.Builder mWaypointMission2;
    static MissionListener2 wp2 = null;
    static MissionListener wp = null;
    boolean uploadActionsV2;
    static MissionType missionType;

    public static boolean inProgress = false;

    public Mission(Activity activity, TextView msg_det, ConnectDroneActivity connectDroneActivity, boolean photoremote, int speed) {
        num_media = 0;
        this.activity = activity;
        this.msg_det = msg_det;
        this.connectDroneActivity = connectDroneActivity;
        this.photoremote = photoremote;
    }

    public static int getNum_media() {
        return num_media;
    }

    public void missionDji(final String missionDji) {
        try {
            if (waypointMissionOperator == null) {
                waypointMissionOperator = MissionControl.getInstance().getWaypointMissionOperator();
                connectDroneActivity.showToast("waypointMissionOperator NULL");
            }
            if (getRosEnabled()) {
                if (mProduct != null) {
                    JSONParser parser = new JSONParser();
                    JSONObject jsonObj = (JSONObject) parser.parse(missionDji);
                    JSONArray gpsInputs = (JSONArray) jsonObj.get("gpsInput");
                    String missionName = (String) jsonObj.get("name");
                    boolean grid = (boolean) jsonObj.get("grid");


                    JSONObject missionCommand = (JSONObject) jsonObj.get("missionCommand");
                    int command = (int) ((long) missionCommand.get("missionCommand"));
                    switch (command) {
                        case 0:
                            break;
                        case 1:
                            if (Helper.isM300Product())
                                waypointMissionOperatorv2.stopMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            else
                                waypointMissionOperator.stopMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            return;
                        case 2:
                            if (Helper.isM300Product())
                                waypointMissionOperatorv2.interruptMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            else
                                waypointMissionOperator.pauseMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            return;
                        case 3:
                            if (Helper.isM300Product())
                                waypointMissionOperatorv2.recoverMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            else
                                waypointMissionOperator.resumeMission(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        connectDroneActivity.handleDjiError(djiError);
                                    }
                                });
                            return;
                    }
                    photoremote = (boolean) (jsonObj.get("captureAndStoreImages") != null) ? (boolean) (jsonObj.get("captureAndStoreImages")) : false;

                    missionType = grid ? MissionType.GRID: MissionType.NORMAL;
                    initMissionManager(missionType);

                    if (grid) {

                        MissionListener2.grid = true;

                        Log.d("Mission", "grid true");
                        double lat[] = new double[2];
                        double lon[] = new double[2];
                        JSONObject gpsInput0 = (JSONObject) gpsInputs.get(0);
                        altitude = (double) gpsInput0.get("altitude");
                        double latitude = (double) gpsInput0.get("latitude");
                        lon[0] = latitude;
                        double longitude = (double) gpsInput0.get("longitude");
                        lat[0] = longitude;

                        gpsInput0 = (JSONObject) gpsInputs.get(1);
                        latitude = (double) gpsInput0.get("latitude");
                        lon[1] = latitude;
                        longitude = (double) gpsInput0.get("longitude");
                        lat[1] = longitude;

                        Coordinates coordinates = new Coordinates();
                        Log.d("Mission", "grid corners lat: " + Arrays.toString(lat));
                        Log.d("Mission", "grid corners lon: " + Arrays.toString(lon));

                        List<Coordinates> points = coordinates.create_grid(lat, lon, CameraDrone.getFov(), ((float) altitude));

                        prepareWayPointMission(points);
                        return;
                    }
                    Log.d(TAG, missionName);
                    WAYPOINT_COUNT = 0;

                    connectDroneActivity.showToast("Start MissionDJI");
                    activity.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            msg_det.setBackgroundColor(0x92FF872C);
                        }
                    });
                    connectDroneActivity.setDisplayDataLeft("Creating Mission");


                    WaypointMission.Builder waypointMissionBuilder = new WaypointMission.Builder().autoFlightSpeed(4f)
                            .maxFlightSpeed(8f)
                            .setExitMissionOnRCSignalLostEnabled(false)
                            .finishedAction(WaypointMissionFinishedAction.NO_ACTION)
                            .flightPathMode(WaypointMissionFlightPathMode.NORMAL)
                            .gotoFirstWaypointMode(WaypointMissionGotoWaypointMode.SAFELY)
                            .headingMode(WaypointMissionHeadingMode.AUTO)
                            .repeatTimes(1);

                    List<Waypoint> waypoints = new LinkedList<>();
                    boolean firstWaypoint = true;
                    double prevLat = 0.0;
                    double prevLon = 0.0;
                    double prevAlt = 0.0;


                    for (int i = 0; i < gpsInputs.size(); i++) {
                        JSONObject gpsInput = (JSONObject) gpsInputs.get(i);

//                        boolean isMissionAdvanced = (boolean) gpsInput.get("isMissionAdvanced");

                        // Mission is default
//                        if(!isMissionAdvanced){
                            double latitude = (double) gpsInput.get("latitude");
                            double longitude = (double) gpsInput.get("longitude");
                            double speed = (double) gpsInput.get("speed");
                            double altitude = (double) gpsInput.get("altitude");
                            long stay = (long) gpsInput.get("stayTime");
                            int stayTime = (int) stay;
                            boolean takePhoto = false;//photoremote;

                            if (!firstWaypoint) {

                                if ((!(prevLat == latitude)) || (!(prevLon == longitude)) || (!(prevAlt == altitude))) {
                                    Waypoint point = new Waypoint(latitude, longitude, (float) altitude);
                                    point.speed = (float) speed;
                                    WaypointAction action = new WaypointAction(WaypointActionType.STAY, stayTime);
                                    point.insertAction(action, 0);
                                    if (takePhoto)
                                        point.insertAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 1), 1);

                                    waypoints.add(point);
                                    WAYPOINT_COUNT++;
//                                showToast("waypoints.add point");
                                    prevLat = latitude;
                                    prevLon = longitude;
                                    prevAlt = altitude;
                                }
                            } else {
                                Waypoint point = new Waypoint(latitude, longitude, (float) altitude);
                                point.speed = (float) speed;
                                WaypointAction action = new WaypointAction(WaypointActionType.STAY, stayTime);
                                point.insertAction(action, 0);
                                if (takePhoto)
                                    point.insertAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 1), 1);
                                waypoints.add(point);
                                WAYPOINT_COUNT++;

                                prevLat = latitude;
                                prevLon = longitude;
                                prevAlt = altitude;
                                firstWaypoint = false;
                            }

                    }
                    if (photoremote&&!waypoints.isEmpty()){
                        Coordinates.setDistance(2.0 * waypoints.get(0).altitude * Math.tan(Math.toRadians(CameraDrone.getFov() / 2.0)));

                        waypoints.get(0).addAction(new WaypointAction(WaypointActionType.GIMBAL_PITCH,-90));
                    }
                    waypointMissionBuilder.waypointList(waypoints).waypointCount(waypoints.size());
                    mission = waypointMissionBuilder.build();
                    if (mission == null)
                        connectDroneActivity.showToast("mission null");

                    DJIError djiError = waypointMissionOperator.loadMission(mission);
                    connectDroneActivity.showToast("MissionDJI LOADED");
                    connectDroneActivity.handleDjiError(djiError);
                    if (djiError != null) {
                        activity.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                msg_det.setBackgroundColor(0x68FFFFFF);
                            }
                        });
                        connectDroneActivity.setDisplayDataLeft("Mission Error: " + djiError.getDescription());
                    }


                    final DJIError[] uploadError = new DJIError[1];
                    if (WaypointMissionState.READY_TO_RETRY_UPLOAD.equals(waypointMissionOperator.getCurrentState()) || WaypointMissionState.READY_TO_UPLOAD.equals(waypointMissionOperator.getCurrentState())) {
                        waypointMissionOperator.uploadMission(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                connectDroneActivity.handleDjiError(djiError);
                                if (djiError == null)
                                    uploadError[0] = djiError;
                            }
                        });
                    }
                    if (uploadError[0] == null) {
                        int q = 0;
                        do {
                            try {
                                if (!WaypointMissionState.READY_TO_EXECUTE.equals(waypointMissionOperator.getCurrentState())) {
                                    connectDroneActivity.showToast("Uploading...");
                                    sleep(2000);
                                }
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            q++;
                        } while (!WaypointMissionState.READY_TO_EXECUTE.equals(waypointMissionOperator.getCurrentState()) && q < (WAYPOINT_COUNT / 20));
                    } else {
                        activity.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                msg_det.setBackgroundColor(0x68FFFFFF);
                            }
                        });
                    }

                    if (WaypointMissionState.READY_TO_EXECUTE.equals(waypointMissionOperator.getCurrentState()) && mission != null) {
                        waypointMissionOperator.startMission(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                connectDroneActivity.handleDjiError(djiError);
                                if (djiError == null) {
                                    try {
                                        String filename = "MissionLOG_" + connectDroneActivity.getFileTimestamp() + ".txt";
                                    } catch (NullPointerException e) {
                                        FirebaseCrashlytics.getInstance().recordException(new Throwable(e.toString()));
                                        Log.e(TAG, e.toString());
                                    }
                                } else {
                                }
                            }
                        });
                    }
                }
            }
        } catch (Exception e) {
            Log.e("MISSIONerr", e.getMessage());
            connectDroneActivity.setDisplayDataLeft(e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Get waypoint mission operator
     *
     * @return
     */
    public static WaypointV2MissionOperator getWaypointMissionOperator2() {
        if (waypointMissionOperatorv2 == null) {
            try {

                waypointMissionOperatorv2 = DJISDKManager.getInstance().getMissionControl().getWaypointMissionV2Operator();
            } catch (Exception e) {
            }
        }
        return waypointMissionOperatorv2;
    }

    List<WaypointV2Action> waypointV2Actions;
    static List<Coordinates> drone_move;

    /**
     * This function configures the mission variables before mission starts.
     * "Also this function use
     * to update the remainINg path of a mission which was stop because of low battery"
     *
     * @todo meaning?
     */
    protected void configWayPointMission2(List<Coordinates> drone_move) throws NullPointerException {

        BaseProduct product = Helper.getProductInstance();
        if (product != null) {

            if (mWaypointMission2 != null) {
                if (mWaypointMission2.getWaypointList() != null) {
                    mWaypointMission2.getWaypointList().clear(); // Remove all the waypoints added to the task
                }


                mWaypointMission2.setAutoFlightSpeed(speed);
                mWaypointMission2.setFinishedAction(WaypointV2MissionTypes.MissionFinishedAction.NO_ACTION);
                mWaypointMission2.setMaxFlightSpeed(8);

                if (drone_move != null) {

                    Log.d("AMAL", "HEREE SOMEHOW");

                            for (Coordinates c : drone_move) {
                                if (!c.include)
                                    continue;


                                WaypointV2.Builder waypointb = new WaypointV2.Builder();
                                waypointb.setCoordinate(new LocationCoordinate2D(c.lat, c.lon));
                                waypointb.setAltitude(altitude);
                                waypointb.setFlightPathMode(WaypointV2MissionTypes.WaypointV2FlightPathMode.GOTO_POINT_STRAIGHT_LINE_AND_STOP);
                                waypointb.setHeadingMode(WaypointV2MissionTypes.WaypointV2HeadingMode.AUTO);
                                if (mWaypointMission2 != null) {
                                    mWaypointMission2.addWaypoint(waypointb.build());

                                }
                            }

                        if (photoremote) {
                            Log.d(TAG, "photoremote enable");
                            WaypointV2Action waypointV2Action = setGimbalWaypointAction(0, 0, -90);
                            waypointV2Actions.add(waypointV2Action);
                        }

                }

            }

        }

        if (mWaypointMission2 != null) {
            DJIError error = null;


            getWaypointMissionOperator2().loadMission(mWaypointMission2.build(), new CommonCallbacks.CompletionCallback<DJIWaypointV2Error>() {
                @Override
                public void onResult(DJIWaypointV2Error djiWaypointV2Error) {
                    if (error != null) {

                        inProgress = false;

                        connectDroneActivity.showToast(error.getDescription().toString());
                    } else {
                    }
                }
            });

            Log.d(TAG, "waypoint count2: " + ((mWaypointMission2 != null) ? mWaypointMission2.getWaypointCount() : "null"));
        }
        Log.d("Mission", "confiq finished");
    }

    /**
     * Configures and upload current mission information (speed, height, coordinates)
     * on the drone flight controller
     *
     * @return void
     */
    public void prepareWayPointMission(List<Coordinates> drone_move) {

        MissionListener.show = true;


        if (Helper.isM300Product()) {
            configWayPointMission2(drone_move);

        }
        else
            configWayPointMission(drone_move);


        if (!Helper.isM300Product() && getWaypointMissionOperator() != null) {


            getWaypointMissionOperator().uploadMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(final DJIError error) {

                    if (error == null) {
                        Log.d("Mission", "upload success");

                    } else {
                        inProgress = false;
                        Log.d("Mission", "upload error :" + error.toString());


                        connectDroneActivity.showToast(error.toString());
                    }

                }
            });


        } else {

            getWaypointMissionOperator2().uploadMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(final DJIError error) {


                    if (error == null) {
                        Log.d("Mission", "upload success");


                    } else {
                        Log.d("Mission", "upload error :" + error.toString());

                        inProgress = false;

                    }

                }
            });

        }
    }

    /**
     * Get waypoint mission operator
     *
     * @return
     */
    public static WaypointMissionOperator getWaypointMissionOperator() {
        if (waypointMissionOperator == null) {
            try {

                waypointMissionOperator = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
            } catch (Exception e) {
            }
        }
        return waypointMissionOperator;
    }

    /**
     * Get waypoint mission operator
     *
     * @return
     */
    public static void removeWaypointMissionOperator() {
        if (getWaypointMissionOperator() != null) {
            try {

                getWaypointMissionOperator().removeListener(wp);
            } catch (Exception e) {
            }
        }
    }


    /**
     * Get waypoint mission operator
     *
     * @return
     */
    public static void removeWaypointMissionOperatorV2() {
        if (getWaypointMissionOperator2() != null) {
            try {

                getWaypointMissionOperator2().removeWaypointListener(wp2);
            } catch (Exception e) {
            }
        }
    }

    /**
     * This function initializes the listener of the waypoint mission.
     * The listener is active while the mission executes
     *
     * @return void
     **/
    public void initMissionManager(Mission.MissionType missionType) {

        Mission.photoremote = photoremote;
        removeWaypointMissionOperator();
        removeWaypointMissionOperatorV2();
        mWaypointMission = new dji.common.mission.waypoint.WaypointMission.Builder();
        mWaypointMission2 = new WaypointV2Mission.Builder();


        if (Helper.isM300Product())
            wp2 = new MissionListener2(activity, activity, photoremote, missionType, connectDroneActivity, msg_det, this);
        else
            wp = new MissionListener(activity, activity, photoremote, missionType, connectDroneActivity, msg_det);


        if (Helper.isM300Product())
            getWaypointMissionOperator2().addWaypointEventListener(wp2);
        else
            getWaypointMissionOperator().addListener(wp);

    }

    private WaypointV2Action setGimbalWaypointAction(int index, int actionid, int angle) {
        WaypointTrigger.Builder trigger = new WaypointTrigger.Builder();
        trigger.setTriggerType(ActionTypes.ActionTriggerType.ASSOCIATE);

        trigger.setAssociateParam(new WaypointV2AssociateTriggerParam.Builder()
                .setAssociateActionID(actionid + 1)
                .setAssociateType(ActionTypes.AssociatedTimingType.AFTER_FINISHED)
                .setWaitingTime(0)
                .build());
        WaypointReachPointTriggerParam.Builder triggerParam = new WaypointReachPointTriggerParam.Builder();
        triggerParam.setStartIndex(index);
        triggerParam.setAutoTerminateCount(0);
        trigger.setReachPointParam(triggerParam.build());
        trigger.setTriggerType(ActionTypes.ActionTriggerType.REACH_POINT);
        WaypointV2Action.Builder builder = new WaypointV2Action.Builder();
        builder.setActionID(actionid + 1);
        WaypointActuator.Builder waypointAct = new WaypointActuator.Builder();
        WaypointGimbalActuatorParam.Builder WaypointGimbalActuatorParam = new WaypointGimbalActuatorParam.Builder();
        WaypointGimbalActuatorParam.operationType(ActionTypes.GimbalOperationType.ROTATE_GIMBAL);
        Rotation.Builder rotationB = new Rotation.Builder();
        rotationB.pitch(-angle);
        rotationB.roll(0);
        rotationB.yaw(0);
        rotationB.time(2);
        rotationB.mode(RotationMode.RELATIVE_ANGLE);

        WaypointGimbalActuatorParam.rotation(rotationB.build());
        waypointAct.setGimbalActuatorParam(WaypointGimbalActuatorParam.build());
        waypointAct.setActuatorType(ActionTypes.ActionActuatorType.GIMBAL);
        builder.setActuator(waypointAct.build());
        builder.setTrigger(trigger.build());
        builder.setActionID(actionid);
        WaypointV2Action waypointV2Action = builder.build();
        return waypointV2Action;
    }

    private WaypointV2Action setWaypointActionCamerav2(ActionTypes.CameraOperationType cameraOperationType, int index) {
        WaypointTrigger.Builder trigger = new WaypointTrigger.Builder();
        WaypointReachPointTriggerParam.Builder triggerParam = new WaypointReachPointTriggerParam.Builder();
        triggerParam.setStartIndex(index);
        triggerParam.setAutoTerminateCount(0);
        trigger.setReachPointParam(triggerParam.build());
        trigger.setTriggerType(ActionTypes.ActionTriggerType.REACH_POINT);

        WaypointV2Action.Builder builder = new WaypointV2Action.Builder();
        WaypointActuator.Builder waypointAct = new WaypointActuator.Builder();

        WaypointCameraActuatorParam.Builder WaypointCameraActuatorParam = new WaypointCameraActuatorParam.Builder();

        Rotation.Builder rotationB = new Rotation.Builder();
        rotationB.pitch(-90);
        WaypointCameraActuatorParam.setCameraOperationType(cameraOperationType);
        waypointAct.setCameraActuatorParam(WaypointCameraActuatorParam.build());
        waypointAct.setActuatorType(ActionTypes.ActionActuatorType.CAMERA);
        builder.setActuator(waypointAct.build());
        builder.setTrigger(trigger.build());
        builder.setActionID(index);
        WaypointV2Action waypointV2Action = builder.build();
        return waypointV2Action;
    }

    /**
     * This function configures the mission variables before mission starts.
     * "Also this function use
     * to update the remainINg path of a mission which was stop because of low battery"
     *
     * @todo meaning?
     */
    public void configWayPointMission(List<Coordinates> drone_move) throws NullPointerException {


        BaseProduct product = Helper.getProductInstance();
        if (product != null) {

            if (mWaypointMission != null) {
                if (mWaypointMission.getWaypointList() != null) {
                    mWaypointMission.getWaypointList().clear(); // Remove all the waypoints added to the task
                }

                mWaypointMission.flightPathMode(WaypointMissionFlightPathMode.CURVED);


                mWaypointMission.gotoFirstWaypointMode(WaypointMissionGotoWaypointMode.SAFELY);
                mWaypointMission.headingMode(WaypointMissionHeadingMode.AUTO);
                mWaypointMission.autoFlightSpeed(speed);
                mWaypointMission.finishedAction(WaypointMissionFinishedAction.NO_ACTION);
                mWaypointMission.maxFlightSpeed(8);


                if (drone_move != null) {


                    {

                                mWaypointMission.setGimbalPitchRotationEnabled(false);

                                for (Coordinates c : drone_move) {
                                    if (!c.include)
                                        continue;


                                    Waypoint mWaypoint = new Waypoint(
                                            c.lat,
                                            c.lon,
                                            ((float) altitude));

                                    //Add Waypoints to Waypoint arraylist;
                                    if (mWaypointMission != null) {
                                        mWaypointMission.addWaypoint(mWaypoint);
                                    }
                                }




                    }
                    if (photoremote) {
                        Log.d(TAG, "photoremote enable");

                        mWaypointMission.getWaypointList().get(0).addAction(new WaypointAction(WaypointActionType.GIMBAL_PITCH, -90));

                    }

                }
            }

        }


        if (mWaypointMission != null) {
            Log.d(TAG, "total distance mission: " + mWaypointMission.calculateTotalDistance());
            DJIError error = null;
            error = getWaypointMissionOperator().loadMission(mWaypointMission.build());
            inProgress = (error == null);


            Log.d(TAG, "waypoint count2: " + ((mWaypointMission != null) ? mWaypointMission.getWaypointCount() : "null"));


            if (error != null) {
                sendMessageToROS(error.toString());
                Log.d(TAG, "loadMission error: " + error.toString());
            }
        }


        Log.d("Mission", "confiq finished 1");

    }

    /**
     * This function give order to drone to start execute mission.
     * Also turns the gimbal of the camra 90 degrees if the drone is going to take photo
     *
     * @return void
     */
    public static void startWaypointMission() {
        final BaseProduct product = Helper.getProductInstance();

        //check if distance between last waypoint and home location is less than 20m
        FlightController mFlightController = null;


        if (getWaypointMissionOperator() != null) {
            getWaypointMissionOperator().startMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    if (error == null) {

                        activity.runOnUiThread(new Runnable() {
                            public void run() {
                                if (photoremote) {//rotate gimbal 90 degrees to take photo

                                    new CameraDrone().switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0);
                                }


                                if (photoremote) {


                                    try {
                                        Helper.getProductInstance().getGimbal().reset(new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {


                                            }
                                        });

                                    } catch (NullPointerException e) {

                                    }


                                }


                            }
                        });

                    } else {
                        connectDroneActivity.setDisplayDataLeft(error.toString());

                    }
                    sendMessageToROS(error != null ? error.toString() : null);

                }
            });
        }
    }


    /**
     * This function give order to drone to start execute mission.
     * Also turns the gimbal of the camera 90 degrees if the drone is going to take photo
     *
     * @return void
     */
    public static void startWaypointMissionV2() {
        final BaseProduct product = Helper.getProductInstance();

        if (getWaypointMissionOperator2() != null) {
            getWaypointMissionOperator2().recoverMission(new CommonCallbacks.CompletionCallback<DJIWaypointV2Error>() {
                @Override
                public void onResult(DJIWaypointV2Error djiWaypointV2Error) {

                }
            });

            getWaypointMissionOperator2().startMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    if (error == null) {


                        act.runOnUiThread(new Runnable() {
                            public void run() {
                                if (photoremote) {//rotate gimbal 90 degrees to take photo

                                    new CameraDrone().switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0
                                    );
                                }


                                if (photoremote) {


                                    try {
                                        Helper.getProductInstance().getGimbal().reset(new CommonCallbacks.CompletionCallback() {
                                            @Override
                                            public void onResult(DJIError djiError) {


                                            }
                                        });

                                    } catch (NullPointerException e) {

                                    }


                                }


                            }
                        });

                    } else {
                        inProgress = false;

                        connectDroneActivity.setDisplayDataLeft(error.toString());

                    }
                }
            });
        }
    }

    private static void sendMessageToROS(String error) {
        RequestParams params = new RequestParams();

        if (error != null)
            params.put("error", error);
        else if (missionType.equals(MissionType.GRID)) {
            params.put("path_grid", drone_move);
        }
        params.put("drone_name", MainActivity.droneName);
        Gson gson = new Gson();
        String json = gson.toJson(params);


        Log.d("Mission", " post values:[" + json + "]");

        SyncHttpClient client = new SyncHttpClient();
        Log.d("Mission", "start post:[" + MainActivity.masterIP + "]");

        client.post("http://" + MainActivity.masterIP + "/grid_missions/result", params, new AsyncHttpResponseHandler() {

            @Override
            public void onStart() {
                Helper.showToast(activity, "sending mission status");

                Log.d("SendPhotosToROS", "onStart post");
            }

            @Override
            public void onSuccess(int statusCode, Header[] headers, byte[] response) {
                Helper.showToast(activity, "sent message successfully");

                Log.d("SendPhotosToROS", "onSuccess post");


                // called when response HTTP status is "200 OK"
            }

            @Override
            public void onFailure(int statusCode, Header[] headers, byte[] errorResponse, Throwable e) {
                Helper.showToast(activity, "failed to send ");

                Log.d("SendPhotosToROS", "onFailure post: " + statusCode);

                // called when response HTTP status is "4XX" (eg. 401, 403, 404)
            }

            @Override
            public void onRetry(int retryNo) {
                Log.d("SendPhotosToROS", "onRetry");

                // called when request is retried
            }
        });
        Log.d("Mission", "end post");
    }



}
