package com.dji.waypoint;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.os.Handler;
import android.util.Log;
import android.widget.TextView;
import androidx.annotation.Nullable;
import com.dji.cameras.CameraDrone;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.Helpers.SharedPreferences;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import coordinates.Coordinates;
import dji.common.error.DJIError;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecuteState;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;
import static com.dji.waypoint.Mission.num_media;
import static com.kios.rosDJI.ui.ConnectDroneActivity.sendError;


public class MissionListener implements WaypointMissionOperatorListener {
    int counter_cancel = 0;
    private int pointstart = 0;
    protected static boolean show;

    public static int photos;
    private Context c;
    public static boolean reachedpoint = false;
    private boolean reached0 = false;
    boolean stitch = true;
    private Activity a;

    private boolean timestarted = false;//boolean value to indicate if TimerTask started or not. TImerTask is responsible to take photos
    static Timer t;//TImer that initialize to take photos every x seconds


    BaseProduct product = Helper.getProductInstance();

    int speed = 4;


    Camera camera;
    ConnectDroneActivity connectDroneActivity;
    TextView msg_text;
    boolean abortTimer = false;
    Mission.MissionType missionType;

    /**
     * Function that stops TImer task which is responsle to take photos every x seconds
     */
    protected void stopListener() {
        abortTimer = false;
        if (t != null) {
            t.cancel();
            t.purge();
        }
    }

    public MissionListener(Context context, Activity activity, boolean stitch, Mission.MissionType missionType, ConnectDroneActivity connectDroneActivity, TextView msg_text) {
        this.missionType = missionType;
        stopListener();
        t = new Timer();
        this.c = context;
        this.a = activity;
        show = true;
        this.stitch = stitch;
        this.connectDroneActivity = connectDroneActivity;
        this.speed = speed;
        num_media = 0;
        startTime = null;
        SharedPreferences.setIntSharedPreference(a, "photos_mission", 0);
        SharedPreferences.setStringSharedPreference(a, "location_photos_mission", CameraDrone.getStorageLocation().name());
        SharedPreferences.setStringSharedPreference(a, "time_start_mission", null);

        camera = ConnectDroneActivity.getAircraft().getCameras().get(0);
        this.msg_text = msg_text;


    }

    protected static String startTime = null;


    public static void increaseMedia() {
        num_media += 1;
    }

    public static String getStartTime() {

        return startTime;
    }


    @Override
    public void onDownloadUpdate(final WaypointMissionDownloadEvent downloadEvent) {
    }

    @Override
    public void onUploadUpdate(final WaypointMissionUploadEvent uploadEvent) {
        if (uploadEvent.getProgress() != null
                && uploadEvent.getProgress().uploadedWaypointIndex == (uploadEvent.getProgress().totalWaypointCount - 1)) {
            Log.d("MissionListener", "onUploadUpdate: success");

            sendError("Upload successful!");
            Mission.startWaypointMission();
        }

    }


    int delay = 5;
    static WaypointMissionExecutionEvent exevent;

    int target = -2;

    @Override
    public void onExecutionUpdate(final WaypointMissionExecutionEvent executionEvent) {


        if (executionEvent != null)
            connectDroneActivity.setDisplayDataLeft("In Waypoint Mission\nStatus:\n Waypoint: " + (executionEvent.getProgress().targetWaypointIndex + 1) + "/" + executionEvent.getProgress().totalWaypointCount);

        exevent = executionEvent;

        if (stitch&&executionEvent!=null&&executionEvent.getProgress().isWaypointReached&&executionEvent.getProgress().targetWaypointIndex>0/*&&missionType.equals(Mission.MissionType.GRID)*/) {
            Log.d("MissionListener", "target vs total " + (executionEvent.getProgress().targetWaypointIndex + 1) + " " + executionEvent.getProgress().totalWaypointCount);

            if (!timestarted && stitch) {


                timestarted = true;

                int time = (int) ((Coordinates.getDistance() * 1.0 / speed * 1.0) * 1000.0);
                Log.d("MissionListener", "timer mission started: " + time);

                delay = 5;


                t = new Timer();
                t.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        if (abortTimer) {
                            Log.d("MissionListener", "stop timer");
                            cancel();

                        }
                        a.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                Log.d("MissionListener", "mission state" + exevent.getCurrentState());

                                if (!exevent.getProgress().executeState.equals(WaypointMissionExecuteState.CURVE_MODE_TURNING) && !exevent.getProgress().isWaypointReached) {


                                    captureAction(0);


                                } else {
                                    Log.d("MissionListener", "drone turning");

                                    new Handler().postDelayed(new Runnable() {
                                        @Override
                                        public void run() {
                                            Log.d("MissionListener", "delay 10");
                                            captureAction(0);


                                        }
                                    }, 10);
                                }


                            }
                        });

                    }
                }, delay, time);

            }
        }


    }


    @Override
    public void onExecutionStart() {
        android.text.format.DateFormat df = new android.text.format.DateFormat();
        startTime = String.valueOf(df.format("dd_MM_yyyy_HH_mm_ss", new Date()));

        SharedPreferences.setStringSharedPreference(a, "time_start_mission", startTime);
        a.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //new CameraDrone(a).recenter_camera();

            }
        });

        a.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                msg_text.setBackgroundColor( Color.parseColor("#AB3DDC84"));

            }
        });



    }

    @Override
    public void onExecutionFinish(@Nullable final DJIError error) {


        {

            SharedPreferences.setIntSharedPreference(a, "photos_mission", num_media);
            Mission.inProgress = false;
            Log.d("MissionListener", "onExecutionFinish start");

            if (t != null) {
                Log.d("MissionListener", "cancel timer");

                t.cancel();
                t.purge();
            }

            sendError("Execution finished");
            connectDroneActivity.setDisplayDataLeft("Waypoint Mission\nStatus:\n Execution finished");
            try {
            } catch (NullPointerException e) {
                e.printStackTrace();
            }
            a.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    msg_text.setBackgroundColor(0x68FFFFFF);

                }
            });
            Log.d("MissionListener", "onExecutionFinish end");

        }
    }


    // Method for taking photo
    private void captureAction(final int index) {

        if (camera != null) {


            camera.startShootPhoto(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        Log.d("MissionListener", "photo taken " + index);
                        num_media += 1;
                    } else {
                        Log.d("MissionListener", "photo failed " + djiError.toString());

                    }
                }
            });


        }
    }


}