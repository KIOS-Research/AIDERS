package com.dji.waypoint;

import android.app.Activity;
import android.content.Context;
import android.os.Handler;
import android.util.Log;
import android.widget.TextView;
import androidx.annotation.Nullable;
import com.dji.FlightController;
import com.dji.download.MediaDownload;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.Helpers.SharedPreferences;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import java.util.Timer;
import java.util.TimerTask;
import coordinates.Coordinates;
import dji.common.error.DJIError;
import dji.common.error.DJIWaypointV2Error;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.waypointv2.Action.ActionDownloadEvent;
import dji.common.mission.waypointv2.Action.ActionExecutionEvent;
import dji.common.mission.waypointv2.Action.ActionState;
import dji.common.mission.waypointv2.Action.ActionUploadEvent;
import dji.common.mission.waypointv2.WaypointV2MissionDownloadEvent;
import dji.common.mission.waypointv2.WaypointV2MissionExecutionEvent;
import dji.common.mission.waypointv2.WaypointV2MissionState;
import dji.common.mission.waypointv2.WaypointV2MissionUploadEvent;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.waypoint.WaypointV2ActionListener;
import dji.sdk.mission.waypoint.WaypointV2MissionOperatorListener;
import dji.sdk.products.Aircraft;
import static com.dji.waypoint.Mission.num_media;


public class MissionListener2 implements WaypointV2MissionOperatorListener {

    private boolean stop_photo = false;
    int speed = 4;
    protected static int reachedfirst = 0;
    protected static boolean show = false;
    protected static boolean grid;
    public static int photos;
    public static int Waypoint_Count;
    private Context c;
    public static int passed = 0;
    public static boolean reachedpoint = false;
    boolean photoremote;
    private Activity a;
    int target = -2;

    private boolean timestarted = false;//boolean value to indicate if TimerTask started or not. TImerTask is responsible to take photos
    static Timer t;//TImer that initialize to take photos every x seconds
    int delay = 5;
    static WaypointV2MissionExecutionEvent exevent;
    private double previousyaw = 0;
    ConnectDroneActivity connectDroneActivity;
    TextView msg_det;
    BaseProduct product = ConnectDroneActivity.getAircraft();


    /**
     * Function that stops TImer task which is responsle to take photos every x seconds
     */
    protected static void stopListener() {
        if (t != null)
            t.cancel();
    }

    /**
     * Function that stops TImer task which is responsle to take photos every x seconds
     */
    protected void startTimerListener() {
        int time = (int) ((Coordinates.getDistance() * 1.0 / speed * 1.0) * 1000.0);
        Log.d("MissionListenerV2", "timer mission started: " + time);

        delay = 1000;
        if (t != null && !timestarted) {
            timestarted = true;

            t = new Timer();
            t.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (stop_photo) {
                        return;
                    }
                    a.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Log.d("MissionListenerV2", "mission state" + exevent.getCurrentState());
                            Log.d("MissionListenerV2", "yaw compare: " + (FlightController.getYaw() - previousyaw));


                            if ((FlightController.getYaw() - previousyaw) < 5 && !exevent.getProgress().isWaypointReached()) {


                                captureAction(0);


                            } else {
                                Log.d("MissionListenerV2", "drone turning");

                                new Handler().postDelayed(new Runnable() {
                                    @Override
                                    public void run() {
                                        Log.d("MissionListenerV2", "delay 10");
                                        captureAction(0);


                                    }
                                }, 10);


                            }
                            previousyaw = FlightController.getYaw();

                        }
                    });

                }
            }, delay, time);
        }

    }

    Mission mission;
    Mission.MissionType missionType;

    public MissionListener2(Context context, Activity activity, boolean photoremote, Mission.MissionType missionType, ConnectDroneActivity connectDroneActivity, TextView msg_det, Mission mission) {
        this.missionType = missionType;
        ;
        this.connectDroneActivity = connectDroneActivity;
        stopListener();
        t = new Timer();
        this.c = context;
        this.a = activity;
        show = true;
        reachedpoint = false;
        this.photoremote = photoremote;
        this.msg_det = msg_det;
        this.mission = mission;


    }

    @Override
    public void onExecutionStopped() {
        Log.d("MissionListenerV2", "onExecutionStopped");

        connectDroneActivity.setDisplayDataLeft("Waypoint Mission\nStatus:\n Mission Cancelled Successfully");

        cameraPitchToZero();

    }


    @Override
    public void onDownloadUpdate(final WaypointV2MissionDownloadEvent downloadEvent) {
    }


    @Override
    public void onUploadUpdate(final WaypointV2MissionUploadEvent uploadEvent) {

        if(uploadEvent !=null && uploadEvent.getProgress() != null)
        {
            Waypoint_Count = uploadEvent.getProgress().getTotalWaypointCount();
        }

        if (uploadEvent.getPreviousState() == WaypointV2MissionState.UPLOADING
                    && uploadEvent.getCurrentState() == WaypointV2MissionState.READY_TO_EXECUTE) {

                if(grid == true){

                    Mission.startWaypointMissionV2();
                    grid = false;
                }

            }


    }


    @Override
    public void onExecutionUpdate(final WaypointV2MissionExecutionEvent executionEvent) {

        if(executionEvent !=null && executionEvent.getProgress() != null)
        {
            if(num_media >0){
                connectDroneActivity.setDisplayDataLeft("In Waypoint Mission\nStatus:\n Waypoint: " + (executionEvent.getProgress().getTargetWaypointIndex() + 1) + "/" + (Waypoint_Count+1)
                        + "\n Photos taken: " + num_media
                );
            }else{
                connectDroneActivity.setDisplayDataLeft("In Waypoint Mission\nStatus:\n Waypoint: " + (executionEvent.getProgress().getTargetWaypointIndex() + 1) + "/" + (Waypoint_Count+1));
            }

        }


        if (executionEvent.getProgress() != null) {

        }



        if (executionEvent.getError() != null)
            Log.d("MissionListenerV2", "waypoint execution error: " + executionEvent.getError().toString());


        if (executionEvent.getProgress() == null || executionEvent.getCurrentState().equals(WaypointV2MissionState.READY_TO_EXECUTE) || executionEvent.getCurrentState().equals(WaypointV2MissionState.READY_TO_UPLOAD))
            return;


        exevent = executionEvent;
        Log.d("MissionListenerV2", "current exec state" + exevent.getProgress().getExecuteState().toString());

        if (photoremote) {

            if (!timestarted && photoremote&&executionEvent.getProgress().isWaypointReached()) {
                startTimerListener();


            }
        }


    }


    @Override
    public void onExecutionStart() {
        Log.d("MissionListenerV2", "execution start " + photoremote);

        Log.d("MissionListenerV2", MediaDownload.getCurrentDateTime_underscore());

        SharedPreferences.setStringSharedPreference((Activity) c,"time_start_mission", MediaDownload.getCurrentDateTime_underscore()) ;

        a.runOnUiThread(new Runnable() {
            @Override
            public void run() {

            }
        });

    }

    @Override
    public void onExecutionFinish(@Nullable final DJIWaypointV2Error error) {
        {
            Log.d("MissionListenerV2", "num media final on executionfinished: " + num_media);



            if (t != null) {
                if (photoremote)
                    captureAction(0);
                t.cancel();
                t.purge();
            }

            cameraPitchToZero();

        }

        connectDroneActivity.setDisplayDataLeft("Waypoint Mission\nStatus:\n Execution finished");

    }


    // Method for taking photo
    private void captureAction(final int index) {
        final Camera camera = ConnectDroneActivity.getAircraft().getCameras().get(0);
        if (camera != null) {


            camera.startShootPhoto(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        Log.d("MissionListenerV2", "photo taken " + index);
                        num_media += 1;


                        ((Activity) c).runOnUiThread(new Runnable() {
                            public void run() {
                                try {
                                    float heading = ((Aircraft) product).getFlightController().getCompass().getHeading();

                                } catch (IllegalArgumentException e) {

                                }
                                Log.d("MissionListenerV2", "repeat_mission index: " + index);


                            }
                        });


                    } else {
                        Log.d("MissionListenerV2", "photo failed " + djiError.toString());

                    }
                }
            });


        }
    }

    private void cameraPitchToZero() {
        Camera camera = Helper.getCameraInstance();
        Gimbal gimbal = Helper.getProductInstance().getGimbal();
        Rotation.Builder builder = new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(0).yaw(Rotation.NO_ROTATION).roll(Rotation.NO_ROTATION).time(2);
        Rotation rotation = builder.build();
        gimbal.rotate(rotation, null);
    }



}