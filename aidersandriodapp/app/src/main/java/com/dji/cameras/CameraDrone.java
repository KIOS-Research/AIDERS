package com.dji.cameras;

import android.app.Activity;
import android.util.Log;
import android.view.MenuItem;
import androidx.annotation.NonNull;
import com.dji.waypoint.Mission;
import com.google.gson.Gson;
import com.jilk.ros.entities.Point;
import com.jilk.ros.message.PointCloud;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.R;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import org.jetbrains.annotations.NotNull;
import java.util.List;
import dji.common.camera.CameraVideoStreamSource;
import dji.common.camera.SettingsDefinitions;
import dji.common.camera.StorageState;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.mission.MissionState;
import dji.common.perception.DJILidarPointCloudRecord;
import dji.common.util.CommonCallbacks;
import dji.sdk.camera.Lens;
import dji.sdk.lidar.Lidar;
import dji.sdk.lidar.reader.PointCloudLiveViewData;
import dji.sdk.products.Aircraft;
import static com.kios.rosDJI.ui.ConnectDroneActivity.act;
import static com.kios.rosDJI.ui.ConnectDroneActivity.getActivity;

public class CameraDrone implements Lidar.DJIPointCloudLiveDataListener {
    String cameraSource = null;
    public List<dji.sdk.camera.Camera> cameras;
    boolean isH20T = false;
    private boolean flag = false;
    public static float fov = 47.8f;

    public static float fovhor = fov;
    private static float aspect = 16.0f / 9.0f;
    private float overlap = 80f;
    final String TAG = "CameraDrone";
    static SettingsDefinitions.StorageLocation storageLocation;
    static SettingsDefinitions.CameraMode cameraMode;

    public CameraDrone() {
        try {
            cameras = ConnectDroneActivity.getAircraft().getCameras();
            isH20T = cameras != null ? (cameras.get(0).getDisplayName().equalsIgnoreCase(dji.sdk.camera.Camera.DisplayNameZenmuseH20) || cameras.get(0).getDisplayName().equalsIgnoreCase(dji.sdk.camera.Camera.DisplayNameZenmuseH20T)) : false;
            Log.d("Camera", "Camera: not null cameras");

        } catch (NullPointerException e) {
            e.printStackTrace();

        }
    }

    public static SettingsDefinitions.StorageLocation getStorageLocation() {
        return storageLocation;
    }

    public static float getFov() {
        return fov;
    }

    public static void setFovOverlap(float overlap) {
         fov = fovhor * ((100f-overlap)/100f);
    }

    public String getCameraName() {
        try {
            return cameras != null ? cameras.get(0).getDisplayName().replace(" ", "_") : null;
        } catch (NullPointerException e) {
            return null;
        }
    }


    public SettingsDefinitions.CameraMode getCameraMode() {
        return cameraMode;
    }
    public String getCameraSourceMode() {
        return cameraSource;
    }
    public boolean isH20T() {
        return isH20T;
    }

    public void cameraCallBack() {
        if (cameras != null && cameras.size() > 0) {
            cameras.get(0).setSystemStateCallback(null);
            cameras.get(0).setStorageStateCallBack(null);

            cameras.get(0).setStorageStateCallBack(new StorageState.Callback() {
                @Override
                public void onUpdate(@NonNull @NotNull StorageState storageState) {
                    storageLocation = storageState.getStorageLocation();

                }
            });
            cameras.get(0).setSystemStateCallback(new SystemState.Callback() {
                @Override
                public void onUpdate(@NonNull @NotNull SystemState systemState) {
                   cameraMode= systemState.getMode();

                    Log.d(TAG, "setSystemStateCallback: " + systemState.isShootingSinglePhoto() + " , " + systemState.isShootingBurstPhoto() + " , " + systemState.isShootingIntervalPhoto());
                    if (Mission.getWaypointMissionOperator().getCurrentState().equals(MissionState.EXECUTING) && systemState.isStoringPhoto()) {
                        Log.d(TAG, "setSystemStateCallback: shooting mission photo: ");
                    }

                }
            });
        }
    }

    public void getCameraVideoStreamSource() {
        if (cameras == null) {
            return;
        }
        try {
            Log.d("Camera", "getCameraVideoStreamSource: not null");

            if (cameras.get(0).isMultiLensCameraSupported())
                cameras.get(0).getCameraVideoStreamSource(new CommonCallbacks.CompletionCallbackWith<CameraVideoStreamSource>() {
                    @Override
                    public void onSuccess(CameraVideoStreamSource cameraVideoStreamSource) {
                        Log.d("Camera", "getCameraVideoStreamSource: onSuccess");
                        cameraSource = cameraVideoStreamSource.name();
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        Log.d("Camera", "getCameraVideoStreamSource: onFailure");
                        cameraSource = "unknown...test";
                    }
                });
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }


    public void stopReadPointCloudLiveViewData() {
        if (ConnectDroneActivity.getAircraft() == null)
            return;
        try {
            List<Lidar> lidar = ConnectDroneActivity.getAircraft().getLidars();
            if (lidar == null)
                return;
            Log.d("CameraDrone", "lidar exist");

            lidar.get(0).removeAllPointCloudStatusListener();
            lidar.get(0).stopReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback<DJIError>() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError != null)

                        Log.d("CameraDrone", "error startReadPointCloudLiveViewData: " + djiError.toString());
                    else
                        Log.d("CameraDrone", " startReadPointCloudLiveViewData successfully");
                }
            });

        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }

    public void startReadPointCloudLiveViewData() {
        try {
            List<Lidar> lidar = ConnectDroneActivity.getAircraft().getLidars();
            if (lidar == null)
                return;
            Log.d("CameraDrone", "lidar exist");

            lidar.get(0).addPointCloudLiveViewDataListener(this);
            lidar.get(0).startReadPointCloudLiveViewData(new CommonCallbacks.CompletionCallback<DJIError>() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError != null)

                        Log.d("CameraDrone", "error startReadPointCloudLiveViewData: " + djiError.toString());
                    else
                        Log.d("CameraDrone", " startReadPointCloudLiveViewData successfully");
                }
            });
            DJILidarPointCloudRecord djiLidarPointCloudRecord = DJILidarPointCloudRecord.START_POINT_CLOUD_RECORDING;
            lidar.get(0).pointCloudRecord(djiLidarPointCloudRecord, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError != null)
                        Log.d("CameraDrone", "error pointCloudRecord: " + djiError.toString());
                    else
                        Log.d("CameraDrone", " pointCloudRecord successfully");
                }
            });
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }

    public void setShootPhotoMode(SettingsDefinitions.ShootPhotoMode shootPhotoMode) {
        try {
            cameras.get(0).setShootPhotoMode(shootPhotoMode, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                }
            });
        } catch (NullPointerException e) {

        }

    }

    public boolean setStorageLocation(SettingsDefinitions.StorageLocation storageLocation) {
        flag = false;

        try {
            cameras.get(0).setStorageLocation(storageLocation, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    String error = "success";
                    if (djiError == null) {
                        flag = true;
                    } else
                        error = djiError.toString();
                    Log.d("Camera", "setStorageLocation: onresult: " + error);
                }
            });
        } catch (NullPointerException e) {

        }

        Log.d("Camera", "setStorageLocation: answerrr : " + storageLocation.name() + ": " + flag);
        return flag;
    }


    public void onReceiveLiveViewData(PointCloudLiveViewData[] var1, int var2) {
        int counter = 0;
        PointCloud pointCloud = new PointCloud();
        for (PointCloudLiveViewData pointCloudLiveViewData : var1) {
            pointCloud.points.add(new Point(pointCloudLiveViewData.x, pointCloudLiveViewData.y, pointCloudLiveViewData.z));
            if (counter > 20)
                break;
            counter++;
        }
        Log.d("CameraDrone", "onReceiveLiveViewData pointCloud: " + (pointCloud.points.size() > 0 ? pointCloud.points.get(0).toString() : "null"));

        Gson gson = new Gson();
        String json = gson.toJson(pointCloud);
        Log.d("CameraDrone", "onReceiveLiveViewData msg ros: " + json);
    }

    public void onError(DJIError var1) {
        if (var1 != null)
            Log.d("CameraDrone", "onReceiveLiveViewData error: " + var1.toString());
    }

    /**
     * This function enables switching the mode of the camera from capturing
     * photos to capturing video and vice-verca.
     *
     * @param cameraMode the new camera mode - can be defined as such
     *                   SettingsDefinitions.CameraMode cameraMode = SettingsDefinitions.CameraMode.RECORD_VIDEO;
     */
    public void switchCameraMode(final SettingsDefinitions.CameraMode cameraMode, int index) {
        if (cameras == null) {
            return;
        } else if (cameras.size() > 0) {
            cameras.get(cameras.size() > 1 ? index : 0).setMode(cameraMode, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                }
            });
        }
    }

    /**
     * Get camera's filed of view based on camera's name
     *
     * @param a
     */
    public void findFOV(Activity a) {
        try {
            String namecamera = cameras.get(0).getDisplayName();
            Log.d(TAG, "CAMERA NAME: " + namecamera + " " + dji.sdk.camera.Camera.DisplayNameZenmuseH20T);

            String array[] = a.getResources().getStringArray(R.array.cam_mode);
            final String fov1[] = a.getResources().getStringArray(R.array.cam_fov);
            float[] fovh = {0, 0};

            if (namecamera.equalsIgnoreCase(dji.sdk.camera.Camera.DisplayNameZenmuseH20T) || namecamera.equalsIgnoreCase(dji.sdk.camera.Camera.DisplayNameZenmuseH20)) {
                fovh[0] = 40.6f;
            } else
                for (int i = 0; i < array.length; i++)
                    if (array[i].compareTo(namecamera) == 0) {
                        String split[] = fov1[i].split("x");
                        fovh[0] = Float.parseFloat(split[0]);
                        if (split.length > 1) {
                            fovh[1] = Float.parseFloat(split[1]);
                        }
                        break;
                    }
            Log.d(TAG, "fov: " + fovh[0] + " " + fovh[1]);
            if (fovh[1] > 0) {
                fovhor = fovh[1];

                fov = fovh[1] * ((100 - overlap) / 100.0f);
                Log.d(TAG, "fov overlap: " + fov);

            } else if (fovh[0] > 0) {
                //fov = 74;
                float aspectratio = aspect;

                fov = ((fovh[0] * 1.0f) / (aspectratio * 1.0f));
                fovhor = fov;
                fov = fov * ((100 - overlap) / 100.0f);
                Log.d(TAG, "fov overlap: " + fov);

            } else {
                //  show_notification("Field of view camera set to 74. You can change  from navigation menu", MainAct);
            }

        } catch (IllegalArgumentException e) {
            Log.d(TAG, "exception: " + e.toString());

            fovhor = fov;
            fov = fovhor * ((100 - overlap) / 100.0f);

        } catch (IllegalStateException e) {
            Log.d(TAG, "exception: " + e.toString());

            fovhor = fov;
            fov = fovhor * ((100 - overlap) / 100.0f);

        } catch (NullPointerException e) {
            Log.d(TAG, "exception: " + e.toString());

            fovhor = fov;
            fov = fovhor * ((100 - overlap) / 100.0f);

            Log.d(TAG, "fovhor: " + fovhor);
        }
    }

    /**
     * Get camera's aspect ratio
     *
     * @return
     */
    public float aspectratio() {
        try {

            dji.sdk.camera.Camera camera = cameras.get(0);
            Aircraft aircraft = (Aircraft) Helper.getProductInstance();
            if (camera.isMultiLensCameraSupported()) {
                Lens lensCamera = getLenSpecificLen(SettingsDefinitions.LensType.INFRARED_THERMAL, aircraft);

                if (camera.getDisplayName().equals(dji.sdk.camera.Camera.DisplayNameZenmuseH20T))
                    lensCamera = getLenSpecificLen(SettingsDefinitions.LensType.INFRARED_THERMAL, aircraft);
                else if (camera.getDisplayName().equals(dji.sdk.camera.Camera.DisplayNameZenmuseH20)) {
                    lensCamera = getLenSpecificLen(SettingsDefinitions.LensType.WIDE, aircraft);

                }

                lensCamera.getPhotoAspectRatio(new CommonCallbacks.CompletionCallbackWith<SettingsDefinitions.PhotoAspectRatio>() {
                    @Override
                    public void onSuccess(SettingsDefinitions.PhotoAspectRatio photoAspectRatio) {

                        if (photoAspectRatio.equals(SettingsDefinitions.PhotoAspectRatio.RATIO_4_3)) {
                            aspect = 4.0f / 3.0f;
                        } else
                            aspect = 16.0f / 9.0f;


                    }

                    @Override
                    public void onFailure(DJIError djiError) {

                        aspect = 16.0f / 9.0f;

                    }
                });
            } else {

                camera.getPhotoAspectRatio(new CommonCallbacks.CompletionCallbackWith<SettingsDefinitions.PhotoAspectRatio>() {
                    @Override
                    public void onSuccess(SettingsDefinitions.PhotoAspectRatio photoAspectRatio) {

                        if (photoAspectRatio.equals(SettingsDefinitions.PhotoAspectRatio.RATIO_4_3)) {
                            aspect = 4.0f / 3.0f;
                        } else
                            aspect = 16.0f / 9.0f;
                        Log.d(TAG, "aspect ratio: " + aspect);

                        findFOV(getActivity());


                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        Log.d(TAG, "aspect ratio: " + aspect);

                        aspect = 16.0f / 9.0f;
                        findFOV(getActivity());

                    }
                });

            }

        } catch (NullPointerException e) {

        }

        return 16.0f / 9.0f;
    }

    public Lens getLenSpecificLen(SettingsDefinitions.LensType lensType, Aircraft product) {
        Lens len = null;
        try {
            final List<Lens> lens = product.getCameras().get(0).getLenses();

            for (Lens l : lens) {
                Log.d("CameraDrone", "len get type: " + l.getType().toString());

                if (l.getType().equals(lensType)) {
                    len = l;
                    Log.d("CameraDrone", "find zoom lens: ");
                    break;

                }

            }
        } catch (NullPointerException e) {
            e.printStackTrace();

        }
        Log.d("CameraDrone", "getLenSpecificLen: " + (len != null));
        return len;
    }
    public  void setCameraVideoStreamSource(CameraVideoStreamSource cameraVideoStreamSource, MenuItem menuItem) {

        try {
            cameras.get(0).setCameraVideoStreamSource(cameraVideoStreamSource, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        act.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                if (menuItem != null)
                                    menuItem.setTitle(cameraVideoStreamSource.name()+ " Stream");
                            }
                        });
                    }
                }
            });
        } catch (NullPointerException e) {

        }
    }

}
