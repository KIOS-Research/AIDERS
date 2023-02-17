package com.dji.download;

import android.app.Activity;
import android.content.Context;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;
import com.dji.cameras.CameraDrone;
import com.dji.waypoint.Mission;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.Helpers.SharedPreferences;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.media.MediaFile;
import dji.sdk.media.MediaManager;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

/**
 * This class helps to download photos or videos from drone's sd card.It can be used mainly after mission finished and download any photo tha it contains
 * This mode is supported by mavic 2 enterprise, zoom etc
 */
public class MediaDownload extends AsyncTask<Integer, Void, Integer> {

    private static MediaManager.FileListState currentFileListState = MediaManager.FileListState.UNKNOWN;
    int success;
    protected static int photoermain;
    static Activity a;
    static List<MediaFile> mediaFileList;
    private static boolean answer;
    protected static CameraDrone cameraDrone;
    protected static int num_media = 0;

    protected static int remaing = 0;
    protected static SendPhotosToROS sendPhotosToROS;
    static    SettingsDefinitions.StorageLocation storageLocation;
    String TAG = "MediaDownload";

    private MediaManager.FileListStateListener updateFileListStateListener = new MediaManager.FileListStateListener() {
        @Override
        public void onFileListStateChange(MediaManager.FileListState state) {

            currentFileListState = state;
            Log.d(TAG, "file state manager listener: " + currentFileListState);
        }
    };

    static File MISSION__DIRECTORY;
    protected static ConnectDroneActivity connectDroneActivity;

    /**
     * @param a
     */
    public MediaDownload(Activity a, ConnectDroneActivity connectDroneActivity) {

        this.connectDroneActivity = connectDroneActivity;

        this.a = a;
        cameraDrone = new CameraDrone();
        cameraDrone.cameraCallBack();
    }


    public MediaDownload() {

    }


    protected static boolean flag = false;
    protected static ProgressDialog progressDialog;

    public static void setFlag(boolean flag) {
        MediaDownload.flag = flag;

    }

    public BaseProduct getBaseProduct() {
        return DJISDKManager.getInstance().getProduct();
    }


    /**
     * Start to download photos from the current mission. It uses a loading bar to show progress while photos are being downloaded
     */
    public void MediaDownloadPhotos() {
        progressDialog = new ProgressDialog(a);
        String folder =  SharedPreferences.getStringSharedPreference(a,"time_start_mission") ;
        Log.d(TAG, "getStartTime: " + folder);
        sendPhotosToROS = new SendPhotosToROS(a, folder,  "172.20.81.82:8000");

        File dir2 = new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/");
        initDirectory(dir2.getPath());
        dir2 = new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder);
        initDirectory(dir2.getPath());
        MISSION__DIRECTORY = dir2;

        remaing = 0;


        num_media = Mission.getNum_media() == 0 ? SharedPreferences.getIntSharedPreference(a, "photos_mission") : Mission.getNum_media();

        Log.d("numMedia", num_media + "");

        if (num_media > 0) {


            Aircraft product = (Aircraft) Helper.getProductInstance();

            final Camera camera = product.getCameras().get(0);

            if (camera.isMediaDownloadModeSupported()) {
                progressDialog.initDownloadProgressDialog("Download 1 of " + num_media, 0);
                progressDialog.setText("Download 1 of " + num_media);
                progressDialog.showDownloadProgressDialog();
                storageLocation = null;

                if (getBaseProduct().getModel().equals(Model.MATRICE_300_RTK)){
                    storageLocation = SettingsDefinitions.StorageLocation.SDCARD;
                }else{
                    storageLocation = SettingsDefinitions.StorageLocation.INTERNAL_STORAGE;
                }

                Log.d(TAG, "storageLocation: " + storageLocation);

                if (storageLocation == null)
                    return;

                if (getBaseProduct().getModel().equals(Model.MATRICE_300_RTK)){
                    camera.enterPlayback(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if(djiError != null){
                                Log.d(TAG, "ENTER PLAYBACK: " + djiError.getDescription());
                            }
                        }
                    });
                }else{
                    camera.setMode(SettingsDefinitions.CameraMode.MEDIA_DOWNLOAD, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                Log.d(TAG, "error set mode: " + djiError.toString());
                            }

                        }
                    });
                }

                final MediaManager mMediaManager = camera.getMediaManager();

                mMediaManager.addUpdateFileListStateListener(updateFileListStateListener);

                if (currentFileListState == MediaManager.FileListState.SYNCING || currentFileListState == MediaManager.FileListState.DELETING) {

                } else {

                    camera.getMediaManager().refreshFileListOfStorageLocation(storageLocation, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            //Log.d(TAG, "mediafilelist1: " + djiError.getDescription());
                            if (null == djiError) {
                                //Log.d(TAG, "mediafilelist2: " + djiError.getDescription());
                                if (storageLocation.equals(SettingsDefinitions.StorageLocation.SDCARD))

                                    while (Helper.getCameraInstance() != null && camera.getMediaManager().getSDCardFileListState().compareTo(MediaManager.FileListState.UP_TO_DATE) != 0
                                    ) {
                                        Log.d(TAG, "not up to date");

                                    }
                                if (storageLocation.equals(SettingsDefinitions.StorageLocation.INTERNAL_STORAGE))
                                    while (Helper.getCameraInstance() != null && camera.getMediaManager().getInternalStorageFileListState().compareTo(MediaManager.FileListState.UP_TO_DATE) != 0
                                    ) {
                                        Log.d(TAG, "not up to date");

                                    }

                                photoermain = num_media;
                                if (storageLocation.equals(SettingsDefinitions.StorageLocation.SDCARD)) {
                                    mediaFileList = mMediaManager.getSDCardFileListSnapshot();
                                }else
                                    mediaFileList = mMediaManager.getInternalStorageFileListSnapshot();

                                Collections.sort(mediaFileList, new Comparator<MediaFile>() {
                                    @Override
                                    public int compare(MediaFile lhs, MediaFile rhs) {
                                        if (lhs.getTimeCreated() < rhs.getTimeCreated()) {
                                            return 1;
                                        } else if (lhs.getTimeCreated() > rhs.getTimeCreated()) {
                                            return -1;
                                        }
                                        return 0;
                                    }
                                });


                                remaing++;

                                new MediaDownload().execute(1);

                            } else {
                                new CameraDrone().switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0
                                );

                                setResultToToast("Get Media File List Failed:" + djiError.getDescription(), a);
                            }
                        }

                    });

                }

            }
        } else {

            new CameraDrone().switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0
            );
            setResultToToast("No Images To Download", a);

        }

    }


    @Override
    protected void onPreExecute() {

        super.onPreExecute();

    }

    protected String fileName;

    /**
     * Asychronous method which is called while user is flying and the leaves his finger on the camera mview
     * This is ana Asynchronous method.
     *
     * @param move Type of monevement (up,right,down, left)
     * @return
     */
    @Override
    protected Integer doInBackground(Integer... move) {
        Log.d(TAG, "download: " + photoermain);

        if (photoermain > 0 && photoermain <= mediaFileList.size()) {
            Log.d(TAG, "MISSION__DIRECTORY: " + MISSION__DIRECTORY);

            DownloadListener downloadListener = new DownloadListener(a, this);
            fileName = mediaFileList.get(photoermain - 1).getFileName();
            mediaFileList.get(photoermain - 1).fetchFileData(MISSION__DIRECTORY, null, downloadListener

            );

            Log.d("CONTROLLER PATH", MISSION__DIRECTORY.getPath());
        }else{

        }

        return success;
    }


    /**
     * This method is triggered when bagrounf methos is done. It checks if user didn't move his finger from the camera view
     *
     * @param result Type of movement (up,down etc)
     */
    protected void onPostExecute(Integer result) {


    }


    public static void setResultToToast(final String string, final Context c) {
        ((Activity) c).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(c, string, Toast.LENGTH_SHORT).show();
            }
        });
    }


    //get current datetime
    public static String getCurrentDateTime_underscore() {
        Calendar c = Calendar.getInstance();
        SimpleDateFormat df = new SimpleDateFormat("yyyy_MM_dd HH.mm.ss.SSS", Locale.getDefault());
        return df.format(c.getTime());
    }

    /**
     * Create a new directory in android device if not exist
     *
     * @param mkdir File directory path
     * @return int if the directory exists, this function returns the number of files
     */
    public int initDirectory(String mkdir) {
        //check exists,if not,create
        File sourceDirectory = new File(mkdir);
        if (!sourceDirectory.exists()) {
            boolean response = sourceDirectory.mkdirs();
            Log.d(TAG, "initDirectory: " + response);
        }
        if (sourceDirectory.listFiles() != null)
            return sourceDirectory.listFiles().length;

        return 0;

    }
}






