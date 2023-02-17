package com.dji.download;

import android.app.Activity;
import android.util.Log;
import com.kios.rosDJI.Helpers.Helper;
import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;


public class DownloadListener implements dji.sdk.media.DownloadListener<String> {
    String TAG = "MediaDownload";
    Activity activity;
    MediaDownload mediaDownload;
    protected static SendPhotosToROS sendPhotosToROS;




    public DownloadListener(Activity activity, MediaDownload mediaDownload) {
        this.activity = activity;
        this.mediaDownload = mediaDownload;
    }

    @Override
    public void onRealtimeDataUpdate(byte[] var1, long var2, boolean var4) {

    }

    @Override
    public void onFailure(final DJIError error) {
        mediaDownload.success = 0;
        Log.d(TAG, "FAILED");
        (activity).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mediaDownload.setResultToToast("Download File Failed" + error.getDescription(), activity);
            }
        });
    }

    @Override
    public void onProgress(final long total, final long current) {
        Log.d(TAG, "progress: " + total + " / " + current);
        (activity).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                final int tmpProgress = (int) (1.0 * current / total * 100);
                if ((int) (tmpProgress) >= 100) {}

            }
        });
    }

    @Override
    public void onRateUpdate(long total, long current, long persize) {
        Log.d(TAG, "onRateUpdate: " + total + " / " + current);

        int tmpProgress = (int) (1.0 * current / total * 100);
        if (mediaDownload.connectDroneActivity != null)
            mediaDownload.connectDroneActivity.setDisplayDataLeft2("Downloading media: " + ((mediaDownload.num_media - mediaDownload.photoermain) + 1) + " of " + (mediaDownload.num_media) + "\n (" + tmpProgress + " % )");
        else
            mediaDownload.progressDialog.setProgress(tmpProgress);
    }

    @Override
    public void onStart() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mediaDownload.remaing++;
                mediaDownload.progressDialog.setText("Downloading " + ((mediaDownload.num_media - mediaDownload.photoermain) + 1) + " of " + (mediaDownload.num_media));

            }
        });

    }

    @Override
    public void onSuccess(String filePath) {
        Log.d(TAG, "filename to send to ros: " + mediaDownload.fileName);
        mediaDownload.success = 1;
        mediaDownload.photoermain--;
        if (mediaDownload.photoermain > 0 && mediaDownload.success == 1) {
            new MediaDownload().execute(1);

        } else if (mediaDownload.photoermain == 0 && mediaDownload.success == 1) {
            try {
                Helper.getProductInstance().getCameras().get(0).getMediaManager().exitMediaDownloading();

                mediaDownload.progressDialog.hideDownloadProgressDialog();
                sendPhotosToROS.sendPhotosFromMission();

                mediaDownload.cameraDrone.switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0
                );

            } catch (NullPointerException e) {

            }
            MediaDownload.sendPhotosToROS.sendPhotosFromMission();

        } else {
            Helper.getProductInstance().getCameras().get(0).getMediaManager().exitMediaDownloading();
            mediaDownload.cameraDrone.switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, 0);
            mediaDownload.progressDialog.hideDownloadProgressDialog();
            MediaDownload.sendPhotosToROS.sendPhotosMultiples();

        }
    }

}



