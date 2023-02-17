package com.dji.download;


import android.app.Activity;
import android.content.DialogInterface;
import android.util.Log;
import com.dji.cameras.CameraDrone;
import com.kios.rosDJI.Helpers.Helper;
import dji.common.camera.SettingsDefinitions;
import dji.sdk.base.BaseProduct;


/**
 * Show a progress dialog
 * Created by george on 04/04/2018.
 */

public class ProgressDialog {
    Activity c;
String TAG="ProgressDialog";
    public ProgressDialog(Activity c) {
        this.c = c;
    }

    protected static android.app.ProgressDialog mDownloadDialog;


    /**
     * Set progrees of the progress dialog
     *
     * @param value Value to set of the progrees dialog (0-100)
     */
    public void setProgress(final int value) {
        ((Activity) c).runOnUiThread(new Runnable() {
            public void run() {
                mDownloadDialog.setProgress(value);
            }
        });

    }

    /**
     * Text to show on the window of dialog
     *
     * @param text
     */
    public void setText(final String text) {
        ((Activity) c).runOnUiThread(new Runnable() {
            public void run() {

                mDownloadDialog.setTitle(text);
            }
        });
        Log.d(TAG, "show download");

    }

    //show download progress dialog
    public void showDownloadProgressDialog() {
        Log.d(TAG, "show download");

        if (mDownloadDialog != null && !mDownloadDialog.isShowing()) {
            ((Activity) c).runOnUiThread(new Runnable() {
                public void run() {
                    mDownloadDialog.show();
                    mDownloadDialog.setProgress(0);
                }
            });
        }
    }

    //hide download progress dialog
    public void hideDownloadProgressDialog() {
        if (null != mDownloadDialog && mDownloadDialog.isShowing()) {
            ((Activity) c).runOnUiThread(new Runnable() {
                public void run() {
                    mDownloadDialog.hide();
                }
            });
        }
    }


    ///init download progress dialog
    public void initDownloadProgressDialog(final String s, final int type) {
        Log.d(TAG, "dialog init");

        mDownloadDialog = new android.app.ProgressDialog(c);
        mDownloadDialog.setTitle(s);
        mDownloadDialog.setIcon(android.R.drawable.ic_dialog_info);
       if (type == 0)
            mDownloadDialog.setProgressStyle(android.app.ProgressDialog.STYLE_HORIZONTAL);
        else
            mDownloadDialog.setProgressStyle(android.app.ProgressDialog.STYLE_SPINNER);
        mDownloadDialog.setCancelable(true);
        mDownloadDialog.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(final DialogInterface dialog, int which) {
                c.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {

                        try {
                            dialog.cancel();


                        } catch (Exception e) {

                            e.printStackTrace();
                        }
                        BaseProduct product = Helper.getProductInstance();

                        if (product != null) {
                            try {
                                product.getCamera().getMediaManager().exitMediaDownloading();
                                product.getCamera().getPlaybackManager().unselectAllFiles();
                                product.getCamera().getPlaybackManager().exitMultipleEditMode();
                            } catch (NullPointerException e) {

                            }
                        }
                        new CameraDrone().switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO,0
                        );

                    }
                });
            }
        });

        mDownloadDialog.setCanceledOnTouchOutside(false);
    }
}