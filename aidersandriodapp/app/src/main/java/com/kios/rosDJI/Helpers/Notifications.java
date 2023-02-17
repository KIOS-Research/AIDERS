package com.kios.rosDJI.Helpers;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;

public class Notifications {

    static AlertDialog ad;

    /**
     * This function show  a  notification to wait
     *
     * @param text Text to show
     * @param c
     */
    public static void show_notification(String text, Activity c,boolean cancelable) {

        c.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(c);
                builder.setCancelable(cancelable);
                builder.setMessage(text);

                ad = builder.show();
            }
        });



    }

    /**HIde notification
     * @param c
     */
    public static void hide_notification(Context c) {
        if (ad != null) {
            ad.dismiss();
        }
        ad = null;

    }
}
