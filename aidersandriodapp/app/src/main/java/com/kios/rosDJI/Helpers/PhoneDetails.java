package com.kios.rosDJI.Helpers;


import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.provider.Settings;
import android.telephony.TelephonyManager;
import android.text.TextUtils;
import android.util.Log;

import androidx.core.app.ActivityCompat;

/**Show android device details lieke IMEI
 *
 */
public class PhoneDetails {
    Activity a;


    public  PhoneDetails(Activity a){
        this.a=a;
    }
    public static  int getAndroidVersion() {
        String release = Build.VERSION.RELEASE;
        Log.d("PhoneDetails","android version: "+release);

        int version=Integer.parseInt(release.split("\\.")[0]);

        return version;
    }
    private static String capitalize(String str) {
        if (TextUtils.isEmpty(str)) {
            return str;
        }
        char[] arr = str.toCharArray();
        boolean capitalizeNext = true;

        StringBuilder phrase = new StringBuilder();
        for (char c : arr) {
            if (capitalizeNext && Character.isLetter(c)) {
                phrase.append(Character.toUpperCase(c));
                capitalizeNext = false;
                continue;
            } else if (Character.isWhitespace(c)) {
                capitalizeNext = true;
            }
            phrase.append(c);
        }

        return phrase.toString();

    }

    /**Get imei from android device
     * @return
     */
    public String getIMEI() {
        String imei;
        TelephonyManager mTelephony = (TelephonyManager) a.getSystemService(Context.TELEPHONY_SERVICE);
        if (ActivityCompat.checkSelfPermission(a, Manifest.permission.READ_PHONE_STATE) != PackageManager.PERMISSION_GRANTED) {

            Log.d("PhoneDetails","deny get imei");
            return null;
        }

        if (android.os.Build.VERSION.SDK_INT >= 29) {


            imei = Settings.Secure.getString(
                    a.getContentResolver(),
                    Settings.Secure.ANDROID_ID);
            Log.d("PhoneDetails"," get imei sdk version 10: "+imei);

            return imei;
        }


        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            Log.d("PhoneDetails"," get imei 1");

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                if (mTelephony.getPhoneCount() == 2) {
                    Log.d("PhoneDetails"," get imei 2");

                    imei = mTelephony.getImei(0);

                } else {
                    Log.d("PhoneDetails"," get imei 3");

                    imei = mTelephony.getImei();
                }
            } else {
                Log.d("PhoneDetails"," get imei 4");

                if (mTelephony.getPhoneCount() == 2) {
                    imei = mTelephony.getDeviceId(0);
                } else {
                    imei = mTelephony.getDeviceId();
                }
            }
        } else {
            imei = mTelephony.getDeviceId();
        }
        return imei;
    }


    /**
     * Returns the consumer friendly device name
     */
    public static String getDeviceName() {
        String manufacturer = Build.MANUFACTURER;
        String model = Build.MODEL;
        if (model.startsWith(manufacturer)) {
            Log.d("PhoneDetails",capitalize(model));
            return capitalize(model);
        }
        Log.d("PhoneDetails",capitalize(manufacturer) + " " + model);

        return capitalize(manufacturer) + " " + model;
    }
}
