package com.kios.rosDJI.Helpers;

import android.app.Activity;

import static android.content.Context.MODE_PRIVATE;

public class SharedPreferences {

    public static void setBooleanSharedPreference(Activity activity, String key, boolean value){
        android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
        android.content.SharedPreferences.Editor myEdit = sharedPreferences.edit();
        myEdit.putBoolean(key,value);
        myEdit.commit();
    }
    public static boolean getBooleanSharedPreference(Activity activity,String key){
        android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
        return sharedPreferences.getBoolean(key,true);

    }

    public static void setIntSharedPreference(Activity activity, String key, int value){
        try {
            android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
            android.content.SharedPreferences.Editor myEdit = sharedPreferences.edit();
            myEdit.putInt(key, value);
            myEdit.commit();
        }catch (NullPointerException e){

        }
    }
    public static int getIntSharedPreference(Activity activity,String key){
        android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
        return sharedPreferences.getInt(key,0);

    }
    public static void setStringSharedPreference(Activity activity, String key, String value){
        android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
        android.content.SharedPreferences.Editor myEdit = sharedPreferences.edit();
        myEdit.putString(key,value);
        myEdit.commit();
    }
    public static String getStringSharedPreference(Activity activity,String key){
        android.content.SharedPreferences sharedPreferences = activity.getSharedPreferences("ROS", MODE_PRIVATE);
        return sharedPreferences.getString(key,null);

    }
}
