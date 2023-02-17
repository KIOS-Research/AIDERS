package com.kios.rosDJI.Helpers;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.widget.TextView;
import android.widget.Toast;

import com.kios.rosDJI.ui.MainActivity;

import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;

import dji.common.product.Model;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.sdkmanager.DJISDKManager;

public class Helper {
    private static BaseProduct mProduct;

    public Helper() {

    }

    /**
     * Shows message on current activity.
     *
     * @param activity The activity the user want to show toast.
     * @param msg      The String that the user want to put in the message.
     */
    public static void showToast(final Activity activity, final String msg) {
        activity.runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(activity, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    public static void showText(final Activity activity, final TextView tv, final String msg) {
        if (tv == null) {
            showToast(activity, "The current textView is null. ");
            return;
        }
        activity.runOnUiThread(new Runnable() {
            public void run() {
                tv.setText(msg);
            }
        });
    }

    /**
     * Transfers to list from enum array.
     *
     * @param o An object array
     * @return An ArrayList object of String for enum.
     */
    public ArrayList<String> makeList(Object[] o) {
        ArrayList<String> list = new ArrayList<String>();
        for (int i = 0; i < o.length; i++) {
            list.add(o[i].toString());
        }
        return list;
    }

    public ArrayList<String> makeList(int[] o) {
        ArrayList<String> list = new ArrayList<String>();
        for (int i = 0; i < o.length; i++) {
            list.add(Integer.valueOf(o[i]).toString());
        }
        return list;
    }

    public ArrayList<String> makeList(List o) {
        ArrayList<String> list = new ArrayList<String>();
        Iterator iterator = o.iterator();
        while (iterator.hasNext()) {
            list.add(iterator.next().toString());
        }
        return list;
    }

    public static String getString(byte[] bytes) {
        if (null == bytes) {
            return "";
        }
        // 去除NULL字符
        byte zero = 0x00;
        byte no = (byte) 0xFF;
        for (int i = 0; i < bytes.length; i++) {
            if (bytes[i] == zero || bytes[i] == no) {
                bytes = readBytes(bytes, 0, i);
                break;
            }
        }
        return getString(bytes, "GBK");
    }

    private static String getString(byte[] bytes, String charsetName) {
        return new String(bytes, Charset.forName(charsetName));
    }

    public static byte[] readBytes(byte[] source, int from, int length) {
        byte[] result = new byte[length];
        System.arraycopy(source, from, result, 0, length);
        return result;
    }

    public static byte[] getBytes(String data) {
        return getBytes(data, "GBK");
    }


    private static byte[] getBytes(String data, String charsetName) {
        Charset charset = Charset.forName(charsetName);
        return data.getBytes(charset);
    }


    public static String getStringUTF8(byte[] bytes, int start, int length) {
        if (null == bytes || bytes.length == 0) {
            return "";
        }
        // 去除NULL字符
        byte zero = 0x00;
        for (int i = start; i < length && i < bytes.length; i++) {
            if (bytes[i] == zero) {
                length = i - start;
                break;
            }
        }
        return getString(bytes, start, length, "UTF-8");
    }

    private static String getString(byte[] bytes, int start, int length, String charsetName) {
        return new String(bytes, start, length, Charset.forName(charsetName));
    }

    public static String byte2hex(byte[] buffer) {
        String h = "";
        if (null == buffer) {
            return h;
        }
        for (int i = 0; i < buffer.length; i++) {
            String temp = Integer.toHexString(buffer[i] & 0xFF);
            if (temp.length() == 1) {
                temp = "0" + temp;
            }
            h = h + " " + temp;
        }
        return h;
    }

    public static String timeStamp2Date(String format) {

        if (format == null || format.isEmpty()) {
            format = "yyyy-MM-dd-HH-mm-ss";
        }
        SimpleDateFormat sdf = new SimpleDateFormat(format);
        long time = System.currentTimeMillis();
        return sdf.format(new Date(time));
    }

    private   boolean answer=false;

    public  boolean showDialog(Activity activity, String message, String title,boolean waitAnswer) {
        final Handler handler = new Handler() {
            @Override
            public void handleMessage(Message mesg) {
                throw new RuntimeException();
            }
        };
        AlertDialog.Builder alertDialog = new AlertDialog.Builder(activity);
        alertDialog.setTitle(title);
        alertDialog.setMessage(message);
        AlertDialog accountDialog = alertDialog.create();

        if (waitAnswer){
            alertDialog.setNegativeButton("No", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    answer=false;
                  //  dialog.dismiss();
                   dialog.cancel();
                    handler.sendMessage(handler.obtainMessage());

                }
            });
        }
        alertDialog.setPositiveButton(waitAnswer?"Yes":"Ok", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                answer=true;

              //  dialog.dismiss();
                dialog.cancel();
                handler.sendMessage(handler.obtainMessage());

            }
        });


        alertDialog.setCancelable(true);
        alertDialog.show();

        // loop till a runtime exception is triggered.
        try { Looper.loop(); }
        catch(RuntimeException e2) {
            MainActivity.getRemoteLogger().log(3,MainActivity.getStackTrace(e2));
            e2.printStackTrace();
        }

        MainActivity.getRemoteLogger().log(6,"showDialog: message: "+message+"\n answer: "+answer);

        return answer;
    }

    public static boolean isMultiStreamPlatform() {
        if (DJISDKManager.getInstance() == null||DJISDKManager.getInstance().getProduct()==null) {
            return false;
        }
        Model model = DJISDKManager.getInstance().getProduct().getModel();
        return model != null && (model == Model.INSPIRE_2
                || model == Model.MATRICE_200
                || model == Model.MATRICE_210
                || model == Model.MATRICE_210_RTK
                || model == Model.MATRICE_600
                || model == Model.MATRICE_600_PRO
                || model == Model.A3
                || model == Model.N3);
    }

    public static boolean isM300Product() {
        try {
            if (getProductInstance() != null && getProductInstance().getModel().equals(Model.MATRICE_300_RTK))


                return true;
            else
                return false;
        } catch (NullPointerException e) {

            return false;
        }

    }
    /**
     * This function is used to get the instance of DJIBaseProduct.
     * If no product is connected, it returns null.
     */
    public static synchronized BaseProduct getProductInstance() {
        if (null == mProduct) {
            mProduct = DJISDKManager.getInstance().getProduct();
        }
        return mProduct;
    }

    /**
     * This function is used to get the instance of DJIBaseProduct.
     * If no product is connected, it returns null.
     */
    public static synchronized Camera getCameraInstance() {
        Camera camera=null;
        try {
            if (null == camera) {
                camera = DJISDKManager.getInstance().getProduct().getCamera();
            }
        }
        catch (NullPointerException e){

        }
        return camera;
    }
}
