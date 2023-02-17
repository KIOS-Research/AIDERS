
package com.dji.download;

import android.app.Activity;
import android.os.Build;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import androidx.annotation.RequiresApi;
import com.jilk.ros.message.BuildMap;
import com.kios.rosDJI.Helpers.Helper;
import com.kios.rosDJI.ui.ConnectDroneActivity;
import com.kios.rosDJI.ui.MainActivity;
import com.loopj.android.http.AsyncHttpClient;
import com.loopj.android.http.AsyncHttpResponseHandler;
import com.loopj.android.http.RequestParams;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import cz.msebera.android.httpclient.Header;
import cz.msebera.android.httpclient.HttpEntity;
import cz.msebera.android.httpclient.HttpResponse;
import cz.msebera.android.httpclient.client.HttpClient;
import cz.msebera.android.httpclient.entity.mime.MultipartEntityBuilder;
import cz.msebera.android.httpclient.entity.mime.content.FileBody;
import cz.msebera.android.httpclient.impl.client.HttpClientBuilder;
import statusbar.NotificationHelper;


public class SendPhotosToROS {
    protected Queue<String> availablePhotos;
    private Handler handler;
    Activity activity;
    String url = "";
    private String folder = "";

    public SendPhotosToROS(Activity activity, String folder, String url) {
        handler = new Handler();
        this.activity = activity;
        availablePhotos = new LinkedList<>();

        this.folder = folder;
        this.url = url;
        //sendPhotos();
    }
    AsyncHttpClient client;
    public SendPhotosToROS(Activity activity, String url) {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                handler = new Handler();

            }
        });
        client = new AsyncHttpClient(8000);
        client.setConnectTimeout(5000);

        this.activity = activity;
        this.url = url;

        //sendPhotos();
    }

    protected void sendPhotosSynchronous() {

        runInBackground(
                new Runnable() {
                    @Override
                    public void run() {
                        final File files = new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder + "/");


                        for (File file : files.listFiles()) {
                            Log.d("SendPhotosToROS", "image_name" + file.getName());
                            Log.d("SendPhotosToROS", "image_path" + file.getPath());
                            ArrayList nameValuePairs = new ArrayList();

                            HttpEntity entity = MultipartEntityBuilder.create()
                                    .addPart("file", new FileBody(file))
                                    .addTextBody("image_name", file.getName())
                                    .addTextBody("date_time", folder)
                                    .build();

                            try {
                                HttpClient httpclient = HttpClientBuilder.create().build();
                                ;
                                cz.msebera.android.httpclient.client.methods.HttpPost httppost = new cz.msebera.android.httpclient.client.methods.HttpPost("http://" + url);
                                httppost.setEntity(entity);
                                HttpResponse response = httpclient.execute(httppost);

                                Helper.showToast(activity, "Response " + response.getStatusLine().getStatusCode());
                                Log.d("SendPhotosToROS", "Response " + response.getStatusLine().getStatusCode());

                            } catch (Exception e) {
                                Helper.showToast(activity, "Response " + e.getMessage());

                                Log.d("SendPhotosToROS", "error");

                                e.printStackTrace();
                            }

                        }
                    }

                });
    }


    private boolean posted = false;


    public void sendPhotosMultiples() {

        Log.d("SendPhotosToROS", "date_time: " + folder);
        Log.d("SendPhotosToROS", "drone_name: " + MainActivity.droneName);
        final File files = new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder + "/");
        runInBackground(new Runnable() {
            @Override
            public void run() {
                AsyncHttpClient client = new AsyncHttpClient(5000);
                NotificationHelper mNotificationHelper = new NotificationHelper(activity);
                mNotificationHelper.createNotification();
                int total = files.listFiles().length;
                Log.d("MATRIX", "Number of files: " + total);
                int current = 1;
                for (File file : files.listFiles()) {
                    Log.d("SendPhotosToROS", "image_name" + file.getName());
                    Log.d("SendPhotosToROS", "image_path" + file.getPath());

                    RequestParams params = new RequestParams();
                    try {
                        params.put("image_file", file);
                        params.put("image_name", file.getName());
                        params.put("date_time", folder);
                        params.put("drone_name", MainActivity.droneName);

                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                        Log.d("SendPhotosToROS", "File not found!!!");
                    }
                    final String filename = file.getName();
                    Log.d("SendPhotosToROS", "start post:[" + url.replaceAll(" ", "") + "]");

                    posted = false;
                    final Handler handler = new Handler() {
                        @Override
                        public void handleMessage(Message mesg) {
                            throw new RuntimeException();
                        }
                    };
                    mNotificationHelper.progressUpdate("Sending " + current + " of " + total);

                    Log.d("MATRIX", "http://" + url + "/grid_missions/image");
                    client.post("http://" + url + "/grid_missions/image", params, new AsyncHttpResponseHandler() {

                        @Override
                        public void onStart() {

                            Log.d("SendPhotosToROS", "onStart post");
                            // called before request is started
                        }

                        @Override
                        public void onSuccess(int statusCode, Header[] headers, byte[] response) {
                            posted = true;

                            handler.sendMessage(handler.obtainMessage());
                            Log.d("SendPhotosToROS", "onSuccess post");
                            // called when response HTTP status is "200 OK"
                        }

                        @Override
                        public void onFailure(int statusCode, Header[] headers, byte[] errorResponse, Throwable e) {

                            posted = true;
                            handler.sendMessage(handler.obtainMessage());
                            Log.d("SendPhotosToROS", "onFailure post: " + statusCode);
                            // called when response HTTP status is "4XX" (eg. 401, 403, 404)
                        }

                        @Override
                        public void onRetry(int retryNo) {
                            Log.d("SendPhotosToROS", "onRetry");
                            // called when request is retried
                        }
                    });

                    try {
                        Looper.loop();
                    } catch (RuntimeException e2) {
                        MainActivity.getRemoteLogger().log(3, MainActivity.getStackTrace(e2));
                        e2.printStackTrace();
                    }
                    current++;
                    Log.d("SendPhotosToROS", "end post");
                }
            }

        });
    }


    public void sendPhotosFromMission() {

        url = MainActivity.masterIP + ":8000";

        Log.d("SendPhotosToROS", "date_time: " + folder);
        Log.d("SendPhotosToROS", "drone_name: " + MainActivity.droneName);
        final File files = new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder + "/");
        runInBackground(new Runnable() {
            @Override
            public void run() {
                AsyncHttpClient client = new AsyncHttpClient(8000);
                NotificationHelper mNotificationHelper = new NotificationHelper(activity);
                mNotificationHelper.createNotification();
                int total = files.listFiles().length;
                Log.d("MATRIX", "Number of files: " + total);
                int current = 1;
                for (File file : files.listFiles()) {
                    Log.d("SendPhotosToROS", "image_name" + file.getName());
                    Log.d("SendPhotosToROS", "image_path" + file.getPath());

                    RequestParams params = new RequestParams();
                    try {
                        params.put("image_file", file);
                        params.put("image_name", file.getName());

                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                        Log.d("SendPhotosToROS", "File not found!!!");
                    }
                    final String filename = file.getName();
                    posted = false;
                    final Handler handler = new Handler() {
                        @Override
                        public void handleMessage(Message mesg) {
                            throw new RuntimeException();
                        }
                    };
                    mNotificationHelper.progressUpdate("Sending " + current + " of " + total);

                    Log.d("MATRIX", "http://" + url + "/postDataImg/");
                    client.post("http://" + url + "/postDataImg/", params, new AsyncHttpResponseHandler() {

                        @Override
                        public void onStart() {

                            Log.d("SendPhotosToROS", "onStart post");
                            // called before request is started
                        }

                        @Override
                        public void onSuccess(int statusCode, Header[] headers, byte[] response) {
                            posted = true;

                            handler.sendMessage(handler.obtainMessage());
                            Log.d("SendPhotosToROS", "onSuccess post");
                            // called when response HTTP status is "200 OK"
                        }

                        @Override
                        public void onFailure(int statusCode, Header[] headers, byte[] errorResponse, Throwable e) {

                            posted = true;
                            handler.sendMessage(handler.obtainMessage());
                            Log.d("SendPhotosToROS", "onFailure post: " + statusCode);
                            // called when response HTTP status is "4XX" (eg. 401, 403, 404)
                        }

                        @Override
                        public void onRetry(int retryNo) {
                            Log.d("SendPhotosToROS", "onRetry");
                            // called when request is retried
                        }
                    });

                    try {
                        Looper.loop();
                    } catch (RuntimeException e2) {
                        MainActivity.getRemoteLogger().log(3, MainActivity.getStackTrace(e2));
                        e2.printStackTrace();
                    }
                    current++;
                    Log.d("SendPhotosToROS", "end post");
                }
            }

        });
    }



    public void sendPhotos(String filename) {
        Log.d("SendPhotosToROS", "filepath: " + Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder + "/" + filename);
        Log.d("SendPhotosToROS", "image_name" + filename);
        Log.d("SendPhotosToROS", "date_time: " + folder);
        Log.d("SendPhotosToROS", "drone_name: " + MainActivity.droneName);

        RequestParams params = new RequestParams();
        try {
            params.put("image_file", new File(Environment.getExternalStorageDirectory() + "/RosDJI/missions/" + folder + "/" + filename));
            params.put("image_name", filename);
            params.put("date_time", folder);
            params.put("drone_name", MainActivity.droneName);

        } catch (FileNotFoundException e) {
            e.printStackTrace();
            Log.d("SendPhotosToROS", "File not found!!!");
        }
        AsyncHttpClient client = new AsyncHttpClient();
        Log.d("SendPhotosToROS", "start post:[" + url.replaceAll(" ", "") + "]");

        client.post("http://" + url + "/grid_missions/image", params, new AsyncHttpResponseHandler() {

            @Override
            public void onStart() {
                Helper.showToast(activity, "sending " + filename);

                Log.d("SendPhotosToROS", "onStart post");

                // called before request is started
            }

            @Override
            public void onSuccess(int statusCode, Header[] headers, byte[] response) {
                Helper.showToast(activity, "sent " + filename + " successfully");

                Log.d("SendPhotosToROS", "onSuccess post");


                // called when response HTTP status is "200 OK"
            }

            @Override
            public void onFailure(int statusCode, Header[] headers, byte[] errorResponse, Throwable e) {
                Helper.showToast(activity, "failed to send " + filename);

                Log.d("SendPhotosToROS", "onFailure post: " + statusCode);

                // called when response HTTP status is "4XX" (eg. 401, 403, 404)
            }

            @Override
            public void onRetry(int retryNo) {
                Log.d("SendPhotosToROS", "onRetry");

                // called when request is retried
            }
        });
        Log.d("SendPhotosToROS", "end post");

    }

    private synchronized void runInBackground(final Runnable r) {
        if (handler != null) {
            handler.post(r);
        }
    }

    public void sendPhotoBuildMap(String imgPath, BuildMap param) {

        Log.d("SendPhotosToROS", "drone_name: " + MainActivity.droneName);

        runInBackground(new Runnable() {
            @RequiresApi(api = Build.VERSION_CODES.O)
            @Override
            public void run() {
                try {

                    RequestParams params = new RequestParams();
                    final File file = new File(imgPath);

                    Log.d("PATH", imgPath);

                    try {
                        params.put("image_file", new File(imgPath));
                        params.put("image_name", file.getName());
                        android.text.format.DateFormat df = new android.text.format.DateFormat();
                        params.put("drone_name", MainActivity.droneName);
                        params.put("bearing", "" + param.heading);
                        params.put("lat", "" + param.latitude);
                        params.put("lon", "" + param.longitude);
                        params.put("alt", "" + param.altitude);
                        Log.d("SendPhotosToROS", "sendPhotoBuildMap : image_name " + file.getName());
                        Log.d("SendPhotosToROS", "sendPhotoBuildMap : bearing " + param.heading);
                        Log.d("SendPhotosToROS", "sendPhotoBuildMap : lat " + param.latitude);
                        Log.d("SendPhotosToROS", "sendPhotoBuildMap : lon " + param.longitude);
                        Log.d("SendPhotosToROS", "sendPhotoBuildMap : alt " + param.altitude);

                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                        Log.d("SendPhotosToROS", "File not found!!!");
                    } catch (IOException e) {
                        e.printStackTrace();
                        Log.d("SendPhotosToROS", "IOException");
                    }
                    Log.d("SendPhotosToROS", "start post:[" + "http://" + url + "/postBuildMapImg/" + "]");
                    client.setLoggingEnabled(true);
                    client.post("http://" + url + "/postBuildMapImg/", params, new AsyncHttpResponseHandler() {

                        @Override
                        public void onStart() {

                            Log.d("SendPhotosToROS", "sendPhotoBuildMap : onStart post");

                        }

                        @Override
                        public void onSuccess(int statusCode, Header[] headers, byte[] response) {

                            Log.d("SendPhotosToROS", "sendPhotoBuildMap: onSuccess post");
                            ConnectDroneActivity.m.setDisplayDataLeft("sent " + file.getName());

                        }

                        @Override
                        public void onFailure(int statusCode, Header[] headers, byte[] errorResponse, Throwable e) {
                            if (file != null) {
                                boolean deleted = file.delete();
                                Log.d("SendPhotosToROS", "sendPhotoBuildMap : file deleted : " + deleted);
                                e.printStackTrace();

                            }

                            Log.d("SendPhotosToROS", "sendPhotoBuildMap : onFailure post: " + statusCode + " , " + e.getMessage()
                            );

                            posted = true;

                            ConnectDroneActivity.m.setDisplayDataLeft("Build Map Successfully Stopped");
                            // called when response HTTP status is "4XX" (eg. 401, 403, 404)
                        }

                        @Override
                        public void onRetry(int retryNo) {
                            Log.d("SendPhotosToROS", "sendPhotoBuildMap: onRetry");
                            // called when request is retried
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                    Log.e("SendPhotosToROS", "exception photos build map");

                }

            }

        });

        Log.d("SendPhotosToROS", "end post");
    }

}

