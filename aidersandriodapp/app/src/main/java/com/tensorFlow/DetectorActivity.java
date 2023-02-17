/*
 * Copyright 2018 The TensorFlow Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.tensorFlow;

import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.os.Environment;
import android.os.SystemClock;
import android.util.Log;
import android.util.Size;
import android.util.TypedValue;
import android.widget.Toast;

import com.tensorFlow.OverlayView.DrawCallback;
import com.tensorFlow.env.BorderedText;
import com.tensorFlow.env.ImageUtils;
import com.tensorFlow.env.Logger;
import com.tensorFlow.tracking.MultiBoxTracker;
import com.kios.rosDJI.R;
import com.kios.rosDJI.ui.ConnectDroneActivity;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

/**
 * An activity that uses a TensorFlowMultiBoxDetector and ObjectTracker to detect and then track
 * objects.
 */
//extends CameraActivity implements OnImageAvailableListener
public class DetectorActivity  {
    private static final Logger LOGGER = new Logger();

    // Configuration values for the prepackaged SSD model.
    private static final int TF_OD_API_INPUT_SIZE = 300;
    private static final boolean TF_OD_API_IS_QUANTIZED = true;
    private static final String TF_OD_API_MODEL_FILE = "detect.tflite";
    private static final String TF_OD_API_LABELS_FILE = "coco_labels_list.txt";

    // Which detection model to use: by default uses Tensorflow Object Detection API frozen
    // checkpoints.
    private enum DetectorMode {
        TF_OD_API;
    }

    private static final DetectorMode MODE = DetectorMode.TF_OD_API;

    // Minimum detection confidence to track a detection.
    private static final float MINIMUM_CONFIDENCE_TF_OD_API = 0.6f;

    private static final boolean MAINTAIN_ASPECT = false;

    private static final Size DESIRED_PREVIEW_SIZE = new Size(640, 480);

    private static final boolean SAVE_PREVIEW_BITMAP = false;
    private static final float TEXT_SIZE_DIP = 10;

    private Integer sensorOrientation;

    private Classifier detector;

    private long lastProcessingTimeMs;
    private Bitmap rgbFrameBitmap = null;
    private Bitmap croppedBitmap = null;
    private Bitmap cropCopyBitmap = null;

    private boolean computingDetection = false;

    private long timestamp = 0;

    private Matrix frameToCropTransform;
    private Matrix cropToFrameTransform;

    private MultiBoxTracker tracker;
    private BorderedText borderedText;
    private byte[] luminanceCopy;

    private int previewWidth;
    private int previewHeight;

    public static OverlayView trackingOverlay;
    private File dir;
    public void onPreviewSizeChosen() {
        final float textSizePx =
                TypedValue.applyDimension(
                        TypedValue.COMPLEX_UNIT_DIP, TEXT_SIZE_DIP,ConnectDroneActivity.act.getResources().getDisplayMetrics());
        borderedText = new BorderedText(textSizePx);
        borderedText.setTypeface(Typeface.MONOSPACE);

        tracker = new MultiBoxTracker(ConnectDroneActivity.act);

        int cropSize = TF_OD_API_INPUT_SIZE;
        try {
            detector = TFLiteObjectDetectionAPIModel.create(
                    ConnectDroneActivity.act.getAssets(),
                    TF_OD_API_MODEL_FILE,
                    TF_OD_API_LABELS_FILE,
                    TF_OD_API_INPUT_SIZE,
                    TF_OD_API_IS_QUANTIZED);
            cropSize = TF_OD_API_INPUT_SIZE;
        } catch (final IOException e) {
            LOGGER.e("Exception initializing classifier!", e.toString());
            Toast toast =
                    Toast.makeText(
                            ConnectDroneActivity.act.getApplicationContext(), "Classifier could not be initialized", Toast.LENGTH_SHORT);
            toast.show();
            ConnectDroneActivity.act.finish();
        }


        previewWidth = ConnectDroneActivity.videoViewWidth;
        previewHeight = ConnectDroneActivity.videoViewHeight;

    sensorOrientation = 0;
        LOGGER.i("Camera orientation relative to screen canvas: %d", sensorOrientation);

    LOGGER.i("Initializing at size %dx%d", previewWidth, previewHeight);
    rgbFrameBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Config.ARGB_8888);
    croppedBitmap = Bitmap.createBitmap(cropSize, cropSize, Config.ARGB_8888);

        frameToCropTransform =
                ImageUtils.getTransformationMatrix(
                        previewWidth, previewHeight,
                        cropSize, cropSize,
                        sensorOrientation, MAINTAIN_ASPECT);

        cropToFrameTransform = new Matrix();
        frameToCropTransform.invert(cropToFrameTransform);

        trackingOverlay = (OverlayView) ConnectDroneActivity.act.findViewById(R.id.tracking_overlay);
        trackingOverlay.addCallback(
                new DrawCallback() {
                    @Override
                    public void drawCallback(final Canvas canvas) {
                        tracker.draw(canvas);
                    }
                });

        SimpleDateFormat sdf = new SimpleDateFormat("MMddyy");
        String currentDate = sdf.format(new Date());

        dir = new File(Environment.getExternalStorageDirectory()+ "/RosDJI/Detection/" + currentDate + "/");
        try{
            if(dir.mkdirs()) {
                System.out.println("Directory created");
                Log.e("MKDIR_", "CREATED");
            } else {
                System.out.println("Directory is not created");
                Log.e("MKDIR_", "NOT_CREATED");

            }
        }catch(Exception e){
            e.printStackTrace();
        }

    }

    public void processImage(Bitmap frameBitmap) {
        ++timestamp;
        final long currTimestamp = timestamp;

        trackingOverlay.postInvalidate();

        computingDetection = true;
        LOGGER.i("Preparing image " + currTimestamp + " for detection in bg thread.");

        rgbFrameBitmap = frameBitmap;

        final Canvas canvas = new Canvas(croppedBitmap);
        canvas.drawBitmap(rgbFrameBitmap, frameToCropTransform, null);
        // For examining the actual TF input.
        if (SAVE_PREVIEW_BITMAP) {
            ImageUtils.saveBitmap(croppedBitmap);
        }

        runInBackground(
                new Runnable() {
                    @Override
                    public void run() {
                        LOGGER.i("Running detection on image " + currTimestamp);
                        final long startTime = SystemClock.uptimeMillis();
                        final List<Classifier.Recognition> results = detector.recognizeImage(croppedBitmap);
                        lastProcessingTimeMs = SystemClock.uptimeMillis() - startTime;

                        cropCopyBitmap = Bitmap.createBitmap(croppedBitmap);
                        final Canvas canvas = new Canvas(cropCopyBitmap);
                        final Paint paint = new Paint();
                        paint.setColor(Color.RED);
                        paint.setStyle(Style.STROKE);
                        paint.setStrokeWidth(2.0f);

                        float minimumConfidence = MINIMUM_CONFIDENCE_TF_OD_API;
                        switch (MODE) {
                            case TF_OD_API:
                                minimumConfidence = MINIMUM_CONFIDENCE_TF_OD_API;
                                break;
                        }

                        final java.util.List<Classifier.Recognition> mappedRecognitions =
                                new LinkedList<Classifier.Recognition>();
                        for (final Classifier.Recognition result : results) {
                            final RectF location = result.getLocation();


                            if (location != null && result.getConfidence() >= minimumConfidence) {

                                // only publish person and car detections.
                                if(result.getTitle().equals("person") || result.getTitle().equals("car")) {
                                    canvas.drawRect(location, paint);
                                    LOGGER.i("detection Results: " + result.getConfidence() + " " + result.getTitle());


                                    ConnectDroneActivity.rosPublishDetectionThread(result.getTitle(), result.getConfidence().toString());


                                    if(result.getConfidence()>=0.65){
                                        String filename = result.getTitle() + "_"+ ConnectDroneActivity.getFileTimestamp() + ".png";
                                        File file = new File(dir.getPath(), filename);
                                        try (FileOutputStream out = new FileOutputStream(file)) {
                                            rgbFrameBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                                        } catch (IOException e) {
                                            e.printStackTrace();
                                        }
                                    }

                                    cropToFrameTransform.mapRect(location);
                                    result.setLocation(location);
                                    mappedRecognitions.add(result);
                                }

                            }
                        }
                        tracker.trackResults(mappedRecognitions, luminanceCopy, currTimestamp);
                        trackingOverlay.postInvalidate();

                        computingDetection = false;
                    }
                });
    }

    private synchronized void runInBackground(final Runnable r) {
        if (ConnectDroneActivity.handler != null) {
            ConnectDroneActivity.handler.post(r);
        }
    }

}
