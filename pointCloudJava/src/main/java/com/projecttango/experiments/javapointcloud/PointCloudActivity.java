/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.experiments.javapointcloud;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import android.app.Activity;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v4.app.NotificationCompat;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.UUID;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the {@link Tango}
 * service and propagation of Tango XyzIj data to OpenGL and Layout views. OpenGL rendering logic is
 * delegated to the {@link PCrenderer} class.
 */
public class PointCloudActivity extends Activity implements OnClickListener {

    private static final String TAG = PointCloudActivity.class.getSimpleName();
    private static final int SECS_TO_MILLISECS = 1000;
    private Tango mTango;
    private TangoConfig mConfig;

    private PCRenderer mRenderer;
    private GLSurfaceView mGLView;

    private TextView mDeltaTextView;
    private TextView mPoseCountTextView;
    private TextView mPoseTextView;
    private TextView mQuatTextView;
    private TextView mPoseStatusTextView;
    private TextView mTangoEventTextView;
    private TextView mPointCountTextView;
    private TextView mTangoServiceVersionTextView;
    private TextView mApplicationVersionTextView;
    private TextView mAverageZTextView;
    private TextView mFrequencyTextView;

    private Button mFirstPersonButton;
    private Button mThirdPersonButton;
    private Button mTopDownButton;
    private ToggleButton mOnButton;

    private int count;
    private int mPreviousPoseStatus;
    private int mPointCount;
    private float mDeltaTime;
    private float mPosePreviousTimeStamp;
    private float mXyIjPreviousTimeStamp;
    private float mCurrentTimeStamp;
    private float mPointCloudFrameDelta;
    private String mServiceVersion;
    private boolean mIsTangoServiceConnected;
    private TangoPoseData mPose;
    private static final int UPDATE_INTERVAL_MS = 100;
    public static Object poseLock = new Object();
    public static Object depthLock = new Object();


    private String  translationString;
    private String quaternionString;
    private Boolean isOn = false;
    private UUID uuid;
    private long nanoTime;
    private double poseTimestamp;
    private FloatBuffer mxyz;
    private float[] s;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_jpoint_cloud);
        setTitle(R.string.app_name);

        mPoseTextView = (TextView) findViewById(R.id.pose);
        mQuatTextView = (TextView) findViewById(R.id.quat);
        mPoseCountTextView = (TextView) findViewById(R.id.posecount);
        mDeltaTextView = (TextView) findViewById(R.id.deltatime);
        mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);
        mPoseStatusTextView = (TextView) findViewById(R.id.status);
        mPointCountTextView = (TextView) findViewById(R.id.pointCount);
        mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
        mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);
        mAverageZTextView = (TextView) findViewById(R.id.averageZ);
        mFrequencyTextView = (TextView) findViewById(R.id.frameDelta);

        mFirstPersonButton = (Button) findViewById(R.id.first_person_button);
        mFirstPersonButton.setOnClickListener(this);
        mThirdPersonButton = (Button) findViewById(R.id.third_person_button);
        mThirdPersonButton.setOnClickListener(this);
        mTopDownButton = (Button) findViewById(R.id.top_down_button);
        mTopDownButton.setOnClickListener(this);
        mOnButton = (ToggleButton) findViewById(R.id.onButton);
        mOnButton.setOnClickListener(this);

        mTango = new Tango(this);
        mConfig = new TangoConfig();
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);

        int maxDepthPoints = mConfig.getInt("max_point_cloud_elements");
        mRenderer = new PCRenderer(maxDepthPoints);
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
        mGLView.setEGLContextClientVersion(2);
        mGLView.setRenderer(mRenderer);

        PackageInfo packageInfo;
        try {
            packageInfo = this.getPackageManager().getPackageInfo(this.getPackageName(), 0);
            mApplicationVersionTextView.setText(packageInfo.versionName);
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }

        // Display the version of Tango Service
        mServiceVersion = mConfig.getString("tango_service_library_version");
        mTangoServiceVersionTextView.setText(mServiceVersion);
        mIsTangoServiceConnected = false;
        startUIThread();
    }

    @Override
    protected void onPause() {
        super.onPause();

        if(isOn) {
            mIsTangoServiceConnected = true;
        }
        else {
            try {
                mTango.disconnect();
                mIsTangoServiceConnected = false;
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsTangoServiceConnected) {
            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                    Tango.TANGO_INTENT_ACTIVITYCODE);
        }
        Log.i(TAG, "onResumed");
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission, Toast.LENGTH_LONG).show();
                finish();
                return;
            }
            try {
                setTangoListeners();
            } catch (TangoErrorException e) {
                Toast.makeText(this, R.string.TangoError, Toast.LENGTH_SHORT).show();
            } catch (SecurityException e) {
                Toast.makeText(getApplicationContext(), R.string.motiontrackingpermission,
                        Toast.LENGTH_SHORT).show();
            }
            try {
                mTango.connect(mConfig);
                mIsTangoServiceConnected = true;
            } catch (TangoOutOfDateException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoOutOfDateException,
                        Toast.LENGTH_SHORT).show();
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT)
                        .show();
            }
            setUpExtrinsics();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
        case R.id.first_person_button:
            mRenderer.setFirstPersonView();
            break;
        case R.id.third_person_button:
            mRenderer.setThirdPersonView();
            break;
        case R.id.top_down_button:
            mRenderer.setTopDownView();
            break;
            case R.id.onButton:
                isOn = mOnButton.isChecked();

                // Gets an instance of the NotificationManager service
                NotificationManager mNotifyMgr = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);

                if(isOn){ // if session has started add notification and create UUID

                    //Intent for Stop button on Notification
                    Intent intent = new Intent(this, PointCloudActivity.class);
                    intent.putExtra("isOn","false");
                    PendingIntent aIntent = PendingIntent.getActivity(this, 0, intent, 0);

                    //NOTIFICATION SETUP
                    NotificationCompat.Builder mBuilder =
                            new NotificationCompat.Builder(this)
                                    .setSmallIcon(R.drawable.ic_launcher)
                                    .setContentTitle("UTC Tango Point Cloud")
                                    .setContentText("Collecting Data! " + getCurrentTimeStamp())
                                    .setOngoing(true)
                                    .setPriority(2)
                                  //  .setAutoCancel(true)
                                   // .setDeleteIntent(aIntent)
                                    .addAction(R.drawable.ic_launcher, "Reset - Stop Session", aIntent);

                    int mNotificationId = 8001; //Notification ID

                    //create UUID for session;
                    uuid = UUID.randomUUID();

                    //Starting Toast
                    Toast.makeText(getApplicationContext(),
                            "Collecting Data...", Toast.LENGTH_SHORT)
                            .show();

                    // Builds the notification and issues it.
                    mNotifyMgr.notify(mNotificationId, mBuilder.build());
                } else{
                    mNotifyMgr.cancelAll(); //kill notification

                    //Stopping Toast
                    Toast.makeText(getApplicationContext(),
                            "Stopped collecting data", Toast.LENGTH_SHORT)
                            .show();
                }
                break;
        default:
            Log.w(TAG, "Unrecognized button click.");
            return;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        return mRenderer.onTouchEvent(event);
    }

    private void setUpExtrinsics() {
        // Set device to imu matrix in Model Matrix Calculator.
        TangoPoseData device2IMUPose = new TangoPoseData();
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        try {
            device2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetDevice2IMUMatrix(
                device2IMUPose.getTranslationAsFloats(), device2IMUPose.getRotationAsFloats());

        // Set color camera to imu matrix in Model Matrix Calculator.
        TangoPoseData color2IMUPose = new TangoPoseData();

        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        try {
            color2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetColorCamera2IMUMatrix(
                color2IMUPose.getTranslationAsFloats(), color2IMUPose.getRotationAsFloats());
    }

    private void setTangoListeners() {


            // Configure the Tango coordinate frame pair
            final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
            framePairs.add(new TangoCoordinateFramePair(
                    TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                    TangoPoseData.COORDINATE_FRAME_DEVICE));
            // Listen for new Tango data
            mTango.connectListener(framePairs, new OnTangoUpdateListener() {

                @Override
                public void onPoseAvailable(final TangoPoseData pose) {
                    // Make sure to have atomic access to Tango Pose Data so that
                    // render loop doesn't interfere while Pose call back is updating
                    // the data.
                    synchronized (poseLock) {
                        mPose = pose;
                        // Calculate the delta time from previous pose.
                        mDeltaTime = (float) (pose.timestamp - mPosePreviousTimeStamp)
                                * SECS_TO_MILLISECS;
                        mPosePreviousTimeStamp = (float) pose.timestamp;
                        if (mPreviousPoseStatus != pose.statusCode) {
                            count = 0;
                        }
                        count++;
                        mPreviousPoseStatus = pose.statusCode;
                        if (!mRenderer.isValid()) {
                            return;
                        }
                        mRenderer.getModelMatCalculator().updateModelMatrix(
                                pose.getTranslationAsFloats(), pose.getRotationAsFloats());
                        mRenderer.updateViewMatrix();
                    }

                    poseTimestamp = pose.timestamp;

                }

                @Override
                public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                    // Make sure to have atomic access to TangoXyzIjData so that
                    // render loop doesn't interfere while onXYZijAvailable callback is updating
                    // the point cloud data.
                    synchronized (depthLock) {
                        mCurrentTimeStamp = (float) xyzIj.timestamp;
                        mPointCloudFrameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp)
                                * SECS_TO_MILLISECS;
                        mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                        try {
                            TangoPoseData pointCloudPose = mTango.getPoseAtTime(mCurrentTimeStamp,
                                    framePairs.get(0));
                            mPointCount = xyzIj.xyzCount;
                            if (!mRenderer.isValid()) {
                                return;
                            }
                            mRenderer.getPointCloud().UpdatePoints(xyzIj.xyz);
                            mRenderer.getModelMatCalculator().updatePointCloudModelMatrix(
                                    pointCloudPose.getTranslationAsFloats(),
                                    pointCloudPose.getRotationAsFloats());
                            mRenderer.getPointCloud().setModelMatrix(
                                    mRenderer.getModelMatCalculator().getPointCloudModelMatrixCopy());
                        } catch (TangoErrorException e) {
                            Toast.makeText(getApplicationContext(), R.string.TangoError,
                                    Toast.LENGTH_SHORT).show();
                        } catch (TangoInvalidException e) {
                            Toast.makeText(getApplicationContext(), R.string.TangoError,
                                    Toast.LENGTH_SHORT).show();
                        }
                    }

                    if (isOn) {

                       //  s = new float[xyzIj.ijRows + xyzIj.ijCols];
                       mxyz = xyzIj.xyz.asReadOnlyBuffer();


                        nanoTime = System.nanoTime();

                        translationString = mPose.translation[0] + ", "
                                + mPose.translation[1] + ", "
                                + mPose.translation[2];
                        quaternionString =
                                +mPose.rotation[0] + ", "
                                        + mPose.rotation[1] + ", "
                                        + mPose.rotation[2] + ", "
                                        + mPose.rotation[3];

                        try {
                            //timestamp = pose.timestamp; //get timestamp data

                            //Initializes HTTP POST request
                            HttpClient httpclient = new DefaultHttpClient();
                            HttpPost httpPost = new HttpPost("http://10.101.102.123/datapoint");
                            List<NameValuePair> nameValuePair = new ArrayList<NameValuePair>(2);
                            nameValuePair.add(new BasicNameValuePair("uuid", uuid + ""));
                            nameValuePair.add(new BasicNameValuePair("timestamp",   getCurrentTimeStamp()));


                            nameValuePair.add(new BasicNameValuePair("nanoTime", nanoTime + ""));
                            nameValuePair.add(new BasicNameValuePair("pose_timestamp", poseTimestamp + ""));

                            nameValuePair.add(new BasicNameValuePair("translation", translationString));
                            nameValuePair.add(new BasicNameValuePair("rotation", quaternionString));

                            for(int i = 0; i < 10; i++) {
                                nameValuePair.add(new BasicNameValuePair("xyz["+i+"]", " | x | " + mxyz.get() + " | y | " + mxyz.get() + " | z | " + mxyz.get()));
                            }



                            httpPost.setEntity(new UrlEncodedFormEntity(nameValuePair));

                            try {
                                //Send Request
                                HttpResponse response = httpclient.execute(httpPost);
                                // write response to log
                                Log.d("Http Post Response:", response.toString());
                            } catch (ClientProtocolException e) { // Catch a few different Exceptions
                                // Log exception
                                e.printStackTrace();
                            } catch (IOException e) {
                                // Log exception
                                e.printStackTrace();
                                Log.i(TAG, e.getMessage());
                            }

                            // HttpResponse response = httpclient.execute(new HttpGet("http://localhost:1234/send-data"));
                        } catch (Exception exception) {
                            Log.i(TAG, exception.getMessage());
                        }

                    } else {
                    }

                }

                @Override
                public void onTangoEvent(final TangoEvent event) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mTangoEventTextView.setText(event.eventKey + ": " + event.eventValue);
                        }
                    });
                }

                @Override
                public void onFrameAvailable(int cameraId) {
                    // We are not using onFrameAvailable for this application.
                }
            });

    }

    /**
     * Create a separate thread to update Log information on UI at the specified interval of
     * UPDATE_INTERVAL_MS. This function also makes sure to have access to the mPose atomically.
     */
    private void startUIThread() {
        new Thread(new Runnable() {
            final DecimalFormat threeDec = new DecimalFormat("0.000");

            @Override
            public void run() {
                while (true) {
                    try {
                        Thread.sleep(UPDATE_INTERVAL_MS);
                        // Update the UI with TangoPose information
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                synchronized (poseLock) {
                                    if (mPose == null) {
                                        return;
                                    }
                                    translationString = "["
                                            + threeDec.format(mPose.translation[0]) + ", "
                                            + threeDec.format(mPose.translation[1]) + ", "
                                            + threeDec.format(mPose.translation[2]) + "] ";
                                    quaternionString = "["
                                            + threeDec.format(mPose.rotation[0]) + ", "
                                            + threeDec.format(mPose.rotation[1]) + ", "
                                            + threeDec.format(mPose.rotation[2]) + ", "
                                            + threeDec.format(mPose.rotation[3]) + "] ";

                                    // Display pose data on screen in TextViews
                                    mPoseTextView.setText(translationString);
                                    mQuatTextView.setText(quaternionString);
                                    mPoseCountTextView.setText(Integer.toString(count));
                                    mDeltaTextView.setText(threeDec.format(mDeltaTime));
                                    if (mPose.statusCode == TangoPoseData.POSE_VALID) {
                                        mPoseStatusTextView.setText(R.string.pose_valid);
                                    } else if (mPose.statusCode == TangoPoseData.POSE_INVALID) {
                                        mPoseStatusTextView.setText(R.string.pose_invalid);
                                    } else if (mPose.statusCode == TangoPoseData.POSE_INITIALIZING) {
                                        mPoseStatusTextView.setText(R.string.pose_initializing);
                                    } else if (mPose.statusCode == TangoPoseData.POSE_UNKNOWN) {
                                        mPoseStatusTextView.setText(R.string.pose_unknown);
                                    }
                                }
                                synchronized (depthLock) {
                                    // Display number of points in the point cloud
                                    mPointCountTextView.setText(Integer.toString(mPointCount));
                                    mFrequencyTextView.setText(""
                                            + threeDec.format(mPointCloudFrameDelta));
                                    mAverageZTextView.setText(""
                                            + threeDec.format(mRenderer.getPointCloud()
                                                    .getAverageZ()));
                                }
                            }
                        });
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).start();
    }

    //Method to get System timestamp
    public static String getCurrentTimeStamp(){
        try {

            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
            String currentTimeStamp = dateFormat.format(new Date()); // Find todays date

            return currentTimeStamp;
        } catch (Exception e) {
            e.printStackTrace();

            return null;
        }
    }
}
