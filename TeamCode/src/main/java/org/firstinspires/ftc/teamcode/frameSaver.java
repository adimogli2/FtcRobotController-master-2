/*
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.os.Bundle;
import android.view.SurfaceView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class frameSaver extends Activity implements CvCameraViewListener2 {
    private CameraBridgeViewBase cameraView;
    private int frameCount = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_save_frames);

        // Set up the camera view
        cameraView = (JavaCameraView) findViewById(R.id.camera_view);
        cameraView.setVisibility(SurfaceView.VISIBLE);
        cameraView.setCvCameraViewListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {}

    @Override
    public void onCameraViewStopped() {}

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        // Get the current frame from the camera
        Mat frame = inputFrame.rgba();

        // Save the frame to a file
        String filename = "frame" + frameCount + ".png";
        Imgcodecs.imwrite(filename, frame);

        // Increment the frame count
        frameCount++;

        return frame;
    }
}
*/
