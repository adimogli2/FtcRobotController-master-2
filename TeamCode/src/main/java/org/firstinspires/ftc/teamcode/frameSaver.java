import android.os.Bundle;

import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class MainActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {
    private JavaCameraView mOpenCvCameraView;
    private Mat mRgba;
    private int frameCount = 0;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mOpenCvCameraView = (JavaCameraView) findViewById(R.id.java_camera_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat();
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        frameCount++;
        if (frameCount % 30 == 0) {
            String filename = "frame_" + frameCount + ".png";
            Imgcodecs.imwrite(filename, mRgba);
        }
        return mRgba;
    }
}
