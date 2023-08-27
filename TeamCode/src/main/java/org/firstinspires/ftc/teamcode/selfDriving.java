package org.firstinspires.ftc.teamcode;

//import static android.os.Environment.getExternalStorageDirectory;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.opencv.core.Core;
import org.datavec.image.loader.AndroidNativeImageLoader;
import org.datavec.image.loader.Java2DNativeImageLoader;
import org.deeplearning4j.util.ModelSerializer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.api.ndarray.INDArray;
//import org.nd4j.linalg.factory.Nd4j;
//import org.datavec.image.loader.NativeImageLoader;
import org.datavec.image.loader.LocalImageLoader;


import java.io.File;
import java.io.IOException;
import java.lang.annotation.Native;
import java.text.SimpleDateFormat;

@TeleOp(name="selfDriving", group="Iterative Opmode")
public class selfDriving extends LinearOpMode {
    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    double dir = 0;
    double hyp = 0;
    double X = 0;
    double Y = 0;
    double ROT_X = 0;
    double ROT_Y = 0;
    int imageCount = 0;
    MultiLayerNetwork model = null;

    private OpenCvCamera camera;

    private VideoCapture videoCapture = new VideoCapture();
    private VideoWriter videoWriter;

    //    private static final int  CAMERA_FRAME_WIDTH = 640;
//    private static final int CAMERA_FRAME_HEIGHT = 480;
//    private static final int CAMERA_FRAME_RATE = 30;
    private static final String VIDEO_FILE_EXTENSION = ".jpg";

    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            sleep(5000);
            initializeVideoRecording();
            waitForStart();

            while (opModeIsActive()) {

            }
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        } finally {
            stopVideoRecording();
        }
    }
    private void initializeHardware() {
        FL0 = hardwareMap.get(DcMotor.class, "Front_Left");
        BL1 = hardwareMap.get(DcMotor.class, "Back_Left");
        BR2 = hardwareMap.get(DcMotor.class, "Back_Right");
        FR3 = hardwareMap.get(DcMotor.class, "Front_Right");
        // Initialize other hardware components if needed
    }
    private void initializeVideoRecording() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //String fileName = String.format("%f_%f_%f_%f%s", FL0.getPower(), BL1.getPower(), BR2.getPower(), FR3.getPower(), VIDEO_FILE_EXTENSION);
        //String fileName = String.format("%d_%f%s", System.currentTimeMillis()/1000, dir, VIDEO_FILE_EXTENSION);
//        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(System.currentTimeMillis());
//        File storageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM) + "/Camera/");
//        if (!storageDir.exists())
//            storageDir.mkdirs();
        //String fileName = String.format("%f_%f%s", dir, hyp, VIDEO_FILE_EXTENSION);
//        String fileName = String.format("%f_%f%s", X, Y, VIDEO_FILE_EXTENSION);
//        File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), fileName);
//        //File videoFile = new File(storageDir, fileName);
//        //to save image sequence, use a proper filename; set fourcc=0 or fps=0
//        videoWriter = new VideoWriter(videoFile.getAbsolutePath(), 0, 0, new Size(240, 320), true);

        File storageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM) + "/Download/");
        String modelPath = storageDir + "lane_navigation_final.h5"; // Path to the
        Log.d("model path", modelPath);
        try {
            model = ModelSerializer.restoreMultiLayerNetwork(modelPath);
        } catch(Exception e){
            Log.d("Model Error", e.getMessage());
        }
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        camera.setPipeline(new SignalPipeline());
    }
    private void updateMotorPowers(double tgp_x, double tgp_y, double rot_x, double rot_y, boolean mac1) {
        // Update motor powers based on joystick input and other conditions
        // Modify the logic as per your requirements
        FL0.setPower(tgp_y);
        BL1.setPower(tgp_y);
        BR2.setPower(tgp_y);
        FR3.setPower(tgp_y);
    }
    private void writeFrameToVideo() {
        Mat frame = new Mat();
        Mat cv_frame = new Mat();
        if (videoCapture.read(frame)) {
            Imgproc.cvtColor(cv_frame, frame, Imgproc.COLOR_BGR2HSV);
            videoWriter.write(cv_frame);
        }
    }
    private void stopVideoRecording() {
        videoCapture.release();
        videoWriter.release();
    }

    private double robotControl(double rot_x){
        double tgp_y = 0;
        double tgp_x = 0;
        //double rot_x = 0;
        double rot_y = 0;
        boolean mac1 = false;
        //if going straight dir is 90 degrees
        //if rot_x = 0
        if(rot_x == 0){
            tgp_y = 0.5;
        }
        FL0.setPower(0);
        BL1.setPower(0);
        BR2.setPower(0);
        FR3.setPower(0);

//        tgp_x = gamepad1.left_stick_x;
//        tgp_y = gamepad1.left_stick_y * -1;
//        rot_x = gamepad1.right_stick_x;
//        rot_y = gamepad1.right_stick_y;
//        mac1 = gamepad1.dpad_right;
//        X = tgp_x;
//        Y = tgp_y;
//        ROT_X = rot_x;
//        ROT_Y = rot_y;

        if(rot_x > 0) {
            //multiplying by 0.3 to make sure it is the maximum
            FL0.setPower(rot_x * 0.3);
            BL1.setPower(rot_x * 0.3);
            BR2.setPower(rot_x * -0.2);
            FR3.setPower(rot_x * -0.2);
        }
        else if (rot_x < 0){
            FL0.setPower(rot_x * 0.2);
            BL1.setPower(rot_x * 0.2);
            BR2.setPower(rot_x * -0.3);
            FR3.setPower(rot_x * -0.3);
        }
        else if(tgp_y > 0){
            FL0.setPower(0.25);
            BL1.setPower(0.25);
            BR2.setPower(0.25);
            FR3.setPower(0.25);
        }
        else if(tgp_y < 0){
            FL0.setPower(0.25 * -1);
            BL1.setPower(0.25 * -1);
            BR2.setPower(0.25 * -1);
            FR3.setPower(0.25 * -1);
        }
        return rot_x;
    }
    class SignalPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            org.opencv.core.Mat hsv = new Mat();
            // Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            // Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2RGB);

            // Rotate the image by 90 degrees counter-clockwise
            Core.transpose(hsv, hsv);
            Core.flip(hsv, hsv, 0);

            //add model code here
            int deg = 90;
            try {
                System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
                //Mat imgMat = Imgcodecs.imread("C:\\Pictures\\image.jpg");
                //INDArray image = loader.asMatrix(imgMat);
                LocalImageLoader nil = new LocalImageLoader();

                INDArray image = nil.asMatrix(hsv);
                INDArray strng_angle = model.output(image);
                deg = strng_angle.getInt(0);
                Log.d("steering angle", Integer.toString(deg));
            }catch(Exception e){
                Log.d("Model Error", e.getMessage());
            }
            double rot = 0;
            if (deg == 90) {
                rot = 0;
            } else if (deg < 90) {
                rot = 1.0 - (deg / 90.0);
            } else {
                rot = -1 * (1.0 - ((180 - deg) / 90.0));
            }
            Log.d("rot_x", Double.toString(rot));
            robotControl(rot);
            // ... (rest of your existing code)
            if(imageCount % 500 == 0) {
                imageCount++;
                String timeStamp = new SimpleDateFormat("yy.MMddHHmmss.SSS").format(System.currentTimeMillis());
                File sdCard = Environment.getExternalStorageDirectory();
                File storageDir = new File(sdCard.getAbsolutePath() + "/Camera/");
                if (!storageDir.exists())
                    storageDir.mkdirs();

                String fileName = String.format("%s_%03d%s", timeStamp, deg, VIDEO_FILE_EXTENSION);
                Log.d("ROT_X", Double.toString(rot));
                Log.d("fileName", fileName);

                File videoFile = new File(storageDir, fileName);
                videoWriter = new VideoWriter(videoFile.getAbsolutePath(), 0, 0, new Size(240, 320), true);
                Log.d("videoFile", videoFile.toString());
                videoWriter.write(hsv);
            }
            return hsv;
        }

    }
}