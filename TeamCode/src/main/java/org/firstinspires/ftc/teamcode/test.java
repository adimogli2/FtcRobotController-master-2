package org.firstinspires.ftc.teamcode;

//import static android.os.Environment.getExternalStorageDirectory;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.opencv.core.Core;
import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


import java.io.File;
import java.text.SimpleDateFormat;

@TeleOp(name="test", group="Iterative Opmode")
public class test extends LinearOpMode {
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
            initializeVideoRecording();
            waitForStart();

            while (opModeIsActive()) {
//                double tgp_x = gamepad1.left_stick_x;
//                double tgp_y = gamepad1.left_stick_y * -1;
//                double rot_x = gamepad1.right_stick_x;
//                double rot_y = gamepad1.right_stick_y;
//                boolean mac1 = gamepad1.dpad_right;

                // Update motor powers
                //updateMotorPowers(tgp_x, tgp_y, rot_x, rot_y, mac1);

                // Write frame to the video file
                //writeFrameToVideo();
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
    private double robotControl(){
        double tgp_y = 0;
        double tgp_x = 0;
        double rot_x = 0;
        double rot_y = 0;
        boolean mac1 = false;
        //if going straight dir is 90 degrees

        tgp_x = gamepad1.left_stick_x;
        tgp_y = gamepad1.left_stick_y * -1;
        rot_x = gamepad1.right_stick_x;
        rot_y = gamepad1.right_stick_y;
        mac1 = gamepad1.dpad_right;
        X = tgp_x;
        Y = tgp_y;
        ROT_X = rot_x;
        ROT_Y = rot_y;

        hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
        //double hyp = hypt * -1;
        //if on y-axis
        if(tgp_x == 0){
            FL0.setPower(tgp_y);
            BL1.setPower(tgp_y);
            BR2.setPower(tgp_y);
            FR3.setPower(tgp_y);
        }
        //if on x-axis (strafing)
        else if(tgp_y == 0 ) {
            FL0.setPower(tgp_x);
            BL1.setPower(tgp_x * -1);
            BR2.setPower(tgp_x);
            FR3.setPower(tgp_x * -1);
            dir = 90;
        }
        //turning right or left
        // else if(rot_x >= -1 && rot_x <= 1) {
        //FL0.setPower(tgp_x);
        //BL1.setPower(tgp_x * -1);
        //BR2.setPower(tgp_x);
        //FR3.setPower(tgp_x * -1);
        //}
        if(rot_y == 0){
            FL0.setPower(rot_x);
            BL1.setPower(rot_x);
            BR2.setPower(rot_x * -1);
            FR3.setPower(rot_x * -1);
        }
        //if in first quadrant
        if(tgp_x > 0 && tgp_y > 0 ){
            //hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
            FL0.setPower(hyp);
            BL1.setPower(0);
            BR2.setPower(hyp);
            FR3.setPower(0);
            dir = Math.atan(tgp_y/tgp_x) * 180/Math.PI;
            telemetry.addData("Angle", dir);
            telemetry.update();
        }
        //if in second quadrant
        else if(tgp_x < 0 && tgp_y > 0 ){
            //hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
            FL0.setPower(0);
            BL1.setPower(hyp);
            BR2.setPower(0);
            FR3.setPower(hyp);
            dir = Math.atan(tgp_y/tgp_x) * 180/Math.PI;
            telemetry.addData("Angle", dir);
            telemetry.update();
        }
        //if in third quadrant
        else if(tgp_x < 0 && tgp_y < 0 ){
            //hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
            FL0.setPower(hyp * -1);
            BL1.setPower(0);
            BR2.setPower(hyp * -1);
            FR3.setPower(0);
        }
        //if in fourth quadrant
        else if(tgp_x > 0 && tgp_y < 0 ) {
            //hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
            FL0.setPower(0);
            BL1.setPower(hyp * -1);
            BR2.setPower(0);
            FR3.setPower(hyp * -1);
        }
        else if(mac1) {
            FL0.setPower(0.5);
            BL1.setPower(0.5);
            BR2.setPower(0.5 * -1);
            FR3.setPower(0.5 * -1);
            sleep(2000);
        }
        return dir;
    }
    class SignalPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            //Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            //Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2RGB);
            robotControl();
            String timeStamp = new SimpleDateFormat("MMddHHmmss").format(System.currentTimeMillis());
            File storageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM) + "/Camera/");
            if (!storageDir.exists())
                storageDir.mkdirs();
            String fileName = String.format("%s_%f_%f_%f%s", timeStamp, dir, hyp, ROT_X, VIDEO_FILE_EXTENSION);
//            Log.d("ROT_X", Double.toString(ROT_X));
//            Log.d("ROT_Y", Double.toString(ROT_Y));
//            Log.d("DIR", Double.toString(dir));
//            Log.d("HYP", Double.toString(hyp));
            Log.d("fileName", fileName);
            //telemetry.addData("filename", fileName);
            //telemetry.update();
            //File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), fileName);
            File videoFile = new File(storageDir, fileName);
            //to save image sequence, use a proper filename; set fourcc=0 or fps=0
            videoWriter = new VideoWriter(videoFile.getAbsolutePath(), 0, 0, new Size(240, 320), true);
            videoWriter.write(hsv);
            return hsv;
        }
    }
}