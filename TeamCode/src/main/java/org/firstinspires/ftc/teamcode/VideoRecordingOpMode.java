package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

@TeleOp(name = "VideoRecordingOpMode", group = "Iterative Opmode")
public class VideoRecordingOpMode extends LinearOpMode {
    private DcMotor motorFL0;
    private DcMotor motorBL1;
    private DcMotor motorBR2;
    private DcMotor motorFR3;

    private CameraName cameraName;
    private VideoCapture videoCapture;
    private VideoWriter videoWriter;

    private OpenCvInternalCamera camera;

    static final int CAMERA_FRAME_WIDTH = 640;
    static final int CAMERA_FRAME_HEIGHT = 480;
    static final int CAMERA_FRAME_RATE = 30;
    static final String VIDEO_FILE_EXTENSION = ".avi";

    @Override
    public void runOpMode() {
        try {
            //working
            initializeHardware();
            initializeVideoRecording();
            //working
            waitForStart();


            while (opModeIsActive()) {
//                double motorFL0Power = motorFL0.getPower();
//                double motorBL1Power = motorBL1.getPower();
//                double motorBR2Power = motorBR2.getPower();
//                double motorFR3Power = motorFR3.getPower();
//
//                // Update motor powers and write frame to the video file
//                updateMotorPowers(motorFL0Power, motorBL1Power, motorBR2Power, motorFR3Power);
//                int count = 0;
//                count++;
//                telemetry.addData("inOpmode ", count);
//                telemetry.update();
//
//                writeFrameToVideo();
//
//                // Update telemetry
////                telemetry.addData("Motor FL Power", motorFL0Power);
////                telemetry.addData("Motor BL Power", motorBL1Power);
////                telemetry.addData("Motor BR Power", motorBR2Power);
////                telemetry.addData("Motor FR Power", motorFR3Power);
////                telemetry.update();
            }
        }catch(Exception e){
            String err = e.getMessage();
            telemetry.addData("Error ", err);
            telemetry.update();
        }
        stopVideoRecording();
        videoWriter.release();
    }

    private void initializeHardware() {
        try {
            motorFL0 = hardwareMap.get(DcMotor.class, "Front_Left");
            motorBL1 = hardwareMap.get(DcMotor.class, "Back_Left");
            motorBR2 = hardwareMap.get(DcMotor.class, "Back_Right");
            motorFR3 = hardwareMap.get(DcMotor.class, "Front_Right");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
            // Configure and initialize other hardware components
        }catch(Exception e){
            String err = e.getMessage();
            telemetry.addData("Error ", err);
            telemetry.update();
        }
    }

    private void initializeVideoRecording() {
        // Initialize OpenCV
        try {
            //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            //Initialize the video capture
            videoCapture = new VideoCapture();
            videoCapture.open(0); // 0 represents the default internal camera

            // Configure video capture properties
            videoCapture.set(3, CAMERA_FRAME_WIDTH);
            videoCapture.set(4, CAMERA_FRAME_HEIGHT);
            videoCapture.set(5, CAMERA_FRAME_RATE);

            //Initialize the video writer
            //String fileName = String.format("%f_%f_%f_%f%s", motorFL0.getPower(), motorBL1.getPower(), motorBR2.getPower(), motorFR3.getPower(), VIDEO_FILE_EXTENSION);
            String fileName = String.format("AutoRobot%s", VIDEO_FILE_EXTENSION);
            File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), fileName);
            telemetry.addData("Success in videoFilename ", videoFile.getName());
            telemetry.update();
            videoWriter = new VideoWriter(videoFile.getName(), VideoWriter.fourcc('M', 'J', 'P', 'G'), CAMERA_FRAME_RATE, new org.opencv.core.Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT));
            //telemetry.addData("Success in videoWriter ", videoWriter);
            //telemetry.update();

        }catch(Exception e) {
            String err = e.getMessage();
            telemetry.addData("Error ", err);
            telemetry.update();
        }
    }



    private void updateMotorPowers(double motorFLPower, double motorBLPower, double motorBRPower, double motorFRPower) {
        motorFL0.setPower(motorFLPower);
        motorBL1.setPower(motorBLPower);
        motorBR2.setPower(motorBRPower);
        motorFR3.setPower(motorFRPower);
    }

    private void writeFrameToVideo() {
        try {
            Mat frame = new Mat();
            int count = 0;
            if (videoCapture.read(frame)) {
                count++;
                telemetry.addData("Read Frame ", count);
                telemetry.update();
                videoWriter.write(frame);
                telemetry.addData("Written Frame ", count);
                telemetry.update();

            }
        }catch(Exception e) {
            String err = e.getMessage();
            telemetry.addData("Error ", err);
            telemetry.update();
        }
    }

    private void stopVideoRecording() {
        videoCapture.release();
        videoWriter.release();
    }
}

/*class SignalPipeline extends OpenCvPipeline {
    @Override
//    public Mat processFrame(Mat input) {
//        Mat hsv = new Mat();
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//        Mat maskRed = new Mat();
//        Mat maskGreen = new Mat();
//        Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(10, 255, 255), maskRed);
//        //Core.inRange(hsv, new Scalar(38, 70, 50), new Scalar(75, 255, 255), maskGreen);
//        if (Core.countNonZero(maskGreen) > 100) {
//            // red signal
//            telemetry.addData("Signal", "Green: " + Core.countNonZero(maskGreen));
//            telemetry.update();
//            FL0.setPower(0.25);
//            BL1.setPower(0.25);
//            BR2.setPower(0.25);
//            FR3.setPower(0.25);
//        } else {
//            // green signal
//            telemetry.addData("Signal", "Red: " + Core.countNonZero(maskGreen));
//            telemetry.update();
//            FL0.setPower(0);
//            BL1.setPower(0);
//            BR2.setPower(0);
//            FR3.setPower(0);
//        }
//        return input;
//    }
    // Initialize OpenCV
        public Mat processFrame(Mat input) {
        try {
            //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            //Initialize the video capture
            VideoCapture videoCapture = new VideoCapture();
            videoCapture.open(0); // 0 represents the default internal camera

            // Configure video capture properties
            videoCapture.set(3, VideoRecordingOpMode.CAMERA_FRAME_WIDTH);
            videoCapture.set(4, VideoRecordingOpMode.CAMERA_FRAME_HEIGHT);
            videoCapture.set(5, VideoRecordingOpMode.CAMERA_FRAME_RATE);

            //Initialize the video writer
            //String fileName = String.format("%f_%f_%f_%f%s", motorFL0.getPower(), motorBL1.getPower(), motorBR2.getPower(), motorFR3.getPower(), VIDEO_FILE_EXTENSION);
            String fileName = String.format("AutoRobot%s", VideoRecordingOpMode.VIDEO_FILE_EXTENSION);
            File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), fileName);
            //telemetry.addData("Success in videoFilename ", videoFile.getName());
            //telemetry.update();
            VideoWriter videoWriter = new VideoWriter(videoFile.getName(), VideoWriter.fourcc('M', 'J', 'P', 'G'),
                    VideoRecordingOpMode.CAMERA_FRAME_RATE, new org.opencv.core.Size(VideoRecordingOpMode.CAMERA_FRAME_WIDTH, VideoRecordingOpMode.CAMERA_FRAME_HEIGHT));
            //telemetry.addData("Success in videoWriter ", videoWriter);
            //telemetry.update();
            //
//          int count = 0;
            if (videoCapture.read(input)) {
                //count++;
                //telemetry.addData("Read Frame ", count);
                //telemetry.update();
                videoWriter.write(input);
                //telemetry.addData("Written Frame ", count);
                //telemetry.update();

            }

        } catch (Exception e) {
            String err = e.getMessage();
            //telemetry.addData("Error ", err);
            //telemetry.update();
        }
        return input;
    }
}*/

