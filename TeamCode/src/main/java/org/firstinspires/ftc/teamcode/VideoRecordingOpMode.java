package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "VideoRecordingOpMode", group = "Iterative Opmode")
public class VideoRecordingOpMode extends LinearOpMode {
    private DcMotor motorFL0;
    private DcMotor motorBL1;
    private DcMotor motorBR2;
    private DcMotor motorFR3;

    private CameraName cameraName;
    private VideoCapture videoCapture;
    private VideoWriter videoWriter;

    private static final int CAMERA_FRAME_WIDTH = 640;
    private static final int CAMERA_FRAME_HEIGHT = 480;
    private static final int CAMERA_FRAME_RATE = 30;
    private static final String VIDEO_FILE_EXTENSION = ".avi";

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeVideoRecording();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double motorFL0Power = motorFL0.getPower();
                double motorBL1Power = motorBL1.getPower();
                double motorBR2Power = motorBR2.getPower();
                double motorFR3Power = motorFR3.getPower();

                // Update motor powers and write frame to the video file
                updateMotorPowers(motorFL0Power, motorBL1Power, motorBR2Power, motorFR3Power);
                writeFrameToVideo();

                // Update telemetry
                telemetry.addData("Motor FL Power", motorFL0Power);
                telemetry.addData("Motor BL Power", motorBL1Power);
                telemetry.addData("Motor BR Power", motorBR2Power);
                telemetry.addData("Motor FR Power", motorFR3Power);
                telemetry.update();
            }
        }

        stopVideoRecording();
    }

    private void initializeHardware() {
        motorFL0 = hardwareMap.get(DcMotor.class, "Front_Left");
        motorBL1 = hardwareMap.get(DcMotor.class, "Back_Left");
        motorBR2 = hardwareMap.get(DcMotor.class, "Back_Right");
        motorFR3 = hardwareMap.get(DcMotor.class, "Front_Right");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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
    }

    private void initializeVideoRecording() {
        // Initialize OpenCV
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Initialize the video capture
        videoCapture = new VideoCapture();
        videoCapture.open(0); // 0 represents the default internal camera

        // Configure video capture properties
        videoCapture.set(3, CAMERA_FRAME_WIDTH);
        videoCapture.set(4, CAMERA_FRAME_HEIGHT);
        videoCapture.set(5, CAMERA_FRAME_RATE);

        // Initialize the video writer
        String videoFilename = String.format("%d_%d_%d_%d%s", motorFL0.getPower(), motorBL1.getPower(), motorBR2.getPower(), motorFR3.getPower(), VIDEO_FILE_EXTENSION);
        videoWriter = new VideoWriter(videoFilename, VideoWriter.fourcc('M', 'J', 'P', 'G'), CAMERA_FRAME_RATE, new org.opencv.core.Size(CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT));
    }

    private void updateMotorPowers(double motorFLPower, double motorBLPower, double motorBRPower, double motorFRPower) {
        motorFL0.setPower(motorFLPower);
        motorBL1.setPower(motorBLPower);
        motorBR2.setPower(motorBRPower);
        motorFR3.setPower(motorFRPower);
    }

    private void writeFrameToVideo() {
        Mat frame = new Mat();
        if (videoCapture.read(frame)) {
            videoWriter.write(frame);
        }
    }

    private void stopVideoRecording() {
        videoCapture.release();
        videoWriter.release();
    }
}