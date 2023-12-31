package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Traffic Signal Classifier")
public class TrafficSignalClassifier extends LinearOpMode {
    private OpenCvCamera camera;
    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    @Override
    public void runOpMode() {
        FL0 = hardwareMap.get(DcMotor.class, "Front_Left");
        BL1 = hardwareMap.get(DcMotor.class, "Back_Left");
        BR2 = hardwareMap.get(DcMotor.class, "Back_Right");
        FR3 = hardwareMap.get(DcMotor.class, "Front_Right");
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
        camera.setPipeline(new SignalPipeline());

        waitForStart();

        while (opModeIsActive()) {
            // processing is done in the pipeline
        }
    }

    class SignalPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat maskRed = new Mat();
            Mat maskGreen = new Mat();
            Mat maskYellow = new Mat();
            Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(10, 255, 255), maskRed);
            Core.inRange(hsv, new Scalar(38, 70, 50), new Scalar(75, 255, 255), maskGreen);
            Core.inRange(hsv, new Scalar(20, 100, 100), new Scalar(30, 255, 255), maskYellow);
            int redCount = Core.countNonZero(maskRed);
            int greenCount = Core.countNonZero(maskGreen);
            int yellowCount = Core.countNonZero(maskYellow);
            if (redCount > greenCount && redCount > yellowCount) {
                // red signal
                telemetry.addData("Signal", "Red");
                telemetry.update();
                FL0.setPower(0);
                BL1.setPower(0);
                BR2.setPower(0);
                FR3.setPower(0);
            } else if (greenCount > redCount && greenCount > yellowCount) {
                // green signal
                telemetry.addData("Signal", "Green");
                telemetry.update();
                FL0.setPower(0.3);
                BL1.setPower(0.3);
                BR2.setPower(0.3);
                FR3.setPower(0.3);
            } else if (yellowCount > redCount && yellowCount > greenCount) {
                // yellow signal
                telemetry.addData("Signal", "Yellow");
                telemetry.update();
                FL0.setPower(0.1);
                BL1.setPower(0.1);
                BR2.setPower(0.1);
                FR3.setPower(0.1);
            }
            return input;
        }
    }
}

