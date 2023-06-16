package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "Signal Classifier")
public class SignalClassifier extends LinearOpMode {
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
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
            Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(10, 255, 255), maskRed);
            Core.inRange(hsv, new Scalar(38, 70, 50), new Scalar(75, 255, 255), maskGreen);
            if (Core.countNonZero(maskRed) > Core.countNonZero(maskGreen)) {
                // red signal
                telemetry.addData("Signal", "Red");
                telemetry.update();
            } else {
                // green signal
                telemetry.addData("Signal", "Green");
                telemetry.update();
            }
            return input;
        }
    }
}
