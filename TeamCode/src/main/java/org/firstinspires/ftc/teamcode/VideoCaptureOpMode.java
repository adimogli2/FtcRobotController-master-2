package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.doclint.Env;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import android.os.Environment;
import android.Manifest;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "VideoCaptureOpMode")
public class VideoCaptureOpMode extends LinearOpMode {
    private OpenCvCamera camera;

    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    private VideoWriter videoWriter;
    private FileWriter fileWriter;

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

        File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), "video.avi");
        videoWriter = new VideoWriter(videoFile.getAbsolutePath(), VideoWriter.fourcc('M', 'J', 'P', 'G'), 30, new Size(240, 320), true);

        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
        File dataFile = new File(path, "data.csv");
        path.mkdirs();
        try {
            telemetry.addData("dataFile", "dataFile: " + dataFile.toString());
            telemetry.addData("dataDir", "dataDir: " + Environment.DIRECTORY_DOCUMENTS);
            telemetry.update();

            fileWriter = new FileWriter(dataFile);
            fileWriter.write("Timestamp,SteeringAngle\n");
        } catch (IOException e) {

            telemetry.addData("FAIL", "FAIL: " + e.getMessage());
            telemetry.update();
            e.printStackTrace();
        }

        waitForStart();

        while (opModeIsActive()) {
            // processing is done in the pipeline
        }

        videoWriter.release();

        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
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
            //Core.inRange(hsv, new Scalar(38, 70, 50), new Scalar(75, 255, 255), maskGreen);
            if (Core.countNonZero(maskGreen) > 100) {
                // red signal
                telemetry.addData("Signal", "Green: " + Core.countNonZero(maskGreen));
                telemetry.update();
                /*FL0.setPower(0.25);
                BL1.setPower(0.25);
                BR2.setPower(0.25);
                FR3.setPower(0.25);*/
            } else {
                // green signal
                telemetry.addData("Signal", "Red: " + Core.countNonZero(maskGreen));
                telemetry.update();
                /*FL0.setPower(0);
                BL1.setPower(0);
                BR2.setPower(0);
                FR3.setPower(0);*/
            }

            videoWriter.write(input);

            double steeringAngle = getSteeringAngle();
            try {
                fileWriter.write(System.currentTimeMillis() + "," + steeringAngle + "\n");
            } catch (IOException e) {
                e.printStackTrace();
            }

            return input;
        }
    }



    private double getSteeringAngle() {
        // Compute and return the steering angle of your robot here
        // ...
        return 0;
    }
}
