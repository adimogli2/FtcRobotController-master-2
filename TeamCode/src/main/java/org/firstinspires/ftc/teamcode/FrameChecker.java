import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Frame Checker")
public class FrameChecker extends LinearOpMode {
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
        camera.setPipeline(new FrameCheckerPipeline());

        waitForStart();

        while (opModeIsActive()) {
            // processing is done in the pipeline
        }
    }

    class FrameCheckerPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Size frameSize = input.size();
            int frameChannels = input.channels();

            telemetry.addData("Frame Size", frameSize);
            telemetry.addData("Frame Channels", frameChannels);
            telemetry.update();

            return input;
        }
    }
}
