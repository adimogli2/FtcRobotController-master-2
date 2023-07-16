package org.firstinspires.ftc.teamcode;

import android.media.Image;
import android.os.Environment;
import android.util.Pair;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="controllerVideoOpMode", group="Iterative Opmode")
public class controllerVideoOpMode extends LinearOpMode {
    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    private OpenCvCamera camera;

    private VideoCapture videoCapture;
    private VideoWriter videoWriter;

    //static Mat frame = new Mat();

    private static final int  CAMERA_FRAME_WIDTH = 640;
    private static final int CAMERA_FRAME_HEIGHT = 480;
    private static final int CAMERA_FRAME_RATE = 30;
    private static final String VIDEO_FILE_EXTENSION = ".jpg";

    private String errMsg;

    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            initializeVideoRecording();
            waitForStart();
            double tgp_y = 0;
            double tgp_x = 0;
            double rot_x = 0;
            double rot_y = 0;
            boolean mac1 = false;

            while (opModeIsActive()) {
                tgp_x = gamepad1.left_stick_x;
                tgp_y = gamepad1.left_stick_y * -1;
                rot_x = gamepad1.right_stick_x;
                rot_y = gamepad1.right_stick_y;
                mac1 = gamepad1.dpad_right;

                double hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
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
                    FL0.setPower(hyp);
                    BL1.setPower(0);
                    BR2.setPower(hyp);
                    FR3.setPower(0);
                }
                //if in second quadrant
                else if(tgp_x < 0 && tgp_y > 0 ){
                    FL0.setPower(0);
                    BL1.setPower(hyp);
                    BR2.setPower(0);
                    FR3.setPower(hyp);
                }
                //if in third quadrant
                else if(tgp_x < 0 && tgp_y < 0 ){
                    FL0.setPower(hyp * -1);
                    BL1.setPower(0);
                    BR2.setPower(hyp * -1);
                    FR3.setPower(0);

                }
                //if in fourth quadrant
                else if(tgp_x > 0 && tgp_y < 0 ) {
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

                // Update motor powers
                //updateMotorPowers(tgp_x, tgp_y, rot_x, rot_y, mac1);

                // Write frame to the video file
                writeFrameToVideo();
            }
        } catch (Exception e) {
            errMsg = e.getMessage();
            telemetry.addData("Error", errMsg);
            telemetry.update();
        } finally {
            telemetry.addData("Error in Final block", errMsg);
            telemetry.update();
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
        videoCapture = new VideoCapture();
        videoCapture.open(0); // 0 represents the default internal camera

        // Configure video capture properties
        videoCapture.set(3, CAMERA_FRAME_WIDTH);
        videoCapture.set(4, CAMERA_FRAME_HEIGHT);
        videoCapture.set(5, CAMERA_FRAME_RATE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        String fileName = String.format("%d_%f_%f_%f_%f%s", System.currentTimeMillis(), FL0.getPower(), BL1.getPower(), BR2.getPower(), FR3.getPower(), VIDEO_FILE_EXTENSION);
        File videoFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), fileName);
        //to save image sequence, use a proper filename; set fourcc=0 or fps=0
        videoWriter = new VideoWriter(videoFile.getAbsolutePath(), 0, 0, new Size(240, 320), true);
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

    /*private void updateMotorPowers(double tgp_x, double tgp_y, double rot_x, double rot_y, boolean mac1) {
        // Update motor powers based on joystick input and other conditions
        // Modify the logic as per your requirements
        FL0.setPower(tgp_y);
        BL1.setPower(tgp_y);
        BR2.setPower(tgp_y);
        FR3.setPower(tgp_y);
    }*/

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

    //frame processing methods
   /* public Pair<List<Mat>, Mat> detectLane(){
        Mat edges = detectEdges(frame);
        List<Mat> lineSegments = new ArrayList<>();
        Mat lineSegmentImage = new Mat();
        List<Mat> laneLines = new ArrayList<>();
        Mat laneLinesImage = new Mat();

        return new Pair<>(laneLines, laneLinesImage);
    }

    public Mat detectEdges(Mat frame) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
        Scalar lowerBlue = new Scalar(30, 40, 0);
        Scalar upperBlue = new Scalar(150, 255, 255);
        Mat mask = new Mat();
        Core.inRange(hsv, lowerBlue, upperBlue, mask);
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges,200,400);

        return edges;
    }

    public static Mat regionOfInterest(Mat canny) {
        int height = canny.rows();
        int width = canny.cols();

        Mat mask = Mat.zeros(canny.size(), CvType.CV_8UC1);

        List <MatOfPoint> vertices = new ArrayList<MatOfPoint>(){{
            add(new MatOfPoint(new Point(0, height * 1 / 2)));
            add(new MatOfPoint(new Point(width, height * 1 / 2)));
            add(new MatOfPoint(new Point(width, height)));
            add(new MatOfPoint(new Point(0, height)));
        }};


        //MatOfPoint[] roi = new MatOfPoint[];
        Imgproc.fillPoly(mask, vertices, new Scalar(255));
        //showImage("mask", mask);

        Mat maskedImage = new Mat();

        Core.bitwise_and(canny, mask, maskedImage);

        return maskedImage;
    }

    public static Mat detectLineSegments(Mat croppedEdges) {
        double rho = 1; // Precision in pixels, i.e., 1 pixel
        double theta = Math.PI / 180; // Angle in radians, i.e., 1 degree
        int minThreshold = 10; // Minimal number of votes
        double minLineLength = 8; // Minimal length of line segment
        double maxLineGap = 4; // Maximum allowed gap between line segments

        MatOfFloat4 lineSegments = new MatOfFloat4();
        Imgproc.HoughLinesP(croppedEdges, lineSegments, rho, theta, minThreshold, minLineLength, maxLineGap);

        if (!lineSegments.empty()) {
            // Process the line segments
            for (int i = 0; i < lineSegments.rows(); i++) {
                double[] lineSegment = lineSegments.get(i, 0);
                System.out.println("Detected line segment:");
                //System.out.println(lineSegmentToString(lineSegment));
            }
        }

        return lineSegments;
    }

    public static List<Mat> averageSlopeIntercept(Mat frame, List<MatOfFloat4> lineSegments) {
        List<Mat> laneLines = new ArrayList<>();

        if (lineSegments == null) {
            System.out.println("No line segments detected");
            return laneLines;
        }

        int height = frame.rows();
        int width = frame.cols();

        List<Scalar> leftFit = new ArrayList<>();
        List<Scalar> rightFit = new ArrayList<>();

        double boundary = 1.0 / 3.0;
        double leftRegionBoundary = width * (1 - boundary); // Left lane line segment should be on the left 2/3 of the screen
        double rightRegionBoundary = width * boundary; // Right lane line segment should be on the right 2/3 of the screen

        for (MatOfFloat4 lineSegment : lineSegments) {
            double[] lineSegmentArray = lineSegment.get(0, 0);
            double x1 = lineSegmentArray[0];
            double y1 = lineSegmentArray[1];
            double x2 = lineSegmentArray[2];
            double y2 = lineSegmentArray[3];

            if (x1 == x2) {
                System.out.println("Skipping vertical line segment (slope=inf): " + lineSegmentArray);
                continue;
            }

            double[] fit = Core.polyfit(new MatOfPoint(new Point(x1, y1), new Point(x2, y2)), 1, 0);
            double slope = fit[0];
            double intercept = fit[1];

            if (slope < 0) {
                if (x1 < leftRegionBoundary && x2 < leftRegionBoundary) {
                    leftFit.add(new Scalar(slope, intercept));
                }
            } else {
                if (x1 > rightRegionBoundary && x2 > rightRegionBoundary) {
                    rightFit.add(new Scalar(slope, intercept));
                }
            }
        }

        Scalar leftFitAverage = averageScalars(leftFit);
        if (!leftFit.isEmpty()) {
            laneLines.add(makePoints(frame, leftFitAverage));
        }

        Scalar rightFitAverage = averageScalars(rightFit);
        if (!rightFit.isEmpty()) {
            laneLines.add(makePoints(frame, rightFitAverage));
        }

        System.out.println("Lane lines: " + laneLines); // Prints the lane lines

        return laneLines;
    }*/

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
//            if (redCount > greenCount && redCount > yellowCount) {
//                // red signal
//                telemetry.addData("Signal", "Red");
//                telemetry.update();
//                FL0.setPower(0);
//                BL1.setPower(0);
//                BR2.setPower(0);
//                FR3.setPower(0);
//            } else if (greenCount > redCount && greenCount > yellowCount) {
//                // green signal
//                telemetry.addData("Signal", "Green");
//                telemetry.update();
//                FL0.setPower(0.3);
//                BL1.setPower(0.3);
//                BR2.setPower(0.3);
//                FR3.setPower(0.3);
//            } else if (yellowCount > redCount && yellowCount > greenCount) {
//                // yellow signal
//                telemetry.addData("Signal", "Yellow");
//                telemetry.update();
//                FL0.setPower(0.1);
//                BL1.setPower(0.1);
//                BR2.setPower(0.1);
//                FR3.setPower(0.1);
//            }
            videoWriter.write(input);
            return input;
        }
    }
}
