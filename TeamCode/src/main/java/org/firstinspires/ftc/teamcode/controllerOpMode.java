package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="controllerOpMode", group="Iterative Opmode")
public class controllerOpMode extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    // private Servo servoTest;
    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        FL0 = hardwareMap.get(DcMotor.class, "Front_Left");
        BL1 = hardwareMap.get(DcMotor.class, "Back_Left");
        BR2 = hardwareMap.get(DcMotor.class, "Back_Right");
        FR3 = hardwareMap.get(DcMotor.class, "Front_Right");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        double tgp_y = 0;
        double tgp_x = 0;
        double rot_x = 0;
        double rot_y = 0;
        boolean mac1 = false;
        boolean mac2 = false;
        boolean mac3 = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            tgp_x = this.gamepad1.left_stick_x;
            tgp_y = this.gamepad1.left_stick_y * -1;
            rot_x = this.gamepad1.right_stick_x;
            rot_y = this.gamepad1.right_stick_y;
            mac1 = this.gamepad1.dpad_right;
            mac2 = this.gamepad1.dpad_up;
            mac3 = this.gamepad1.dpad_down;
            FL0.setPower(0);
            BL1.setPower(0);
            BR2.setPower(0);
            FR3.setPower(0);

            //double hyp = Math.sqrt(Math.pow(tgp_x, 2) + Math.pow(tgp_y, 2));
            //double hyp = hypt * -1;
            //if on y-axis
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
            else if(tgp_x == 0 && tgp_y > 0){
                FL0.setPower(0.25);
                BL1.setPower(0.25);
                BR2.setPower(0.25);
                FR3.setPower(0.25);
            }
            else if(tgp_x == 0 && tgp_y < 0){
                FL0.setPower(0.25 * -1);
                BL1.setPower(0.25 * -1);
                BR2.setPower(0.25 * -1);
                FR3.setPower(0.25 * -1);
            }
//            if(rot_y > 0){
//                FL0.setPower(0.25);
//                BL1.setPower(0.25);
//                BR2.setPower(0.25);
//                FR3.setPower(0.25);
//            }
//            else if(rot_y < 0){
//                FL0.setPower(0.25 * -1);
//                BL1.setPower(0.25 * -1);
//                BR2.setPower(0.25 * -1);
//                FR3.setPower(0.25 * -1);
//            }
            //if on x-axis (strafing)
            /*else if(tgp_y == 0 ) {
                FL0.setPower(tgp_x);
                BL1.setPower(tgp_x * -1);
                BR2.setPower(tgp_x);
                FR3.setPower(tgp_x * -1);
            }*/

            //turning right or left
            // else if(rot_x >= -1 && rot_x <= 1) {
            //FL0.setPower(tgp_x);
            //BL1.setPower(tgp_x * -1);
            //BR2.setPower(tgp_x);
            //FR3.setPower(tgp_x * -1);
            //}



            //if in first quadrant
            /*if(tgp_x > 0 && tgp_y > 0 ){
                FL0.setPower(hyp);
                BL1.setPower(0);
                BR2.setPower(hyp);
                FR3.setPower(0);
            }*/
            //if in second quadrant
           /* else if(tgp_x < 0 && tgp_y > 0 ){
                FL0.setPower(0);
                BL1.setPower(hyp);
                BR2.setPower(0);
                FR3.setPower(hyp);
            }*/
            //if in third quadrant
            /*else if(tgp_x < 0 && tgp_y < 0 ){
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
            }*/

            else if(mac1 == true) {
                FL0.setPower(0.5);
                BL1.setPower(0.5);
                BR2.setPower(0.5 * -1);
                FR3.setPower(0.5 * -1);
                sleep(2000);
            }

            else if(mac2 == true) {
                FL0.setPower(0.25);
                BL1.setPower(0.25);
                BR2.setPower(0.25);
                FR3.setPower(0.25);
            }

            else if(mac3 == true) {
                FL0.setPower(0);
                BL1.setPower(0);
                BR2.setPower(0);
                FR3.setPower(0);
            }

            //telemetry.addData("Hypot ", hyp);
            telemetry.addData("tgp_x ", tgp_x);
            telemetry.addData("tgp_y ", tgp_y);
            telemetry.addData("Front Left Motor Power", FL0.getPower());
            telemetry.addData("Back Left Motor Power", BL1.getPower());
            telemetry.addData("Back Right Motor Power", BR2.getPower());
            telemetry.addData("Front Right Motor Power", FR3.getPower());
            telemetry.addData("Status", "Running via WiFi");

            telemetry.update();
        }
    }
}