package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motorTester", group="Iterative Opmode")
public class motorTester extends LinearOpMode {
    private DcMotor FL0;
    private DcMotor BL1;
    private DcMotor BR2;
    private DcMotor FR3;

    @Override
    public void runOpMode(){
        //getting hardware names from robot configuration
        FL0 = hardwareMap.get(DcMotor.class, "Front_Left");
        BL1 = hardwareMap.get(DcMotor.class, "Back_Left");
        BR2 = hardwareMap.get(DcMotor.class, "Back_Right");
        FR3 = hardwareMap.get(DcMotor.class, "Front_Right");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            FL0.setPower(1);
            sleep(2000);
            FL0.setPower(0);
            BL1.setPower(1);
            sleep(2000);
            BL1.setPower(0);
            BR2.setPower(1);
            sleep(2000);
            BR2.setPower(0);
            FR3.setPower(1);
            sleep(2000);
            FR3.setPower(0);

            telemetry.addData("Front Left Motor Power", FL0.getPower());
            telemetry.addData("Back Left Motor Power", BL1.getPower());
            telemetry.addData("Back Right Motor Power", BR2.getPower());
            telemetry.addData("Front Right Motor Power", FR3.getPower());
            telemetry.addData("Status", "Running via WiFi");
            telemetry.update();
        }
    }
}

