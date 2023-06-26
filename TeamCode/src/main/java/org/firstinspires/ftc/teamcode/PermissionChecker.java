package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Permission Checker")
public class PermissionChecker extends LinearOpMode {
    @Override
    public void runOpMode() {
        Activity activity = (Activity) hardwareMap.appContext;

        if (activity.checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            // Permission is not granted
            // Request the permission
            activity.requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1);
        }

        waitForStart();

        while (opModeIsActive()) {
            // ...
        }
    }
}
