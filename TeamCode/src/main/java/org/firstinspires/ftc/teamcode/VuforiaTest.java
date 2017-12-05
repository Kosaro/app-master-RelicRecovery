package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by okosa on 11/6/2017.
 */
//@Disabled
@TeleOp(name = "Vuforia Test")
public class VuforiaTest extends LinearOpMode {
    Hardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        robot.initializeVuforia();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Cryptobox Key", robot.getPictograph());
            telemetry.update();
            idle();
        }
    }

}