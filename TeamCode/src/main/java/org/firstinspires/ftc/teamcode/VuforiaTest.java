package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by okosa on 11/6/2017.
 */
//@Disabled
@TeleOp(name = "Vuforia Test")
public class VuforiaTest extends OpMode{
    Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        robot.initializeVuforia();
    }

    @Override
    public void loop() {
        telemetry.addData("Cryptobox Key", robot.getPictograph());
    }
}
