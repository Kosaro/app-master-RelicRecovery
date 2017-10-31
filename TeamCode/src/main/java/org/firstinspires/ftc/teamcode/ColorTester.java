package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by okosa on 10/28/2017.
 */
//@Disabled
@Autonomous(name = "Color Tester")
public class ColorTester extends OpMode {
    Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Blue Value", robot.colorSensor.blue());
        telemetry.addData("Red Value", robot.colorSensor.red());
    }
}
