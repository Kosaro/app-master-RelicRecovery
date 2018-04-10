package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by okosa on 11/10/2017.
 */
//@Disabled
@TeleOp(name = "Tilt Gyro Calibrate")
public class TiltGyroCalibrate extends OpMode{
    Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Is Calibrating", 0);
    }

    @Override
    public void loop() {
        telemetry.addData("Angle", robot.tiltGyro.getAngularOrientation().firstAngle);
    }
}
