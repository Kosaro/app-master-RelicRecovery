package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by okosa on 10/28/2017.
 */
//@Disabled
@Autonomous(name = "Optical Distance Sensor Tester")
public class ColorTester extends OpMode {
    Hardware robot;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, false);
    }

    @Override
    public void loop() {
        if (robot.distanceSensor != null)
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
        if (robot.rangeSensor != null) {
            telemetry.addData("Range", robot.rangeSensor.cmUltrasonic());
            telemetry.addData("Range Adjusted", robot.getRange());
        }
        telemetry.addData("Potentiometer Angle", robot.getPotentiometerAngle());
        if (robot.potentiometer != null) {
            telemetry.addData("Potentiometer Voltage", robot.potentiometer.getVoltage());
        }
    }
}
