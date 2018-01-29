package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by okosa on 11/10/2017.
 */
//@Disabled
@TeleOp(name = "Range Sensor Test")
public class RangeSensorTest extends OpMode {
    Hardware robot;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
    }


    @Override
    public void loop() {
        telemetry.addData("Distance", robot.rangeSensor.cmUltrasonic());
        telemetry.addData("CM Optical", robot.rangeSensor.cmOptical()) ;
        telemetry.addData("CM Ultrasonic", robot.rangeSensor.cmUltrasonic());
    }
}
