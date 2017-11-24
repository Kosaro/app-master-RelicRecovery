package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
@TeleOp(name = "Distance Test")
public class DistanceTest extends OpMode{
Hardware robot;
DistanceSensor distanceSensor;
//SensorMRRangeSensor
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        distanceSensor = (DistanceSensor)hardwareMap.i2cDevice.get("ds");
    }



    @Override
    public void loop() {
    telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM ));
    }
}
