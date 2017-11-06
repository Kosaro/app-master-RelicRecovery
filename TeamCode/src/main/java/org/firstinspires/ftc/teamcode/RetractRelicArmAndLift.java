package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by okosa on 9/29/2017.
 */
//@Disabled
@TeleOp(name = "Retract Relic Arm and Lift")
public class RetractRelicArmAndLift extends OpMode{
Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        robot.relicArmExtendServo.setPosition(Hardware.RELIC_ARM_EXTEND_SERVO_UPPER_LIMIT);
    }

    @Override
    public void loop() {
        if (robot.touchSensorBottom.getState()){
            robot.liftMotor.setPower(-1);
        }
        else {
            robot.liftMotor.setPower(0);
        }
    }
}
