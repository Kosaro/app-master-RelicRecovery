package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by okosa on 1/31/2018.
 */
@TeleOp(name = "Servo Tester")
public class ServoTester extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("fs");
    }

    double value = .5;
    boolean dpadUpPrev = false;
    boolean dpadDownPrev = false;
    boolean dpadRightPrev = false;
    boolean dpadLeftPrev = false;

    @Override
    public void loop() {
        if (gamepad1.dpad_up != dpadUpPrev) {
            if (gamepad1.dpad_up) {
                value += .1;
            }
            dpadUpPrev = gamepad1.dpad_up;
        }
        if (gamepad1.dpad_down != dpadDownPrev) {
            if (gamepad1.dpad_down) {
                value -= .1;
            }
            dpadDownPrev = gamepad1.dpad_down;
        }
        if (gamepad1.dpad_right != dpadRightPrev) {
            if (gamepad1.dpad_right) {
                value += .005;
            }
            dpadRightPrev = gamepad1.dpad_right;
        }
        if (gamepad1.dpad_left != dpadLeftPrev) {
            if (gamepad1.dpad_left) {
                value -= .005;
            }
            dpadLeftPrev = gamepad1.dpad_left;
        }
        value = Range.clip(value, 0, 1);
        servo.setPosition(value);
        telemetry.addData("Servo Value", value);
    }
}
