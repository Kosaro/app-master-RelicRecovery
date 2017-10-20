package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by okosa on 9/9/2017.
 */
@Disabled
@TeleOp(name = "RelicRecoveryAutonomous")
public class RelicRecoveryTeleOp extends OpMode {
    Hardware robot;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.setCollectorPower(gamepad1.right_trigger - gamepad1.left_trigger);
        rotate();
        pickUp();
        tilt();
        driveDirection();
    }

    public void driveDirection(){
        if (gamepad1.dpad_up){
            robot.robotOrientationForward = true;
        }else if (gamepad2.dpad_down){
            robot.robotOrientationForward = false;
        }
    }

    public void pickUp() {
        if (gamepad2.a){
            robot.grabServo.setPosition(Hardware.GRAB_SERVO_GRAB);
        }else if (gamepad2.b){
            robot.grabServo.setPosition(Hardware.GRAB_SERVO_RELEASE);
        }
    }

    public void rotate() {
        if (gamepad2.right_bumper){
            robot.flipServo.setPosition(Hardware.FLIP_SERVO_UP);
        }else if (gamepad2.left_bumper){
            robot.flipServo.setPosition(Hardware.FLIP_SERVO_DOWN);
        }
    }

    public void tilt() {
        if (gamepad2.dpad_up){
            robot.tiltServo.setPosition(Hardware.TILT_SERVO_UP);
        }else if (gamepad2.dpad_down){
            robot.tiltServo.setPosition(Hardware.TILT_SERVO_DOWN);
        }
    }



    public void robotOrientation() {
        if (gamepad1.right_bumper) {
            robot.robotOrientationForward = true;

        }
        if (gamepad1.left_bumper) {
            robot.robotOrientationForward = false;
        }
    }

}
