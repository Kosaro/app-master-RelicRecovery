package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by okosa on 9/9/2017.
 */
//@Disabled
@TeleOp(name = "RelicRecoveryTeleop")
public class RelicRecoveryTeleOp extends OpMode {
    Hardware robot;
    boolean relicMode = false;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    boolean previousSelectedValue = false;
    @Override
    public void loop() {
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.setCollectorPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if (!relicMode) {
            robot.setLiftPower(-gamepad2.right_stick_y);
            rotate();
            pickUp();
            tilt();
            driveDirection();
            telemetry.addData("Touch Bottom", robot.touchSensorBottom.getState());
            telemetry.addData("Touch Top", robot.touchSensorTop.getState());
        }else {
            if (gamepad2.x){
                robot.isRelicTiltParallelToGround = true;
            }else if (gamepad2.y){
                robot.isRelicTiltParallelToGround = false;
            }
            if (gamepad2.a){
                robot.relicTiltServo.setPosition(Hardware.RELIC_GRAB_SERVO_GRAB);
            }else if (gamepad2.b){
                robot.relicTiltServo.setPosition(Hardware.RELIC_GRAB_SERVO_RELEASE);
            }
            robot.incrementRelicArmExtendPosition(-gamepad2.left_stick_y, getRuntime());
            robot.incrementRelicArmTiltPosition(-gamepad2.right_stick_y, getRuntime());
        }
        if (gamepad1.left_stick_button || gamepad1.right_stick_button){
            robot.speedMultiplier = .5;
        }else {
            robot.speedMultiplier = 1;
        }

        if (gamepad1.right_bumper){
            robot.leftCollectorMotor.setPower(1);
            robot.rightCollectorMotor.setPower(-1);
        } else if (gamepad1.left_bumper){
            robot.leftCollectorMotor.setPower(-1);
            robot.rightCollectorMotor.setPower(1);
        }
    }
    boolean previousValue = false;
    double v = .9;

    public void driveDirection(){
        if (gamepad1.dpad_up){
            robot.robotOrientationForward = true;
        }else if (gamepad1.dpad_down){
            robot.robotOrientationForward = false;
        }
    }

    public void pickUp() {
        if (gamepad2.a){
            robot.grabBottomServo.setPosition(Hardware.GRAB_BOTTOM_SERVO_GRAB);
            robot.grabTopServo.setPosition(Hardware.GRAB_TOP_SERVO_GRAB);
        }else if (gamepad2.b){
            robot.grabBottomServo.setPosition(Hardware.GRAB_BOTTOM_SERVO_RELEASE);
            robot.grabTopServo.setPosition(Hardware.GRAB_TOP_SERVO_RELEASE);
        }
    }

    public void rotate() {
        if (gamepad2.right_bumper){
            robot.setFlipServoUp(true);
        }else if (gamepad2.left_bumper){
            robot.setFlipServoUp(false);
        }
    }

    public void tilt() {
        if (gamepad2.dpad_up){
            robot.setTiltServoPositionUp(true);
        }else if (gamepad2.dpad_down){
            robot.setTiltServoPositionUp(false);
        }
    }


}
