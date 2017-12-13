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
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        super.start();
        robot.relicArmTiltServo.setPosition(Hardware.RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE);
        robot.jewelServo.setPosition(Hardware.JEWEL_SERVO_UP);
    }

    boolean previousSelectedValue = false;
    double timeLastincremented = getRuntime();


    @Override
    public void loop() {
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            robot.speedMultiplier = .5;
        } else {
            robot.speedMultiplier = 1;
        }
        telemetry.addData("Left Front Motor", robot.leftFrontMotor.getCurrentPosition());
        telemetry.addData("Right Front Motor", robot.rightFrontMotor.getCurrentPosition());
        telemetry.addData("Left Rear Motor", robot.leftRearMotor.getCurrentPosition());
        telemetry.addData("Right Rear Motor", robot.rightRearMotor.getCurrentPosition());
        telemetry.addData("Lift Motor", robot.liftMotor.getCurrentPosition());
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.setCollectorPower(gamepad1.right_trigger - gamepad1.left_trigger);

        driveDirection();
        /*(
        if (gamepad1.right_bumper) {
            robot.leftCollectorMotor.setPower(1);
            robot.rightCollectorMotor.setPower(-1);
        } else if (gamepad1.left_bumper) {
            robot.leftCollectorMotor.setPower(-1);
            robot.rightCollectorMotor.setPower(1);
        }
        */

        if (gamepad2.back != previousSelectedValue) {
            if (gamepad2.back) {
                relicMode = !relicMode;
            }
            previousSelectedValue = gamepad2.back;
        }
        if (!relicMode) {
            robot.setLiftPower(-gamepad2.right_stick_y);
            flip();
            pickUp();
            tilt();
            //telemetry.addData("Touch Bottom", robot.touchSensorBottom.getState());
            //telemetry.addData("Touch Top", robot.touchSensorTop.getState());
            //telemetry.addData("p", robot.flipServo.getPosition());
            //telemetry.addData("t", robot.tiltServo.getPosition());
            //telemetry.addData("constant", Hardware.TILT_SERVO_UP);
        } else {
            if (gamepad2.y) {
                robot.isRelicTiltParallelToGround = true;
            } else if (gamepad2.x) {
                robot.isRelicTiltParallelToGround = false;
            }
            if (gamepad2.a) {
                robot.relicGrabServo.setPosition(Hardware.RELIC_GRAB_SERVO_GRAB);
            } else if (gamepad2.b) {
                robot.relicGrabServo.setPosition(Hardware.RELIC_GRAB_SERVO_RELEASE);
            }
            if (getRuntime() - .1 > timeLastincremented) {
                robot.incrementRelicArmExtendPosition(gamepad2.right_stick_y, getRuntime());
                robot.incrementRelicArmTiltPosition(gamepad2.left_stick_y, getRuntime());
                timeLastincremented = getRuntime();


            }
            robot.updateRelicTiltServoPosition();

        }
    }

    public void driveDirection() {
        if (gamepad1.dpad_up) {
            robot.robotOrientationForward = true;
        } else if (gamepad1.dpad_down) {
            robot.robotOrientationForward = false;
        }
    }

    public void pickUp() {
        if (gamepad2.a) {
            robot.setGrabbersClosed(true);
        } else if (gamepad2.b) {
            robot.setGrabbersClosed(false);
        }
    }

    public void flip() {
        if (gamepad2.left_bumper) {
            robot.setFlipServoUp(true);
        } else if (gamepad2.right_bumper) {
            robot.setFlipServoUp(false);
        }
    }

    public void tilt() {
        if (gamepad2.dpad_up) {
            robot.setTiltServoPositionUp(true);
        } else if (gamepad2.dpad_down) {
            robot.setTiltServoPositionUp(false);
        }
    }


}
