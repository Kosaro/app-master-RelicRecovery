package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Hardware.GRAB_BOTTOM_SERVO_RELEASE;
import static org.firstinspires.ftc.teamcode.Hardware.GRAB_TOP_SERVO_GRAB;
import static org.firstinspires.ftc.teamcode.Hardware.GRAB_TOP_SERVO_RELEASE;

/**
 * Created by okosa on 9/9/2017.
 */
//@Disabled
@TeleOp(name = "RelicRecoveryTeleop")
public class RelicRecoveryTeleOp extends OpMode {
    Hardware robot;
    boolean relicMode = false;
    boolean retractLift = false;
    double waitTime;
    Boolean isFlipping;

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
        /**
         telemetry.addData("Left Front Motor", robot.leftFrontMotor.getCurrentPosition());
         telemetry.addData("Right Front Motor", robot.rightFrontMotor.getCurrentPosition());
         telemetry.addData("Left Rear Motor", robot.leftRearMotor.getCurrentPosition());
         telemetry.addData("Right Rear Motor", robot.rightRearMotor.getCurrentPosition());
         telemetry.addData("Lift Motor", robot.liftMotor.getCurrentPosition());
         */
        telemetry.addData("Tilt Relic Arm Position", robot.relicArmTiltServo.getPosition());
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
                robot.setRelicArmExtendMotorPower(-gamepad2.right_stick_y);
                robot.incrementRelicArmTiltPosition(gamepad2.left_stick_y, getRuntime());
                timeLastincremented = getRuntime();
            }
            robot.updateRelicTiltServoPosition();
            if (gamepad2.left_stick_button) {
                robot.relicArmServoTiltPosition = .864;
            }
            if (gamepad2.a && gamepad2.b && gamepad2.y && gamepad2.x) {
                retractLift = true;
            }
            if (retractLift) {
                robot.setTiltServoPositionUp(false);
                if (isFlipping == null && robot.flipServoIsUp) {
                    isFlipping = false;
                } else if(isFlipping == null && !robot.flipServoIsUp){
                    isFlipping = true;
                    waitTime = getRuntime();
                }
                robot.setFlipServoUp(true);
                if (isFlipping) {
                    if (getRuntime() < waitTime + .5) {
                        robot.setLiftPower(1);

                    } else if (getRuntime() < waitTime + 1.5) {
                        robot.setLiftPower(0);
                    } else {
                        isFlipping = false;
                    }
                } else {
                    robot.setLiftPower(-1);
                }
                if (!robot.touchSensorBottom.getState()) {
                    retractLift = false;
                    isFlipping = null;
                    robot.isRelicTiltParallelToGround = false;
                    robot.relicGrabServo.setPosition(Hardware.RELIC_GRAB_SERVO_RELEASE);
                    robot.setTiltServoPositionUp(false);
                }
            }

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
        } else if (gamepad2.dpad_left) {
            robot.grabBottomServo.setPosition(GRAB_BOTTOM_SERVO_RELEASE);
            robot.grabTopServo.setPosition(GRAB_TOP_SERVO_GRAB);
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
