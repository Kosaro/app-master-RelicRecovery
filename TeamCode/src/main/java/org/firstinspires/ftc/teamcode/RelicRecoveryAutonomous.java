package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.Hardware.RELIC_ARM_TILT_SERVO_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.Hardware.RELIC_GRAB_SERVO_GRAB;

/**
 * Created by okosa on 9/9/2017.
 */
@Disabled
@Autonomous(name = "RelicRecoveryAutonomous")
public abstract class RelicRecoveryAutonomous extends LinearOpMode {
    //constants
    private final static int DISTANCE_FAR = 3000;
    private final static int DISTANCE_CENTER = 2500;
    private final static int DISTANCE_CLOSE = 2000;

    //Local Variables
    Hardware robot;
    RelicRecoveryVuMark cryptoboxKey;  //UNKNOWN, LEFT, CENTER, RIGHT

    int directionMultiplier;

    @Override
    public void runOpMode() throws InterruptedException {

        if (getDesiredColor() == Hardware.ColorDetected.RED)
            directionMultiplier = 1;
        else
            directionMultiplier = -1;
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot = new Hardware(hardwareMap);
        //robot.initializeVuforia();
        robot.setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.relicArmTiltServo.setPosition(RELIC_ARM_TILT_SERVO_LOWER_LIMIT);
        robot.setGrabbersClosed(true);

        //telemetry.addData("Status", "Resetting lift");
        //telemetry.update();
        //calibrateGyroAndInitializeVuforia();
        /**
        boolean neededToResetLift = !robot.touchSensorBottom.getState();
        while (!robot.touchSensorBottom.getState()){
            robot.setTiltServoPositionUp(true);
            robot.setLiftPower(-1);
        }
        if (neededToResetLift){
            robot.setTiltServoPositionUp(false);
            sleep(2000);
        }

         */
        telemetry.addData("Status", "Calibrating Gyro and Initializing Vuforia");
        telemetry.update();
        calibrateGyroAndInitializeVuforia();

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();/**
         cryptoboxKey = RelicRecoveryVuMark.CENTER;
         while (!isStarted()) {
         if (gamepad1.x)
         cryptoboxKey = RelicRecoveryVuMark.LEFT;
         else if (gamepad1.a)
         cryptoboxKey = RelicRecoveryVuMark.CENTER;
         else if (gamepad1.b)
         cryptoboxKey = RelicRecoveryVuMark.RIGHT;
         telemetry.addData("Key", cryptoboxKey);
         telemetry.update();
         idle();
         }
         */

        waitForStart();

        //robot.relicArmTiltServo.setPosition(Hardware.RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE);
        robot.relicGrabServo.setPosition(RELIC_GRAB_SERVO_GRAB);

        telemetry.addData("Status", "Remove Jewel");
        telemetry.update();
        removeJewel();


        telemetry.addData("Status", "Drive to Cryptobox");
        telemetry.update();
        driveToCryptobox();

        //sleep(1000);

        telemetry.addData("Status", "Turn to Cryptobox");
        telemetry.update();
        turnToCryptobox();

        //sleep(1000);

        telemetry.addData("Status", "Line Up With CryptoBox Key");
        telemetry.update();
        //lineUpWithCryptoboxKey();

        telemetry.addData("Status", "Place Glyph in Cryptobox");
        telemetry.update();
        driveUpToCryptobox();

        //sleep(1000);

        telemetry.addData("Status", "Align With Column");
        telemetry.update();
        alignWithColumnAndPlaceInCryptobox();

        //telemetry.addData("Status", "Saving Cipher to File");
        //telemetry.update();
        //robot.saveActiveCipherToFile();


        telemetry.addData("Status", "Done");
        telemetry.update();

        sleep(2000);

    }


    private void decodePictograph() {
        double scanDuration = 3;    //second(s)
        double startTime = getRuntime();
        do {
            cryptoboxKey = robot.getPictograph();
            telemetry.addData("Cryptobox Key", cryptoboxKey);
            telemetry.update();
            idle();
        }
        while (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN && getRuntime() < startTime + scanDuration && opModeIsActive());
    }


    private void removeJewel() {
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_DOWN);
        sleep(1000);
        decodePictograph();
        Hardware.ColorDetected desiredColor;
        if (goForRankingPoints()) {
            if (getDesiredColor() == Hardware.ColorDetected.BLUE)
                desiredColor = Hardware.ColorDetected.RED;
            else
                desiredColor = Hardware.ColorDetected.BLUE;

        } else {
            desiredColor = getDesiredColor();
        }
        if (getDesiredColor() == Hardware.ColorDetected.RED) {
            if (robot.getColorDetected() == desiredColor) {
                driveFor(-100, .3);
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
                driveFor(200, .3);
            } else if (robot.getColorDetected() != Hardware.ColorDetected.NONE) {
                driveFor(100, .3);
            } else {
                driveFor(100, .3);// do nothing OR random!!!!! rawr xD lolzor;
            }
        } else {
            if (robot.getColorDetected() == desiredColor) {
                driveFor(-100, .3);

            } else if (robot.getColorDetected() != Hardware.ColorDetected.NONE) {
                driveFor(100, .3);
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
                driveFor(-200, .3);
            } else {
                driveFor(-100, .3);// do nothing OR random!!!!! rawr xD lolzor;
            }
        }

        robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
/**
 while ((robot.getAngle() > 2 || robot.getAngle() < -2) && opModeIsActive()) {
 robot.drive(0, 0, robot.turn(0));
 telemetry.addData("Angle", robot.getAngle());
 telemetry.addData("Color", robot.getColorDetected());
 telemetry.update();
 idle();
 }
 */
        robot.drive(0, 0, 0);

    }

    private void driveToCryptobox() {
        /**
         int finalPosition = 0;
         if (getDesiredColor() == Hardware.ColorDetected.RED) {
         if (cryptoboxKey == RelicRecoveryVuMark.RIGHT)
         finalPosition = DISTANCE_CLOSE;
         else if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
         finalPosition = DISTANCE_FAR;
         else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
         finalPosition = DISTANCE_CENTER;
         else if (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN)
         finalPosition = DISTANCE_CLOSE;
         } else {
         if (cryptoboxKey == RelicRecoveryVuMark.RIGHT)
         finalPosition = DISTANCE_FAR;
         else if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
         finalPosition = DISTANCE_CLOSE;
         else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
         finalPosition = DISTANCE_CENTER;
         else if (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN)
         finalPosition = DISTANCE_CLOSE;
         }
         robot.drive(1 * directionMultiplier, 0, 0);
         while (Math.abs(robot.leftFrontMotor.getCurrentPosition()) < finalPosition && opModeIsActive()) {
         idle();
         }
         robot.stop();
         */
        if (!otherBalancingBoard()) {
            if (getDesiredColor() == Hardware.ColorDetected.RED) {
                if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
                    driveFor(1800 * directionMultiplier, .5, 5, 0.0);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(1700 * directionMultiplier, .5, 5, 0.0);
                else
                    driveFor(1250 * directionMultiplier, .5, 5, 0.0);
                driveFor(400 * directionMultiplier, .2, 5, 0.0);
            } else {
                if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
                    driveFor(1250 * directionMultiplier, .5, 5, 0.0);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(1350 * directionMultiplier, .5, 5, 0.0);
                else
                    driveFor(1800 * directionMultiplier, .5, 5, 0.0);
                driveFor(400 * directionMultiplier, .2, 5, 0.0);
            }
        } else {
            driveFor(900 * directionMultiplier, .5, 5, 0.0);
            driveFor(500 * directionMultiplier, .2, 5, 0.0);
            double angle;
            if (getDesiredColor() == Hardware.ColorDetected.RED) {
                angle = -90;
            } else {
                angle = 90;
            }
            while ((robot.getAngle() < angle - 2 || robot.getAngle() > angle + 2) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(angle));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.addData("CryptoKey", cryptoboxKey);
                telemetry.update();
                idle();
            }
            sleep(200);
            while ((robot.getAngle() < angle - 2 || robot.getAngle() > angle + 2) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(angle));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.addData("CryptoKey", cryptoboxKey);
                telemetry.update();
                idle();
            }
            if (getDesiredColor() == Hardware.ColorDetected.RED) {
                if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
                    driveFor(900 * directionMultiplier, .2, 5, angle);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(800 * directionMultiplier, .2, 5, angle);
                else
                    driveFor(400 * directionMultiplier, .2, 5, angle);
            } else {
                if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
                    driveFor(400 * directionMultiplier, .2, 5, angle);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(500 * directionMultiplier, .2, 5, angle);
                else
                    driveFor(850 * directionMultiplier, .2, 5, angle);

            }

        }
        robot.drive(0, 0, 0);
    }

    double alignmentAngle;

    private void turnToCryptobox() {
        if (!otherBalancingBoard()) {
            alignmentAngle = -90;
        } else {
            if (getDesiredColor() == Hardware.ColorDetected.BLUE)
                alignmentAngle = 0;
            else {
                alignmentAngle = 179.99;
            }
        }
        while ((robot.getAngle() < alignmentAngle - 2 || robot.getAngle() > alignmentAngle + 2) && opModeIsActive()) {
            robot.drive(0, 0, robot.turn(alignmentAngle));
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
            idle();
        }
        sleep(200);
        while ((robot.getAngle() < alignmentAngle - 2 || robot.getAngle() > alignmentAngle + 2) && opModeIsActive()) {
            robot.drive(0, 0, robot.turn(alignmentAngle));
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    private void lineUpWithCryptoboxKey() {
    }

    private void driveUpToCryptobox() {
        //robot.setTiltServoPositionUp(true);
        // sleep(1500);
        /**
         driveFor(-300, .25, 3);
         robot.setGrabbersClosed(false);
         robot.setTiltServoPositionUp(false);
         driveFor(100, .25, 2);
         */
        driveFor(-450, .5, 2);
        driveFor(-250, .2, 2);
        if (cryptoboxKey != RelicRecoveryVuMark.LEFT)
            driveFor(50, .5, 2);
        else {
            driveFor(38, .5, 2);
            while ((robot.getAngle() < alignmentAngle - 2 || robot.getAngle() > alignmentAngle + 0) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(alignmentAngle - 1));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.update();
                idle();
            }
            robot.drive(0, 0, 0);
        }
        /** //drive sideways
         startingValue = robot.rightFrontMotor.getCurrentPosition();
         startTime = getRuntime();
         int target = -800;
         while (Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue) < Math.abs(target) && opModeIsActive() && getRuntime() < startTime + 2.5) {
         double rotationValue = 0;

         rotationValue = robot.turn(0);

         int direction;
         if (target > 0) {
         direction = 1;
         } else {
         direction = -1;
         }
         robot.drive(0, direction * .3, 0);
         telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
         telemetry.update();
         idle();
         }
         */
        //robot.turn(45);
        //driveFor(100, .5, 2);
        robot.drive(0, 0, 0);
    }

    void alignWithColumnAndPlaceInCryptobox() {
        sleep(500);
        robot.setTiltGyroOffset();
        startingValue = robot.rightFrontMotor.getCurrentPosition();


        int sideDirectionMultiplier;
        if (cryptoboxKey == RelicRecoveryVuMark.LEFT) {
            sideDirectionMultiplier = -1;
        } else {
            sideDirectionMultiplier = 1;
        }

        startTime = getRuntime();
        int target = -500 * sideDirectionMultiplier;
        double startHeading = robot.getAngle();
        while (robot.rangeSensor.getDistance(DistanceUnit.CM) > 8 && Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue) < Math.abs(target) && opModeIsActive() && getRuntime() < startTime + 5) {
            double rotationValue = 0;

            rotationValue = robot.turn(alignmentAngle);

            int direction;
            if (target > 0) {
                direction = 1;
            } else {
                direction = -1;
            }
            robot.drive(0, direction * .2, rotationValue);
            telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
            telemetry.addData("CM Optical", robot.rangeSensor.cmOptical());
            telemetry.addData("Ultrasonic CM", robot.rangeSensor.cmUltrasonic());
            telemetry.update();
            idle();
        }
        if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
            driveFor(70, .5, 2);
        else
            driveFor(140, .5, 2);
        Double failedAngle = null;
        while (!attemptToLowerGlyph() && opModeIsActive()) {
            if (failedAngle == null) {
                failedAngle = robot.getTiltGyroAngle();
            } else if (robot.getTiltGyroAngle() > failedAngle + 15) {
                sideDirectionMultiplier = -sideDirectionMultiplier;
                failedAngle = robot.getTiltGyroAngle();
            } else if (robot.getTiltGyroAngle() < failedAngle - 15) {
                break;
            } else {
                failedAngle = robot.getTiltGyroAngle();
            }
            robot.setTiltServoPositionUp(false);
            sleep(500);
            startingValue = robot.rightFrontMotor.getCurrentPosition();
            startTime = getRuntime();
            int targetS = 70 * sideDirectionMultiplier;
            while (Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue) < Math.abs(targetS) && opModeIsActive() && getRuntime() < startTime + 2) {
                double rotationValue = 0;

                rotationValue = robot.turn(alignmentAngle);

                int direction;
                if (targetS > 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                robot.drive(0, direction * .3, rotationValue);
                telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
                telemetry.update();
                idle();
            }
            robot.drive(0, 0, 0);
        }
        robot.setGrabbersClosed(false);
        sleep(500);
        driveFor(100, .2, 2);
        sleep(500);
        driveFor(-170, .2, 2);
        driveFor(290, .2, 2);
        robot.setTiltServoPositionUp(false);
    }


    int startingValue = 0;

    void driveFor(int x, double speed) {
        driveFor(x, speed, 10);
    }

    void driveFor(int x, double speed, double time) {
        driveFor(x, speed, time, null);
    }

    double startTime;

    boolean attemptToLowerGlyph() {
        robot.setTiltServoPositionUp(true);
        int degreesPerSecond = 8;
        boolean isMoving = true;
        double lastTime = getRuntime();
        double lastAngle = robot.getTiltGyroAngle();
        sleep(300);
        while (opModeIsActive() && isMoving) {
            double currentTime = getRuntime();
            double currentAngle = robot.getTiltGyroAngle();
            if (Math.abs(currentTime - lastTime) > .25) {
                if (Math.abs(currentAngle - lastAngle) / Math.abs(currentTime - lastTime) < degreesPerSecond) {
                    isMoving = false;
                }
                lastAngle = currentAngle;
                lastTime = currentTime;
            }
            telemetry.addData("Tilt Gyro Angle", currentAngle);
            telemetry.update();
            idle();
        }
        if (robot.getTiltGyroAngle() < 290) {
            return true;
        } else {
            return false;
        }
    }


    void driveFor(int encoderTicks, double speed, double time, Double angle) {
        startingValue = robot.rightFrontMotor.getCurrentPosition();
        startTime = getRuntime();
        double direction;
        int target = encoderTicks;
        if (target > 0) {
            direction = 1;
        } else {
            direction = -1;
        }

        while (Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue) < Math.abs(target) && opModeIsActive() && getRuntime() < startTime + time) {
            double rotationValue = 0;
            if (angle != null) {
                rotationValue = robot.turn(angle);
            }
            robot.drive(direction * speed, 0, rotationValue * 2);
            telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    void calibrateGyroAndInitializeVuforia() {
        robot.tiltGyro.calibrate();
        robot.initializeVuforia();
        while (opModeIsActive() && robot.tiltGyro.isCalibrating()) {
            idle();
        }
    }


    abstract Hardware.ColorDetected getDesiredColor();

    abstract boolean otherBalancingBoard();

    abstract boolean goForRankingPoints();

}
