package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.UNKNOWN;
import static org.firstinspires.ftc.teamcode.Hardware.ColorDetected.BLUE;
import static org.firstinspires.ftc.teamcode.Hardware.ColorDetected.RED;
import static org.firstinspires.ftc.teamcode.Hardware.GRAB_BOTTOM_SERVO_RELEASE;
import static org.firstinspires.ftc.teamcode.Hardware.GRAB_TOP_SERVO_GRAB;
import static org.firstinspires.ftc.teamcode.Hardware.RELIC_ARM_TILT_SERVO_LOWER_LIMIT;

/**
 * Created by okosa on 9/9/2017.
 */
@Disabled
@Autonomous(name = "RelicRecoveryAutonomous")
public abstract class RelicRecoveryAutonomous extends LinearOpMode {

    //Local Variables
    Hardware robot;
    RelicRecoveryVuMark cryptoboxKey;  //UNKNOWN, LEFT, CENTER, RIGHT

    int directionMultiplier;
    double opModeStartTime;

    @Override
    public void runOpMode() throws InterruptedException {

        if (getDesiredColor() == RED)
            directionMultiplier = 1;
        else
            directionMultiplier = -1;
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot = new Hardware(hardwareMap);
        robot.setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.relicArmTiltServo.setPosition(RELIC_ARM_TILT_SERVO_LOWER_LIMIT);
        robot.setAlignmentServoUp(true);
        robot.setGrabbersClosed(false);

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

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Status", "Waiting For Start");
            telemetry.addData("CryptoKey", robot.getPictograph());
            telemetry.addData("Tilt Gyro", robot.getTiltGyroAngle());
            telemetry.update();
            idle();
        }
        opModeStartTime = getRuntime();

        //robot.relicArmTiltServo.setPosition(Hardware.RELIC_ARM_TILT_SERVO_0_DEGREE_VALUE);
        //robot.relicGrabServo.setPosition(RELIC_GRAB_SERVO_FULL_CLOSE);
        robot.setGrabbersClosed(true);

        telemetry.addData("Status", "Remove Jewel");
        telemetry.update();
        removeJewel();

        telemetry.addData("Status", "Drive to Cryptobox");
        telemetry.update();
        driveToCryptobox();

        telemetry.addData("Status", "Turn to Cryptobox");
        telemetry.update();
        turnToCryptobox();


        sleep(500);

        telemetry.addData("Status", "Drive up to Cryptobox");
        telemetry.update();
        driveUpToCryptobox();

        telemetry.addData("Status", "Place Glyph in Cryptobox");
        telemetry.update();
        placeInCryptobox();


        telemetry.addData("Status", "Collect Glyphs");
        telemetry.update();

        //collectGlyphs();

        telemetry.addData("Status", "Done");
        telemetry.update();

        sleep(1000);
    }


    private void decodePictograph() {
        double scanDuration = 2;    //second(s)
        double startTime = getRuntime();
        do {
            cryptoboxKey = robot.getPictograph();
            telemetry.addData("Cryptobox Key", cryptoboxKey);
            telemetry.update();
            if (cryptoboxKey == UNKNOWN)
                idle();
        }
        while (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN && getRuntime() < startTime + scanDuration && opModeIsActive());
    }


    private void removeJewel() {
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_DOWN);
        double lowerTime = getRuntime();
        decodePictograph();
        while (getRuntime() < lowerTime + .75 && opModeIsActive()) {
            idle();
        }
        Hardware.ColorDetected desiredColor;
        if (goForRankingPoints()) {
            if (getDesiredColor() == Hardware.ColorDetected.BLUE)
                desiredColor = RED;
            else
                desiredColor = Hardware.ColorDetected.BLUE;

        } else {
            desiredColor = getDesiredColor();
        }
        if (getDesiredColor() == RED) {
            if (robot.getColorDetected() == desiredColor) {
                driveFor(-100, .3);
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
                driveFor(200, .3);
            } else if (robot.getColorDetected() != Hardware.ColorDetected.NONE) {
                driveFor(100, .3);
            } else {
                driveFor(100, .3);// do nothing OR random!!!!! rawr xD lolzor
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
        robot.drive(0, 0, 0);
    }

    private void driveToCryptobox() {
        if (!otherBalancingBoard()) {
            if (getDesiredColor() == RED) {
                if (cryptoboxKey == LEFT)
                    driveFor(2330 * directionMultiplier, 1, 3, 0.0);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(1810 * directionMultiplier, 1, 3, 0.0);
                else
                    driveFor(1400 * directionMultiplier, 1, 3, 0.0);
            } else {
                if (cryptoboxKey == LEFT)
                    driveFor(1050 * directionMultiplier, 1, 3, 0.0);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(1470 * directionMultiplier, 1, 3, 0.0);
                else
                    driveFor(2000 * directionMultiplier, 1, 3, 0.0);
            }
        } else {
            driveFor(1150 * directionMultiplier, 1, 5, 0.0);
            if (getDesiredColor() == RED) {
                if (cryptoboxKey == LEFT) {
                    driveFor(1650, 0, -1, 5, 0.0);
                } else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
                    driveFor(-1100, 0, -1, 4, 0.0);
                else {
                    driveFor(-600, 0, -1, 3, 0.0);
                }
            } else {
                alignmentAngle = 0;
                if (cryptoboxKey == LEFT)
                    driveFor(-50 * directionMultiplier, .5, 5, 0.0);
                else if (cryptoboxKey == RelicRecoveryVuMark.CENTER) {
                    startingValue = robot.rightFrontMotor.getCurrentPosition();
                    startingValue2 = robot.rightRearMotor.getCurrentPosition();
                    startingValue3 = robot.leftFrontMotor.getCurrentPosition();
                    startingValue4 = robot.leftRearMotor.getCurrentPosition();
                    startTime = getRuntime();
                    int targetS = -500;
                    while (Math.abs(getAverageEncoderValue()) < Math.abs(targetS) && opModeIsActive() && getRuntime() < startTime + 4) {
                        double rotationValue = 0;
                        rotationValue = robot.turn(alignmentAngle);
                        int direction;
                        if (targetS > 0) {
                            direction = 1;
                        } else {
                            direction = -1;
                        }
                        robot.drive(0, direction * 1, rotationValue);
                        telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
                        telemetry.update();
                        idle();
                    }
                    robot.drive(0, 0, 0);
                } else
                    driveFor(-1000 * directionMultiplier, 0, -1, 5, 0.0);
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
        if (otherBalancingBoard() && getDesiredColor() == RED){
            robot.setAlignmentServoUp(false);
            while ((robot.getAngle() < alignmentAngle - 3 || robot.getAngle() > alignmentAngle + 3) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(alignmentAngle));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.update();
                idle();
            }
        }else {
            while ((robot.getAngle() < alignmentAngle - 3 || robot.getAngle() > alignmentAngle + 3) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(alignmentAngle));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.update();
                idle();
            }
            robot.setAlignmentServoUp(false);
            sleep(100);
            while ((robot.getAngle() < alignmentAngle - 3 || robot.getAngle() > alignmentAngle + 3) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(alignmentAngle));
                telemetry.addData("Angle", robot.getAngle());
                telemetry.update();
                idle();
            }
        }
        robot.drive(0, 0, 0);
    }


    private void driveUpToCryptobox() {
        robot.setAlignmentServoUp(false);
        driveFor(-55, .5, 1);
        while (robot.getRange() > 20 && opModeIsActive()) {
            robot.drive(-.5, 0, robot.turn(alignmentAngle));
            telemetry.addData("Ultrasonic Distance", robot.getRange());
            telemetry.update();
            idle();
        }
        while (robot.getRange() > 11 && opModeIsActive()) {
            robot.drive(-.2, 0, robot.turn(alignmentAngle));
            telemetry.addData("Ultrasonic Distance", robot.getRange());
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    void alignWithColumn() {
        boolean canContinue = false;
        do {
            double sideValue = -.1;
            if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 20 || Double.isNaN(robot.distanceSensor.getDistance(DistanceUnit.CM))) {
                sideValue = -.15;
            } else if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 15) {
                sideValue = -.15;
            } else if (robot.distanceSensor.getDistance(DistanceUnit.CM) >= 9) {
                sideValue = -.1;
            } else if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 5.5) {
                sideValue = -.1;
            }
            if (robot.getPotentiometerAngle() > 35) {
                sideValue = .1;
            } else if (robot.getPotentiometerAngle() > 30) {
                sideValue = .1;
            } else if (robot.getPotentiometerAngle() > 20) {
                sideValue = .075;
            } else if (robot.getPotentiometerAngle() > 15) {
                sideValue = .052;
            } else if (robot.getPotentiometerAngle() > 3 && robot.getPotentiometerAngle() < 4) {
                sideValue = -.052;
            } else if (robot.getPotentiometerAngle() > 1 && robot.getPotentiometerAngle() <= 3) {
                sideValue = -.06;
            } else if (robot.getPotentiometerAngle() <= 15 && robot.getPotentiometerAngle() >= 4) {
                sideValue = 0;
            }
            double forwardValue = 0;
            if (robot.getRange() > 13) {
                forwardValue = -.05;
                if (robot.getRange() > 16)
                    forwardValue = -.15;
            } else if (robot.getRange() < 10) {
                forwardValue = .03;
                if (robot.getRange() < 8)
                    forwardValue = .05;
                if (robot.getRange() < 5)
                    forwardValue = .1;
            }
            double turnValue = robot.turn(alignmentAngle);
            if (robot.getRange() > 9 && robot.getRange() < 16 && (robot.getPotentiometerAngle() <= 15 && robot.getPotentiometerAngle() >= 4) && Math.abs(turnValue) <= .1) {
                canContinue = true;
                robot.drive(0, 0, 0);
            } else {
                if (robot.getPotentiometerAngle() > 40) {
                    forwardValue = .2;
                }
                if (forwardValue == .2 || robot.getRange() > 15) {
                    sideValue = 0;
                }
                if (Math.abs(turnValue) < .1) {
                    turnValue /= 3;
                }
                if (robot.getPotentiometerAngle() > 40) {
                    sideValue = .2;
                }
                robot.drive(forwardValue, sideValue, turnValue);
            }
            telemetry.addData("Light Sensor", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Potentiometer Angle", robot.getPotentiometerAngle());
            telemetry.addData("Range Sensor", robot.getRange());
            telemetry.update();
            idle();
        } while (!canContinue && opModeIsActive() && getTimeLeft() > 4);
        robot.drive(0, 0, 0);
    }

    void placeInCryptobox() {
        alignWithColumn();
        int numberOfAttempts = 0;
        boolean changedSideDirectionMultiplier = false;

        //sleep(100);
        robot.setTiltGyroOffset();
        int sideDirectionMultiplier;
        if (cryptoboxKey == LEFT) {
            sideDirectionMultiplier = -1;
        } else {
            sideDirectionMultiplier = 1;
        }

        Double failedAngle = null;
        while (!attemptToLowerGlyph() && opModeIsActive() && getTimeLeft() > 4) {
            numberOfAttempts++;
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
            /**
             startingValue = robot.rightFrontMotor.getCurrentPosition();
             startingValue2 = robot.rightRearMotor.getCurrentPosition();
             startingValue3 = robot.leftFrontMotor.getCurrentPosition();
             startingValue4 = robot.leftRearMotor.getCurrentPosition();
             startTime = getRuntime();
             int targetS = 60;
             if (!changedSideDirectionMultiplier) {
             {
             if (numberOfAttempts == 3) {
             sideDirectionMultiplier = -sideDirectionMultiplier;
             targetS *= 3;
             }
             }
             }

             targetS = targetS * sideDirectionMultiplier;
             while (Math.abs(getAverageEncoderValue()) < Math.abs(targetS) && opModeIsActive() && getRuntime() < startTime + 2) {
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
             */
            alignWithColumn();
        }
        robot.setAlignmentServoUp(true);
        if (getTimeLeft() > 15 && !secondGlyph) {
            robot.setTiltServoPositionUp(true);
            robot.setGrabbersClosed(false);
            sleep(500);
            robot.setTiltServoPositionUp(false);
            driveFor(150, .1, 2);
            collectGlyphs();
        } else if (getTimeLeft() > 4) {
            robot.setGrabbersClosed(false);
            sleep(500);
            driveFor(100, .2, 2);
            sleep(500);
            driveFor(-170, .2, 2);
            driveFor(290, .2, 2);
            robot.setTiltServoPositionUp(false);
        } else if (getTimeLeft() < 1.5) {
            robot.setGrabbersClosed(false);
            driveFor(200, .5, 2);
            robot.setTiltServoPositionUp(false);
        } else {
            robot.setTiltServoPositionUp(true);
            robot.setGrabbersClosed(false);
            sleep(500);
            driveFor(290, .2, 2);
            robot.setTiltServoPositionUp(false);
        }
    }


    int startingValue = 0;
    int startingValue2 = 0;
    int startingValue3 = 0;
    int startingValue4 = 0;

    void driveFor(int x, double speed) {
        driveFor(x, speed, 10);
    }

    void driveFor(int x, double speed, double time) {
        driveFor(x, speed, time, null);
    }

    double startTime;

    boolean attemptToLowerGlyph() {
        robot.setTiltServoPositionUp(true);
        if (secondGlyph && ((cryptoboxKey == LEFT && getDesiredColor() == RED) || (getDesiredColor() == BLUE))) {
            robot.grabBottomServo.setPosition(GRAB_BOTTOM_SERVO_RELEASE);
            robot.grabTopServo.setPosition(GRAB_TOP_SERVO_GRAB);
        }
        int degreesPerSecond = 8;
        boolean isMoving = true;
        double lastTime = getRuntime();
        double lastAngle = robot.getTiltGyroAngle();
        sleep(300);
        double startTime = getRuntime();
        while (opModeIsActive() && (isMoving && robot.getTiltGyroAngle() > -80 && getRuntime() < startTime + 3)) {
            double currentTime = getRuntime();
            double currentAngle = robot.getTiltGyroAngle();
            if (Math.abs(currentTime - lastTime) > .15) {
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


        if (robot.getTiltGyroAngle() < -70) {
            return true;
        } else {
            double failedAngle = robot.getTiltGyroAngle();
            int direction = -1;
            int position = robot.leftFrontMotor.getCurrentPosition();
            if (robot.getPotentiometerAngle() > 8)
                direction = 1;
            while (robot.getTiltGyroAngle() > failedAngle - 15 && Math.abs(robot.leftFrontMotor.getCurrentPosition() - position) < 150 && opModeIsActive()) {
                if ((robot.getTiltGyroAngle() > failedAngle - 7))
                    robot.drive(0, .1 * direction, 0);
                else
                    robot.drive(0, .05 * direction, 0);
                idle();
            }
            robot.drive(0, 0, 0);
            if (robot.getTiltGyroAngle() <= failedAngle - 15) {
                startTime = getRuntime();
                while (robot.getTiltGyroAngle() > -80 && Math.abs(startTime - getRuntime()) < 2 && opModeIsActive()) {
                    idle();
                }
                return true;
            } else
                return false;
        }
    }

    boolean secondGlyph = false;

    void collectGlyphs() {

        secondGlyph = true;
        robot.setGrabbersClosed(false);
        robot.setSpearServoUp(false);
        if (!otherBalancingBoard()) {
            robot.setCollectorPower(1);
            driveForRearWheel(1200, 1, 0, 3, alignmentAngle);
            while ((robot.getAngle() < alignmentAngle + 45 || robot.getAngle() > alignmentAngle + 55) && opModeIsActive()) {
                robot.drive(0, 0, robot.turn(alignmentAngle + 50));
                idle();
            }
            driveForRearWheel(500, 1, 0, 3, alignmentAngle + 50);
            driveForRearWheel(-800, 1, 0, 3, alignmentAngle);
            startingValue = robot.rightFrontMotor.getCurrentPosition();
            startingValue2 = robot.rightRearMotor.getCurrentPosition();
            startingValue3 = robot.leftFrontMotor.getCurrentPosition();
            startingValue4 = robot.leftRearMotor.getCurrentPosition();
            startTime = getRuntime();
            int targetS = 900;
            if (cryptoboxKey == CENTER)
                targetS = -400;
            if (cryptoboxKey == LEFT)
                targetS = -800;
            while (Math.abs(getAverageEncoderValue()) < Math.abs(targetS) && opModeIsActive() && getRuntime() < startTime + 4) {
                double rotationValue = 0;
                rotationValue = robot.turn(alignmentAngle);
                int direction;
                if (targetS > 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                robot.drive(0, direction * 1, rotationValue);
                telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
                telemetry.update();
                idle();
            }
            robot.drive(0, 0, 0);
            robot.setCollectorPower(0);
        } else {
            if (getDesiredColor() == RED) {
                alignmentAngle = -180;
                if (cryptoboxKey == CENTER)
                    driveForRearWheel(500, 0, 1, 3, alignmentAngle);
                else if (cryptoboxKey == RIGHT)
                    driveForRearWheel(900, 0, 1, 3, alignmentAngle);
                else if (cryptoboxKey == LEFT)
                    driveForRearWheel(100, 0, 1, 3, alignmentAngle);
                robot.setCollectorPower(1);
                driveForRearWheel(1200, 1, 0, 3, alignmentAngle);
                driveForRearWheel(900, 1, 0, 3, alignmentAngle + 45);
                driveForRearWheel(-800, 1, 3, alignmentAngle);
                robot.drive(0, 0, 0);
            } else {
                alignmentAngle = 0;
                if (cryptoboxKey == CENTER)
                    driveForRearWheel(600, 0, -1, 3, alignmentAngle);
                else if (cryptoboxKey == RIGHT)
                    driveForRearWheel(200, 0, -1, 3, alignmentAngle);
                else if (cryptoboxKey == LEFT)
                    driveForRearWheel(1100, 0, -1, 3, alignmentAngle);
                robot.setCollectorPower(1);
                driveForRearWheel(600, 1, 0, 3, alignmentAngle);
                while ((robot.getAngle() > alignmentAngle - 45 || robot.getAngle() < alignmentAngle - 55) && opModeIsActive()) {
                    robot.drive(0, 0, robot.turn(alignmentAngle - 50));
                    idle();
                }
                driveForRearWheel(450, .5, 1, 3, alignmentAngle - 50);
                driveForRearWheel(600, .6, 0, 3, alignmentAngle - 50);
                while ((robot.getAngle() > alignmentAngle + 05 || robot.getAngle() < alignmentAngle - 05) && opModeIsActive()) {
                    robot.drive(0, 0, robot.turn(alignmentAngle - 0)* 3);
                    idle();
                }
                driveForRearWheel(600, 1, 0, 3, alignmentAngle - 0);
                robot.setAlignmentServoUp(false);
                driveForRearWheel(-800, 1, 3, alignmentAngle);
                robot.drive(0, 0, 0);
                while (robot.getRange() > 30 && opModeIsActive()) {
                    robot.drive(-.5, 0, robot.turn(alignmentAngle));
                    if (robot.getRange() > 40 && opModeIsActive())
                        robot.drive(-1, 0, robot.turn(alignmentAngle));
                    telemetry.addData("Ultrasonic Distance", robot.getRange());
                    telemetry.update();
                    idle();
                }
                robot.drive(0, 0 ,0);
                robot.setAlignmentServoUp(true);
                sleep(500);
                if (cryptoboxKey == LEFT)
                driveFor(1000, .2, 1, 3, alignmentAngle);
                else
                    driveFor(1900, .2, 1, 3, alignmentAngle);

                robot.setAlignmentServoUp(false);
                sleep(1000);
            }
        }
        driveUpToCryptobox();
        robot.setCollectorPower(0);
        robot.setSpearServoUp(true);
        placeInCryptobox();
    }

    void driveFor(int encoderTicks, double speed, double time, Double angle) {
        driveFor(encoderTicks, speed, 0, time, angle);
    }

    void driveFor(int encoderTicks, double speed, double sideSpeed, double time, Double angle) {
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
            robot.drive(direction * Math.abs(speed), sideSpeed, rotationValue * 2);
            telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    void driveForRearWheel(int encoderTicks, double speed, double time, Double angle) {
        driveForRearWheel(encoderTicks, speed, 0, time, angle, 0);
    }

    void driveForRearWheel(int encoderTicks, double speed, double sideSpeed, double time, Double angle) {
        driveForRearWheel(encoderTicks, speed, sideSpeed, time, angle, 0);
    }

    void driveForRearWheel(int encoderTicks, double speed, double sideSpeed, double time, Double angle, double rotationMultiplier) {
        startingValue = robot.rightRearMotor.getCurrentPosition();
        startTime = getRuntime();
        double direction;
        int target = encoderTicks;
        if (target > 0) {
            direction = 1;
        } else {
            direction = -1;
        }

        while (Math.abs(robot.rightRearMotor.getCurrentPosition() - startingValue) < Math.abs(target) && opModeIsActive() && getRuntime() < startTime + time) {
            double rotationValue = 0;
            if (angle != null) {
                rotationValue = robot.turn(angle);
            }
            robot.drive(direction * Math.abs(speed), sideSpeed, rotationValue * 2);
            telemetry.addData("Encoder Value", robot.rightRearMotor.getCurrentPosition() - startingValue);
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    void calibrateGyroAndInitializeVuforia() {
        robot.initializeVuforia();
    }

    double getTimeLeft() {

        return 30 - (getRuntime() - opModeStartTime);
    }

    double getAverageEncoderValue() {
        double sum = 0;
        sum += Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue);
        sum += Math.abs(robot.rightRearMotor.getCurrentPosition() - startingValue2);
        sum += Math.abs(robot.leftFrontMotor.getCurrentPosition() - startingValue3);
        sum += Math.abs(robot.leftRearMotor.getCurrentPosition() - startingValue4);
        return sum / 4.0;
    }

    abstract Hardware.ColorDetected getDesiredColor();

    abstract boolean otherBalancingBoard();

    abstract boolean goForRankingPoints();

}
