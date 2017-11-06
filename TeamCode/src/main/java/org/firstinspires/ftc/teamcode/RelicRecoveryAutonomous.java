package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.Hardware.RELIC_ARM_TILT_SERVO_LOWER_LIMIT;

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

        telemetry.addData("Status", "Decoding Pictograph");
        telemetry.update();
        //decodePictograph();

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();

        //
        // robot.relicArmTiltServo.setPosition(RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE);

        telemetry.addData("Status", "Remove Jewel");
        telemetry.update();
        removeJewel();

        if (!justJewel()) {
            telemetry.addData("Status", "Drive to Cryptobox");
            telemetry.update();
            driveToCryptobox();

            sleep(1000);

            telemetry.addData("Status", "Turn to Cryptobox");
            telemetry.update();
            turnToCryptobox();

            sleep(1000);

            telemetry.addData("Status", "Line Up With CryptoBox Key");
            telemetry.update();
            //lineUpWithCryptoboxKey();

            telemetry.addData("Status", "Place Glyph in Cryptobox");
            telemetry.update();
            placeGlyphInCryptoBox();

            telemetry.addData("Status", "Saving Cipher to File");
            telemetry.update();
            //robot.saveActiveCipherToFile();
        }

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
        sleep(1500);
        if (getDesiredColor() == Hardware.ColorDetected.RED) {
            if (robot.getColorDetected() == getDesiredColor()) {
                driveFor(-100, .3);
                robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
                driveFor(200, .3);
            } else if (robot.getColorDetected() != Hardware.ColorDetected.NONE) {
                driveFor(100, .3);
            } else {
                driveFor(100, .3);// do nothing OR random!!!!! rawr xD lolzor;
            }
        } else {
            if (robot.getColorDetected() == getDesiredColor()) {
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
        driveFor(600 * directionMultiplier, 1);
    }

    private void turnToCryptobox() {
        double angle;
        if (getDesiredColor() == Hardware.ColorDetected.RED) {
            angle = -120;
        } else {
            angle = -60;
        }
        while ((robot.getAngle() < angle - 3 || robot.getAngle() > angle + 3) && opModeIsActive()) {
            robot.drive(0, 0, robot.turn(angle));
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }

    private void lineUpWithCryptoboxKey() {
    }

    private void placeGlyphInCryptoBox() {
        robot.setTiltServoPositionUp(true);
        sleep(1500);
        driveFor(-300, .25, 3);
        robot.setGrabbersClosed(false);
        robot.setTiltServoPositionUp(false);
        driveFor(100, .25, 2);
    }

    int startingValue = 0;

    void driveFor(int x, double speed) {
        driveFor(x, speed, 10);
    }
    void driveFor(int x, double speed, double time) {
        driveFor(x, speed, 10, null);
    }

    double startTime;


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
            if (angle != null){
                rotationValue = robot.turn(0);
            }
            robot.drive(direction * speed, 0, 0);
            telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }


    abstract Hardware.ColorDetected getDesiredColor();
    abstract boolean justJewel();

}
