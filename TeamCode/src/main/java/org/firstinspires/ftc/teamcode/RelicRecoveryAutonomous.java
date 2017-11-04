package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import static org.firstinspires.ftc.teamcode.Hardware.RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE;
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

        telemetry.addData("Status", "Decoding Pictograph");
        telemetry.update();
        //decodePictograph();

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();

        //robot.relicArmTiltServo.setPosition(RELIC_ARM_TILT_SERVO_90_DEGREE_VALUE);

        telemetry.addData("Status", "Remove Jewel");
        telemetry.update();
        removeJewel();

        //telemetry.addData("Status", "Drive to Cryptobox");
        telemetry.update();
        driveToCryptobox();

        //telemetry.addData("Status", "Turn to Cryptobox");
        telemetry.update();
        turnToCryptobox();

        //telemetry.addData("Status", "Line Up With CryptoBox Key");
        telemetry.update();
        lineUpWithCryptoboxKey();

        //telemetry.addData("Status", "Place Glyph in Cryptobox");
        telemetry.update();
        placeGlyphInCryptoBox();

        telemetry.addData("Status", "Saving Cipher to File");
        telemetry.update();
        //robot.saveActiveCipherToFile();

        telemetry.addData("Status", "Done");
        telemetry.update();

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
        if (robot.getColorDetected() == getDesiredColor()) {
            driveInInches(3, .3);
        } else if (robot.getColorDetected() != Hardware.ColorDetected.NONE) {
            driveInInches(-3, .3);
            robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);
            driveInInches(6, .3);
        } else {
            driveInInches(3, .3);// do nothing OR random!!!!! rawr xD lolzor;
        }

        robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);

        while ((robot.getAngle() > 2 || robot.getAngle() < -2) && opModeIsActive()) {
            robot.drive(0, 0, robot.turn(0));
            telemetry.addData("Angle", robot.getAngle());
            telemetry.addData("Color", robot.getColorDetected());
            telemetry.update();
            idle();
        }
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
        driveInInches(20, 1);
    }

    private void turnToCryptobox() {
        while ((robot.getAngle() > 92 * directionMultiplier && robot.getAngle() < 88 * directionMultiplier) && opModeIsActive()) {
            robot.drive(0, 0, robot.turn(90 * directionMultiplier));
            idle();
        }
    }

    private void lineUpWithCryptoboxKey() {
    }

    private void placeGlyphInCryptoBox() {
        robot.setTiltServoPositionUp(true);
        driveInInches(5, .5);
        robot.grabTopServo.setPosition(Hardware.GRAB_TOP_SERVO_RELEASE);
        robot.grabTopServo.setPosition(Hardware.GRAB_TOP_SERVO_RELEASE);
    }
    int startingValue = 0;
    void driveInInches(double x, double speed){
        startingValue = robot.rightFrontMotor.getCurrentPosition();
        double direction;
        int target = (int) (x / (4 * Math.PI) * 1120);
        if (target > startingValue){
            direction = 1;
        }else{
            direction = -1;
        }
        robot.drive(direction * speed, 0, 0);
        while (Math.abs(robot.rightFrontMotor.getCurrentPosition() - startingValue) < Math.abs(target) && opModeIsActive()){
            telemetry.addData("Encoder Value", robot.rightFrontMotor.getCurrentPosition() - startingValue);
            telemetry.update();
            idle();
        }
        robot.drive(0, 0, 0);
    }


    abstract Hardware.ColorDetected getDesiredColor();

}
