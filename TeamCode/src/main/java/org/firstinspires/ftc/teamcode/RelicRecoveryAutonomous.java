package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by okosa on 9/9/2017.
 */
//@Disabled
@Autonomous(name = "RelicRecoveryAutonomous")
public abstract class RelicRecoveryAutonomous extends LinearOpMode {
    //Local Variables
    Hardware robot;
    RelicRecoveryVuMark cryptoboxKey;  //UNKNOWN, LEFT, CENTER, RIGHT

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot = new Hardware(hardwareMap);
        robot.initializeVuforia();
        robot.setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Decoding Pictograph");
        telemetry.update();
        decodePictograph();

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Remove Jewel");
        telemetry.update();
        removeJewel();

        telemetry.addData("Status", "Drive to Cryptobox");
        telemetry.update();
        driveToCryptobox();

        telemetry.addData("Status", "Line Up With CryptoBox Key");
        telemetry.update();
        lineUpWithCryptoboxKey();

        telemetry.addData("Status", "Place Glyph in Cryptobox");
        telemetry.update();
        placeGlyphInCryptoBox();
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
        robot.getColorDetected();
        if (robot.getColorDetected() == getDesiredColor()){
            while (robot.getAngle() > -15 && opModeIsActive()){
                robot.drive(0, 0, robot.turn(-17));
                idle();
            }
        }
        else if (robot.getColorDetected() != Hardware.ColorDetected.NONE){
            while (robot.getAngle() < 15 && opModeIsActive()){
                robot.drive(0, 0, robot.turn(17));
                idle();
            }
        }
        else {
            while (robot.getAngle() > -15 && opModeIsActive()){
                robot.drive(0, 0, robot.turn(-17));
                idle();
            }// do nothing OR random!!!!! rawr xD lolzor;
        }
        robot.jewelServo.setPosition(robot.JEWEL_SERVO_UP);

    }

    private void driveToCryptobox() {
        while ((robot.getAngle() > 2 && robot.getAngle() < -2) && opModeIsActive()){
            robot.drive(0, 0, robot.turn(0));
            idle();
        }

        int finalPosition = 0;
        if (cryptoboxKey == RelicRecoveryVuMark.RIGHT)
            finalPosition = 1;
        else if (cryptoboxKey == RelicRecoveryVuMark.LEFT)
            finalPosition = 2;
        else if (cryptoboxKey == RelicRecoveryVuMark.CENTER)
            finalPosition = 3;
        else if (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN)
            finalPosition = 4;
        robot.drive(1,0,0);
        while ((robot.leftFrontMotor.getCurrentPosition() < finalPosition)&&(opModeIsActive())){
            idle();
        }
        robot.stop();
    }

    private void lineUpWithCryptoboxKey() {
    }

    private void placeGlyphInCryptoBox() {
    }

    abstract Hardware.ColorDetected getDesiredColor();

}
