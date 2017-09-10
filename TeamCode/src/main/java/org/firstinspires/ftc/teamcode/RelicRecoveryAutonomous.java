package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by okosa on 9/9/2017.
 */
//@Disabled
@Autonomous(name = "RelicRecoveryAutonomous")
public class RelicRecoveryAutonomous extends LinearOpMode {
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

        telemetry.addData("Status", "Waiting For Start");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Decoding Pictograph");
        telemetry.update();
        decodePictograph();

        telemetry.addData("Status", "Drive to Jewel Set");
        telemetry.update();
        driveToJewelSet();

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
        double scanDuration = 1;    //second(s)
        double startTime = getRuntime();
        do {
            cryptoboxKey = robot.getPictograph();
            telemetry.addData("Cryptobox Key", cryptoboxKey);
            telemetry.update();
            idle();
        }
        while (cryptoboxKey == RelicRecoveryVuMark.UNKNOWN && getRuntime() < startTime + scanDuration && opModeIsActive());
    }

    private void driveToJewelSet() {
    }

    private void removeJewel() {
    }

    private void driveToCryptobox() {
    }

    private void lineUpWithCryptoboxKey() {
    }

    private void placeGlyphInCryptoBox() {
    }
}
