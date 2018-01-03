package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by okosa on 12/15/2017.
 */
//@Disabled
    @TeleOp(name = "Relic Motor")
public class RelicMotor extends OpMode{
    Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        robot.relicArmExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.relicArmExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        robot.relicArmExtendMotor.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Encoder Value", robot.relicArmExtendMotor.getCurrentPosition());
    }
}
