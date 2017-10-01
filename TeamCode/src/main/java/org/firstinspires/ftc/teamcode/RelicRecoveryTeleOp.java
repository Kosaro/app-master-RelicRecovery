package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

/**
 * Created by okosa on 9/9/2017.
 */
@Disabled
@TeleOp(name = "RelicRecoveryAutonomous")
public class RelicRecoveryTeleOp extends OpMode {
    Hardware robot;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public void pickUp(){
    }

    public void rotate(){

    }

    public void robotOrientation(){
        if (gamepad1.right_bumper){
            robot.robotOrientation = true;
        }
        if (gamepad1.left_bumper) {
            robot.robotOrientation = false;
        }
    }

}
