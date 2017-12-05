package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 9/29/2017.
 */
//@Disabled
@Autonomous(name = "Blue Autonomous")
public class BlueAutonomous extends RelicRecoveryAutonomous{
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
    }

    @Override
    boolean otherBalancingBoard() {
        return false;
    }
}
