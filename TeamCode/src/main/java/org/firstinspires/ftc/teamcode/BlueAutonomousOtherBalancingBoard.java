package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 9/29/2017.
 */
//@Disabled
@Autonomous(name = "Blue Autonomous Other Balancing Board")
public class BlueAutonomousOtherBalancingBoard extends RelicRecoveryAutonomous {
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
    }

    @Override
    boolean otherBalancingBoard() {
        return true;
    }
}