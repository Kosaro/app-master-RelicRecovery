package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 9/29/2017.
 */
//@Disabled
@Autonomous(name = "Red Autonomous")
public class RedAutonomous extends RelicRecoveryAutonomous {
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.RED;
    }

    @Override
    boolean otherBalancingBoard() {
        return false;
    }
}