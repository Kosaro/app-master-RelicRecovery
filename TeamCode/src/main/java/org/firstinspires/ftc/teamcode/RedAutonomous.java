package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 9/29/2017.
 */

public class RedAutonomous extends RelicRecoveryAutonomous {
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.RED;
    }
}