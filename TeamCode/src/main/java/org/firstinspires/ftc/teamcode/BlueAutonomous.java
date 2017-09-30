package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

/**
 * Created by okosa on 9/29/2017.
 */

public class BlueAutonomous extends RelicRecoveryAutonomous{
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
    }
}
