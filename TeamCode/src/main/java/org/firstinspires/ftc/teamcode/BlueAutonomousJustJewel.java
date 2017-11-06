package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 9/29/2017.
 */
//@Disabled
@Autonomous(name = "Blue Autonomous Just Jewel")
public class BlueAutonomousJustJewel extends RelicRecoveryAutonomous{
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
    }

    @Override
    boolean justJewel() {
        return true;
    }
}
