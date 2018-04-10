package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by okosa on 12/9/2017.
 */
@Disabled
@Autonomous(name = "Ranking Points Blue Auto", group = "Blue")
public class RankingPointsBlueAuto extends RelicRecoveryAutonomous{
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
    }

    @Override
    boolean otherBalancingBoard() {
        return false;
    }

    @Override
    boolean goForRankingPoints() {
        return true;
    }
}
