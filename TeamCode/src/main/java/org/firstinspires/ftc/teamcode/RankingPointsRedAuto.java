package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by okosa on 12/9/2017.
 */
@Disabled
@Autonomous(name = "Ranking Points Red Auto", group = "Red")
public class RankingPointsRedAuto extends RelicRecoveryAutonomous{
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.RED;
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