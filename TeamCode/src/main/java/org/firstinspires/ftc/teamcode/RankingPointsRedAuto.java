package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 12/9/2017.
 */

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