package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by okosa on 12/9/2017.
 */

//@Disabled
@Autonomous(name = "Ranking Points Red Other Balancing Board", group = "Red")
public class RankingPointsRedAutoOtherBalancingBoard extends RelicRecoveryAutonomous {
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.RED;
    }

    @Override
    boolean otherBalancingBoard() {
        return true;
    }

    @Override
    boolean goForRankingPoints() {
        return true;
    }
}
