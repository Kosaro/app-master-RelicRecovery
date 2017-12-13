package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RelicRecoveryAutonomous;

/**
 * Created by okosa on 12/9/2017.
 */
//@Disabled
@Autonomous(name = "Ranking Points Blue Other Balancing Board", group = "Blue")
public class RankingPointsBlueAutoOtherBalancingBoard extends RelicRecoveryAutonomous {
    @Override
    Hardware.ColorDetected getDesiredColor() {
        return Hardware.ColorDetected.BLUE;
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