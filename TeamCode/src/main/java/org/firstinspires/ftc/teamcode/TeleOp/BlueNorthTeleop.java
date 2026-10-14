package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Blue alliance, north wall start (BIOBUZZ Manual Fig 11-1, blue cols D/E/F,
 * touching north perimeter, clear of GARDEN / FLOWER).
 * Opens aiming the north CELL (BLUE_SCORING); call {@link #switchGoalForHiveTip()}
 * when the HIVE tips.
 */
@TeleOp(name = "Blue North", group = "Competition")
public class BlueNorthTeleop extends GenericTeleop {

    @Override
    protected Start getStart() {
        return Start.BLUE_NORTH;
    }

    @Override
    protected Goal getInitialGoal() {
        return Goal.BLUE_SCORING;
    }
}
