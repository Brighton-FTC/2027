package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Red alliance, north wall start (BIOBUZZ Manual Fig 11-1, red cols A/B/C,
 * touching north perimeter, clear of red LOADING ZONE / FLOWER).
 * Opens aiming the north CELL (RED_SCORING); call {@link #switchGoalForHiveTip()}
 * when the HIVE tips.
 */
@TeleOp(name = "Red North", group = "Competition")
public class RedNorthTeleop extends GenericTeleop {

    @Override
    protected Start getStart() {
        return Start.RED_NORTH;
    }

    @Override
    protected Goal getInitialGoal() {
        return Goal.RED_SCORING;
    }
}
