package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Red alliance, south wall start (BIOBUZZ Manual Fig 11-1, red cols A/B/C,
 * touching south perimeter, clear of GARDEN / FLOWER).
 * Opens aiming the south CELL (RED_AUDIENCE); call {@link #switchGoalForHiveTip()}
 * when the HIVE tips.
 */
@TeleOp(name = "Red South", group = "Competition")
public class RedSouthTeleop extends GenericTeleop {

    @Override
    protected Start getStart() {
        return Start.RED_SOUTH;
    }

    @Override
    protected Goal getInitialGoal() {
        return Goal.RED_AUDIENCE;
    }
}
