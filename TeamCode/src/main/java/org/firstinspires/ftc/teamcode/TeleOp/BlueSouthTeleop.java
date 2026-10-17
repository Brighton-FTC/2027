package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Blue alliance, south wall start (BIOBUZZ Manual Fig 11-1, blue cols D/E/F,
 * touching south perimeter, clear of blue LOADING ZONE / FLOWER).
 * Opens aiming the south CELL (BLUE_AUDIENCE); call {@link #switchGoalForHiveTip()}
 * when the HIVE tips.
 */
@TeleOp(name = "Blue South", group = "Competition")
public class BlueSouthTeleop extends GenericTeleop {

    @Override
    protected Start getStart() {
        return Start.BLUE_SOUTH;
    }

    @Override
    protected Goal getInitialGoal() {
        return Goal.BLUE_AUDIENCE;
    }
}
