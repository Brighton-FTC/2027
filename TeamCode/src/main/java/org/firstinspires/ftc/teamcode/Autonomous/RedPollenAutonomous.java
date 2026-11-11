package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Red alliance vision-autonomous from the NORTH wall start (36, 132, facing
 * the HIVE) — the wall spot none of the tip routines use (they all start
 * south). The shooter loops aim at the SCORING CELL; the drive itself is the
 * yellow-pollen chase from {@link GenericPollenAutonomous}.
 */
@Configurable
@Autonomous(name = "Red 1 Pollen", group = "Autonomous")
public class RedPollenAutonomous extends GenericPollenAutonomous {

    @Override
    protected void assignPoses() {
        fieldStart = Start.RED_NORTH;
        goal = Goal.RED_SCORING;
        // North-wall start, mirrors Start.RED_NORTH (AutoConfig red1 slot).
        startPose = AutoConfig.AutoPoses.red1Start;
    }
}
