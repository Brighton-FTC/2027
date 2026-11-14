package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

/**
 * Blue alliance vision-autonomous from the NORTH wall start (108, 132,
 * facing the HIVE) — the wall spot none of the tip routines use (they all
 * start south). The shooter loops aim at the SCORING CELL; the drive itself
 * is the yellow-pollen chase from {@link GenericPollenAutonomous}.
 */
@Configurable
@Autonomous(name = "Blue 1 Pollen", group = "Autonomous")
public class BluePollenAutonomous extends GenericPollenAutonomous {

    @Override
    protected void assignPoses() {
        fieldStart = Start.BLUE_NORTH;
        goal = Goal.BLUE_SCORING;
        // North-wall start, mirrors Start.BLUE_NORTH (AutoConfig blue1 slot).
        startPose = AutoConfig.AutoPoses.blue1Start;
    }
}
