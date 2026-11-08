package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Red alliance, "tip" routine — start, flower, side, shooting station.
 *
 * <p>The three paths are hand-tuned rather than generated from the generic
 * pose slots, so this auton overrides {@link #buildPaths()}. Everything else
 * (follower, subsystems, background autoaim + spin-up, the command blocks used
 * in {@link #buildRoutine()}) comes from {@link GenericAutonomous}.
 *
 * <p>Geometry, all in inches / degrees, field origin at the corner:
 * <pre>
 *   start            (56.00,   8.00,  90)
 *     -> flower      (12.75,  46.75, 180)   curve, control (34.75, 50.75)
 *     -> side        (10.75,   5.75, 270)   straight, heading piecewise
 *     -> shoot       ( 7.75, 102.75, 270)   curve, control (13.75, 54.75)
 * </pre>
 *
 * <p>Poses live in {@link AutoConfig.AutoPoses} (degrees section, prefix
 * {@code red2Tip}) so they stay editable from Panels like every other auton.
 */
@Configurable
@Autonomous(name = "Red 2 Tip", group = "Autonomous")
public class Red2Tip extends GenericAutonomous {

    // ------------------------------------------------------------------
    // Poses — names kept verbatim from the tuned config.
    // ------------------------------------------------------------------

    private final Pose start = AutoConfig.AutoPoses.red2TipStart;
    private final Pose startToFlower = AutoConfig.AutoPoses.red2TipStartToFlower;
    private final Pose startToFlowerControl1 = AutoConfig.AutoPoses.red2TipStartToFlowerControl1;
    private final Pose flowerToSide = AutoConfig.AutoPoses.red2TipFlowerToSide;
    private final Pose flowerToSideSegment1Heading = AutoConfig.AutoPoses.red2TipFlowerToSideSegment1Heading;
    private final Pose flowerToSideSegment2Start = AutoConfig.AutoPoses.red2TipFlowerToSideSegment2Start;
    private final Pose flowerToSideSegment2End = AutoConfig.AutoPoses.red2TipFlowerToSideSegment2End;
    private final Pose sideToShoot = AutoConfig.AutoPoses.red2TipSideToShoot;
    private final Pose sideToShootControl1 = AutoConfig.AutoPoses.red2TipSideToShootControl1;

    // ------------------------------------------------------------------
    // Paths — built by buildPaths() from the poses above.
    // ------------------------------------------------------------------

    private Path pathStartToFlower;
    private Path pathFlowerToSide;
    private Path pathSideToShoot;

    // ------------------------------------------------------------------
    // GenericAutonomous hooks
    // ------------------------------------------------------------------

    @Override
    protected void assignPoses() {
        // Only used for telemetry and as the default goal seed; the real
        // start pose is the tuned `start` above.
        fieldStart = Start.RED_SOUTH;
        goal = Goal.RED_SCORING;

        // Keep the generic slots in sync: the base class seeds the localizer
        // from startPose and its helper commands read the others.
        startPose = start;
        pickup1Pose = startToFlower;
        controlPoint1 = startToFlowerControl1;
        pickup2Pose = flowerToSide;
        scorePose = sideToShoot;
        controlPoint2 = sideToShootControl1;
    }

    @Override
    protected void buildPaths() {
        // Start -> flower, bowed out through the control point.
        pathStartToFlower = Paths.curve(start, startToFlowerControl1, startToFlower)
                .linear(start, startToFlower);

        // Flower -> side: straight drive, heading held at 180 for the first
        // half and then swept to 270 over the second half.
        pathFlowerToSide = Paths.line(startToFlower, flowerToSide)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(flowerToSideSegment1Heading))
                        .until(1, Interpolator.linear(flowerToSideSegment2Start,
                                flowerToSideSegment2End)));

        // Side -> shooting station, holding the shooting heading throughout.
        pathSideToShoot = Paths.curve(flowerToSide, sideToShootControl1, sideToShoot)
                .constant(sideToShoot);
    }

    @Override
    protected Command buildRoutine() {
        return sequential(
                // 1. Collect on the way to the flower and on to the side.
                collectAlong(pathStartToFlower),
                collectAlong(pathFlowerToSide),

                // 2. Ensure autoaim + SFA are up, then drive to the shot.
                spinUpAlong(pathSideToShoot),

                // 3 + 4. Shoot, then stow the collector (turret + wheel stay up).
                shoot());
    }
}
