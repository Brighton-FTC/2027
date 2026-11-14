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
 * Red alliance "3 tip" routine — start, flower, side, north shot, flower.
 *
 * <p>Four legs: the first two are the same shape as {@link Red2Tip}, then the
 * robot drives up the field to shoot from the north side and goes back out to
 * a flower.
 *
 * <p>Geometry, all in inches / degrees, field origin at the corner:
 * <pre>
 *   start        (56.00,   8.00,  90)
 *     -> flower  (12.75,  46.75, 180)   curve, control (34.75, 50.75)
 *     -> side    (10.75,   5.75, 270)   straight, heading piecewise at 0.5
 *     -> shoot   (46.75, 130.75,  90)   straight, heading piecewise at 0.7
 *     -> flower  (14.75, 119.75,  90)   straight, heading held at 90
 * </pre>
 *
 * <p>{@link Blue3Tip} is the mirror image of this class across {@code x = 72}.
 */
@Configurable
@Autonomous(name = "Red 3 Tip", group = "Autonomous")
public class Red3Tip extends GenericAutonomous {

    // ------------------------------------------------------------------
    // Poses — names kept verbatim from the tuned config.
    // ------------------------------------------------------------------

    private final Pose start = AutoConfig.AutoPoses.red3TipStart;
    private final Pose startToFlower = AutoConfig.AutoPoses.red3TipStartToFlower;
    private final Pose startToFlowerControl1 = AutoConfig.AutoPoses.red3TipStartToFlowerControl1;
    private final Pose flowerToSide = AutoConfig.AutoPoses.red3TipFlowerToSide;
    private final Pose flowerToSideSegment1Heading = AutoConfig.AutoPoses.red3TipFlowerToSideSegment1Heading;
    private final Pose flowerToSideSegment2Start = AutoConfig.AutoPoses.red3TipFlowerToSideSegment2Start;
    private final Pose flowerToSideSegment2End = AutoConfig.AutoPoses.red3TipFlowerToSideSegment2End;
    private final Pose sideToShoot = AutoConfig.AutoPoses.red3TipSideToShoot;
    private final Pose sideToShootSegment1Heading = AutoConfig.AutoPoses.red3TipSideToShootSegment1Heading;
    private final Pose sideToShootSegment2Start = AutoConfig.AutoPoses.red3TipSideToShootSegment2Start;
    private final Pose sideToShootSegment2End = AutoConfig.AutoPoses.red3TipSideToShootSegment2End;
    private final Pose shootToFlower = AutoConfig.AutoPoses.red3TipShootToFlower;

    // ------------------------------------------------------------------
    // Paths — built by buildPaths() from the poses above.
    // ------------------------------------------------------------------

    private Path pathStartToFlower;
    private Path pathFlowerToSide;
    private Path pathSideToShoot;
    private Path pathShootToFlower;

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
        pickup3Pose = shootToFlower;
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

        // Side -> shooting station: straight drive, heading held at 270 for the
        // first 70% and then swung round to 90 before the shot.
        pathSideToShoot = Paths.line(flowerToSide, sideToShoot)
                .heading(Interpolator.piecewise()
                        .until(0.7, Interpolator.constant(sideToShootSegment1Heading))
                        .until(1, Interpolator.linear(sideToShootSegment2Start,
                                sideToShootSegment2End)));

        // Shoot -> flower, holding the approach heading.
        pathShootToFlower = Paths.line(sideToShoot, shootToFlower)
                .constant(shootToFlower);
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
                shoot(),

                // 5. Back out to the flower, collecting for the next cycle.
                collectAlong(pathShootToFlower),
                resetMechanisms());
    }
}
