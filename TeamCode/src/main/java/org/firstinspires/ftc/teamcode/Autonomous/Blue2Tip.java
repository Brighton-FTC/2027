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
 * Blue alliance "tip" routine — the mirror image of {@link Red2Tip}: start,
 * flower, side, shooting station, on the blue half of the field.
 *
 * <p>Every pose is the red one reflected across {@code x = 72}
 * ({@code x -> 144 - x}, {@code heading -> 180 - heading}), so the two routines
 * describe the same three path shapes and run the same sequence of actions.
 * {@link com.pedropathing.api.PoseFactory#mirrorX(double)} is deliberately not
 * used: it negates the heading instead of applying {@code 180 - heading}, which
 * would leave the robot facing the wrong way.
 *
 * <p>Geometry, all in inches / degrees, field origin at the corner:
 * <pre>
 *   start            ( 88.00,   8.00,  90)
 *     -> flower      (131.25,  46.75,   0)   curve, control (109.25, 50.75)
 *     -> side        (133.25,   5.75, 270)   straight, heading piecewise
 *     -> shoot       (136.25, 102.75, 270)   curve, control (130.25, 54.75)
 * </pre>
 *
 * <p>Poses live in {@link AutoConfig.AutoPoses} (degrees section, prefix
 * {@code blue2Tip}) so they stay editable from Panels like every other auton.
 */
@Configurable
@Autonomous(name = "Blue 2 Tip", group = "Autonomous")
public class Blue2Tip extends GenericAutonomous {

    // ------------------------------------------------------------------
    // Poses — the Red2Tip names, mirrored.
    // ------------------------------------------------------------------

    private final Pose start = AutoConfig.AutoPoses.blue2TipStart;
    private final Pose startToFlower = AutoConfig.AutoPoses.blue2TipStartToFlower;
    private final Pose startToFlowerControl1 = AutoConfig.AutoPoses.blue2TipStartToFlowerControl1;
    private final Pose flowerToSide = AutoConfig.AutoPoses.blue2TipFlowerToSide;
    private final Pose flowerToSideSegment1Heading = AutoConfig.AutoPoses.blue2TipFlowerToSideSegment1Heading;
    private final Pose flowerToSideSegment2Start = AutoConfig.AutoPoses.blue2TipFlowerToSideSegment2Start;
    private final Pose flowerToSideSegment2End = AutoConfig.AutoPoses.blue2TipFlowerToSideSegment2End;
    private final Pose sideToShoot = AutoConfig.AutoPoses.blue2TipSideToShoot;
    private final Pose sideToShootControl1 = AutoConfig.AutoPoses.blue2TipSideToShootControl1;

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
        fieldStart = Start.BLUE_SOUTH;
        goal = Goal.BLUE_SCORING;

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

        // Flower -> side: straight drive, heading held at 0 for the first half
        // and then swept to 270 over the second half.
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
