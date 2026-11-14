package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.OpenCV.OpenCVComponent;
import org.firstinspires.ftc.teamcode.OpenCV.PollenPaths;
import org.firstinspires.ftc.teamcode.OpenCV.PoseCreatorComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Shared vision-autonomous: chases yellow pollens with the webcam.
 *
 * <p>The same system the TeleOp teleauton toggle runs, as the whole
 * autonomous routine: whenever no leg is in progress, the locator re-reads
 * the yellow pipeline against the pose the robot is actually at, the largest
 * detection becomes the target, and the robot drives a collision-checked
 * straight line to it ({@link PollenPaths}, 9in keep-out around every other
 * detection) with the collector up — repeating until the match timer ends
 * the OpMode. Subclasses only pick the start spot (see {@code
 * RedPollenAutonomous} / {@code BluePollenAutonomous}, which take the NORTH
 * wall starts the tip routines don't use).
 *
 * <p>The keep-out covers detected objects; the HIVE structure itself is not
 * visible to the yellow pipeline, so start poses / target selection keep the
 * robot on its own half the same way the TeleOp teleauton does.
 */
public abstract class GenericPollenAutonomous extends GenericAutonomous {

    /** Below this travel the "leg" is a point — nothing to follow. */
    private static final double MIN_LEG_IN = 0.5;

    /** Yellow-pollen locator; camera streamed from init, closed on stop. */
    protected PoseCreatorComponent pollenLocator;

    /** What the chase did on its last tick (telemetry). */
    private String chaseStatus = "waiting for start";

    @Override
    public void init() {
        super.init();

        // Pollens are the yellow artifact — the whole chase runs the yellow
        // pipeline. Stream from init so frames are flowing by match start.
        // This autonomous owns its camera (the TeleOp shares one instead).
        pollenLocator = new PoseCreatorComponent(
                new OpenCVComponent(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME,
                        telemetry, OpenCVComponent.Target.YELLOW),
                follower.pose());
        pollenLocator.startCamera();

        telemetry.addData("Pollen chase", "yellow pipeline, %s start", fieldStart);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pollenLocator != null) {
            pollenLocator.closeCamera();
        }
    }

    /**
     * Paths are vision-driven (built leg-by-leg from the webcam), so there is
     * nothing to pre-build from the generic pose slots.
     */
    @Override
    protected void buildPaths() {
        // intentionally empty — see chaseTick()
    }

    /** The whole routine: chase pollens with the collector up, until time. */
    @Override
    protected Command buildRoutine() {
        return pollenChase();
    }

    /**
     * The pollen chase itself: whenever no leg is in progress, re-read the
     * webcam, pick the largest detection and drive a collision-checked line
     * to it. A blocked or invisible target simply retries next tick with
     * fresh data.
     */
    protected Command pollenChase() {
        return Command.build()
                .setStart(this::startCollector)
                .setExecute(this::chaseTick)
                .setDone(() -> false) // the match timer ends the chase
                .setEnd(endCondition -> stowCollector());
    }

    /** One chase tick — acquire, plan one leg, follow it. */
    private void chaseTick() {
        if (follower.isBusy()) {
            return; // leg in progress — let it finish before re-acquiring
        }

        // Fresh vision, re-anchored to the pose we are actually at.
        pollenLocator.refresh(follower.pose());
        Pose target = pollenLocator.createPose();
        if (target == null) {
            chaseStatus = "searching (no pollen visible)";
            telemetry.addData("Pollen chase", chaseStatus);
            return;
        }

        Path leg = PollenPaths.plan(follower.pose(), target,
                pollenLocator.getObstaclePositions(),
                RobotConfig.Field.POLLEN_PATH_CLEARANCE_IN, MIN_LEG_IN);
        if (leg == null) {
            chaseStatus = String.format("blocked: object within %.1f in of leg",
                    RobotConfig.Field.POLLEN_PATH_CLEARANCE_IN);
            telemetry.addData("Pollen chase", chaseStatus);
            return;
        }

        chaseStatus = String.format("leg -> (%.1f, %.1f)", target.x(), target.y());
        telemetry.addData("Pollen chase", chaseStatus);
        follower.follow(leg);
    }

    /** Collector up for the chase (same states as {@link #collecting()}). */
    private void startCollector() {
        rampServo.open();               // ramp down
        launchCap.close();
        intake.startMotor();
        transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
    }
}
