package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.commands.AutoAimCommands;
import org.firstinspires.ftc.teamcode.Autonomous.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.Autonomous.commands.ServoCommands;
import org.firstinspires.ftc.teamcode.Autonomous.commands.TransferCommands;
import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.Servo.ServoComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Shared autonomous for every BIOBUZZ start.
 *
 * <p><b>Everything the routine needs lives here.</b> All poses are declared and
 * initialised in this class (generic = the RED 1 routine), so a subclass only
 * has to {@link #assignPoses() assign} its own start's poses; the paths are
 * rebuilt from those poses by {@link #buildPaths()}. That is the same layout as
 * {@code Brighton-FTC/2026}'s auton, updated to the Pedro 3 API and driven by
 * Ivy commands instead of a hand-rolled state machine.
 *
 * <p><b>How the pieces fit</b>
 * <ul>
 *   <li>{@link #init()} resets Ivy, creates the follower, seeds the
 *       localizer, creates every subsystem and builds the paths.</li>
 *   <li>{@link #start()} schedules the two sustained background commands
 *       (turret autoaim + SFA, the distance-based wheel speed) and then the
 *       routine returned by {@link #buildRoutine()}. Both keep running for the
 *       whole match, exactly like the driver's AUTO_AIM and SFA toggles held
 *       on; {@link #resetMechanisms()} only re-asserts them.</li>
 *   <li>{@link #loop()} runs the follower, then the Ivy scheduler.</li>
 * </ul>
 *
 * <p><b>Assembling an auton</b> — one cycle is: {@link #collectAlong(Path)
 * collect along a path}, {@link #spinUpAlong(Path) spin up along the approach},
 * {@link #shoot()}, then the next collect. The building blocks are ordinary
 * Ivy commands ({@link #followPath(Path)}, {@link #collecting()},
 * {@link #spinUpShooter()}, {@link #feedShot()}, {@link #resetMechanisms()},
 * plus the servo/motor one-shots below), so they compose with
 * {@code sequential(...)}, {@code parallel(...)}, {@code race(...)} and
 * {@code deadline(...)} exactly like the examples.
 */
public abstract class GenericAutonomous extends OpMode {

    // ------------------------------------------------------------------
    // Subsystems (created in init so hardwareMap exists)
    // ------------------------------------------------------------------

    protected Follower follower;
    protected TurretPIDComponent turret;
    protected DynamicAngleComponent launcher;
    protected FlyWheelMotorComponent transfer;
    protected IntakeMotorComponent intake;
    protected ServoComponent launchCap;
    protected ServoComponent rampServo;
    protected ServoComponent umbrellaServo;

    /** Sustained background commands, scheduled for the whole match. */
    protected Command autoAim;
    protected Command shootSpinUp;

    // ------------------------------------------------------------------
    // Poses — ALL of them are initialised here.
    // The defaults are the generic (RED 1) routine; a subclass re-assigns
    // them in assignPoses() before anything reads them.
    // ------------------------------------------------------------------

    protected Pose startPose = AutoConfig.AutoPoses.red1Start;
    protected Pose startScorePose = AutoConfig.AutoPoses.red1StartScore;
    protected Pose scorePose = AutoConfig.AutoPoses.red1Score;
    protected Pose pickup1Pose = AutoConfig.AutoPoses.red1Pickup1;
    protected Pose pickup2Pose = AutoConfig.AutoPoses.red1Pickup2;
    protected Pose pickup3Pose = AutoConfig.AutoPoses.red1Pickup3;
    protected Pose controlPoint1 = AutoConfig.AutoPoses.red1Control1;
    protected Pose controlPoint2 = AutoConfig.AutoPoses.red1Control2;
    protected Pose controlPoint3 = AutoConfig.AutoPoses.red1Control3;
    protected Pose parkPose = AutoConfig.AutoPoses.red1Park;

    /** Which wall-touching spot this routine starts from (telemetry + goal default). */
    protected Start fieldStart = Start.RED_NORTH;

    /** CELL this routine shoots at. Defaults to {@link Start#defaultGoal()} when left null. */
    protected Goal goal;

    // ------------------------------------------------------------------
    // Paths — rebuilt from the poses above by buildPaths()
    // ------------------------------------------------------------------

    protected Path pathStartToStartScore;
    protected Path pathStartScoreToScore;

    protected Path pathScoreToPickup1;
    protected Path pathPickup1ToScore;

    protected Path pathScoreToPickup2;
    protected Path pathPickup2ToScore;

    protected Path pathScoreToPickup3;
    protected Path pathPickup3ToScore;

    protected Path pathScoreToPark;

    // ------------------------------------------------------------------
    // OpMode lifecycle
    // ------------------------------------------------------------------

    @Override
    public void init() {
        // The scheduler is static, so a previous OpMode's commands must be
        // cleared before this one schedules anything.
        Scheduler.reset();

        // Poses first: everything below (follow pose, goal, paths) reads them.
        assignPoses();
        if (startPose == null) {
            startPose = Pose.zero();
        }
        if (goal == null) {
            goal = fieldStart.defaultGoal();
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        turret = new TurretPIDComponent(
                hardwareMap,
                RobotConfig.Hardware.TURRET_MOTOR,
                RobotConfig.Turret.DEGREES_PER_TICK,
                goal.x(), goal.y(),
                telemetry);
        turret.setGoal(goal);
        turret.resetTurretEncoder();

        launcher = new DynamicAngleComponent(
                hardwareMap,
                RobotConfig.Hardware.FLYWHEEL_MOTOR,
                goal.x(), goal.y(), goal.height());
        launcher.setGoal(goal);

        transfer = new FlyWheelMotorComponent(hardwareMap, RobotConfig.Hardware.TRANSFER_MOTOR);
        intake = new IntakeMotorComponent(hardwareMap, RobotConfig.Hardware.INTAKE_MOTOR);

        launchCap = new ServoComponent(
                hardwareMap,
                RobotConfig.Hardware.LAUNCH_CAP_SERVO,
                RobotConfig.ServoPose.LAUNCH_CAP_OPEN_POSITION,
                RobotConfig.ServoPose.LAUNCH_CAP_CLOSED_POSITION,
                RobotConfig.ServoPose.LAUNCH_CAP_CLOSED_POSITION);
        rampServo = new ServoComponent(
                hardwareMap,
                RobotConfig.Hardware.RAMP_SERVO,
                RobotConfig.ServoPose.RAMP_OPEN_POSITION,
                RobotConfig.ServoPose.RAMP_CLOSED_POSITION,
                RobotConfig.ServoPose.RAMP_CLOSED_POSITION);
        umbrellaServo = new ServoComponent(
                hardwareMap,
                RobotConfig.Hardware.UMBRELLA_SERVO,
                RobotConfig.ServoPose.UMBRELLA_OPEN_POSITION,
                RobotConfig.ServoPose.UMBRELLA_CLOSED_POSITION,
                RobotConfig.ServoPose.UMBRELLA_CLOSED_POSITION);

        // Built after the subsystems exist, so the lambdas capture live objects.
        autoAim = AutoAimCommands.aim(follower, turret);
        shootSpinUp = AutoAimCommands.spinUp(follower, launcher);

        buildPaths();

        telemetry.addData("Auton", getClass().getSimpleName());
        telemetry.addData("Start", "%s (%.1f, %.1f, %.0f deg)",
                fieldStart, startPose.x(), startPose.y(),
                Math.toDegrees(startPose.heading()));
        telemetry.addData("Goal", "%s (%.1f, %.1f, h=%.1f)",
                goal, goal.x(), goal.y(), goal.height());
        telemetry.update();
    }

    @Override
    public void start() {
        // Both loops stay scheduled for the whole match, exactly like the
        // driver's AUTO_AIM + SFA toggles held on: the turret tracks the goal
        // and the wheel is always at the speed for the current distance.
        startBackgroundLoops();
        schedule(buildRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        reportTelemetry();
    }

    // ------------------------------------------------------------------
    // Subclass hooks
    // ------------------------------------------------------------------

    /**
     * Assign this routine's poses (and, if it differs from
     * {@link Start#defaultGoal()}, its {@link #goal}).
     *
     * <p>Called once from {@link #init()} <i>before</i> the follower, the
     * shooter and the paths are set up.
     */
    protected void assignPoses() {
        // Generic routine: keep the field defaults initialised above.
    }

    /**
     * The auton itself. Assemble it from the command blocks in this class —
     * the sequence is deliberately left to each subclass.
     */
    protected abstract Command buildRoutine();

    /**
     * Builds every path from the current poses.
     *
     * <p>Override in a subclass if a routine needs a different path shape
     * (for example {@code Paths.through(...)} for a sweep through several
     * pickup points). The paths themselves are plain Pedro 3 {@link Path}s,
     * so they stay reusable for the whole match.
     */
    protected void buildPaths() {
        pathStartToStartScore = line(startPose, startScorePose);
        pathStartScoreToScore = line(startScorePose, scorePose);

        pathScoreToPickup1 = curve(scorePose, controlPoint1, pickup1Pose);
        pathPickup1ToScore = line(pickup1Pose, scorePose);

        pathScoreToPickup2 = curve(scorePose, controlPoint2, pickup2Pose);
        pathPickup2ToScore = line(pickup2Pose, scorePose);

        pathScoreToPickup3 = curve(scorePose, controlPoint3, pickup3Pose);
        pathPickup3ToScore = line(pickup3Pose, scorePose);

        pathScoreToPark = line(scorePose, parkPose);
    }

    // ------------------------------------------------------------------
    // Command blocks — mix and match these in buildRoutine()
    // ------------------------------------------------------------------

    /** Follows a path; finishes when the follower reaches its end. */
    protected Command followPath(Path path) {
        return PedroCommands.follow(follower, path);
    }

    /**
     * <b>Step 1.</b> Drives {@code path} with the collector running, and stows
     * the collector the moment the path ends — ramp up, intake and transfer
     * off, launch cap closed.
     */
    protected Command collectAlong(Path path) {
        return deadline(followPath(path), collecting());
    }

    /**
     * <b>Step 2.</b> Makes sure the turret autoaim and the SFA (shoot-any-
     * distance) wheel loop are running, then drives the approach path. Both
     * loops stay up for the rest of the match.
     */
    protected Command spinUpAlong(Path path) {
        return sequential(
                spinUpShooter(),
                followPath(path));
    }

    /**
     * <b>Steps 3 and 4.</b> One shot: waits for the turret, opens the launch
     * cap, feeds, then puts every mechanism back to its rest state.
     */
    protected Command shoot() {
        return sequential(
                waitForAim(),
                openLaunchCap(),
                waitMs(AutoConfig.Timing.SHOOT_SETTLE_MS),
                feedShot(),
                waitMs(AutoConfig.Timing.SHOOT_KICK_MS),
                resetMechanisms());
    }

    /**
     * The collecting state: ramp down, intake and transfer running, launch cap
     * closed. Sustained — compose it with {@code deadline}/{@code race}; it
     * stows the collector when it ends.
     */
    protected Command collecting() {
        return Command.build()
                .setStart(() -> {
                    rampServo.open();               // ramp down
                    launchCap.close();
                    intake.startMotor();
                    transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
                })
                .setEnd(endCondition -> stowCollector());
    }

    /** Ramp up, launch cap closed, intake and transfer off. */
    protected void stowCollector() {
        rampServo.close();
        launchCap.close();
        intake.stopMotor();
        transfer.stopMotor();
    }

    /** Starts the transfer and the intake — the shot itself. */
    protected Command feedShot() {
        return instant(() -> {
            intake.startMotor();
            transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
        });
    }

    /**
     * <b>Step 4.</b> Ramp up, cap closed, intake and transfer off. The turret
     * autoaim and the SFA wheel loop are re-asserted here rather than stopped:
     * they run for the whole match.
     */
    protected Command resetMechanisms() {
        return instant(() -> {
            stowCollector();
            startBackgroundLoops();
        });
    }

    /**
     * Schedules the two always-on loops (turret autoaim + SFA / shoot-any-
     * distance wheel control) if they are not already running. Safe to call at
     * any time: each command holds its own subsystem, and an already scheduled
     * command is left alone instead of being queued twice.
     */
    protected void startBackgroundLoops() {
        if (!Scheduler.isScheduled(autoAim)) {
            schedule(autoAim);
        }
        if (!Scheduler.isScheduled(shootSpinUp)) {
            schedule(shootSpinUp);
        }
    }

    /** Makes sure the turret + wheel loops are up; see {@link #startBackgroundLoops()}. */
    protected Command spinUpShooter() {
        return instant(this::startBackgroundLoops);
    }

    /** Waits until the turret is on target (bounded by the aim timeout). */
    protected Command waitForAim() {
        return AutoAimCommands.waitForAim(follower, turret, AutoConfig.Timing.AIM_TIMEOUT_MS);
    }

    /** Runs the intake (sustained — end it with a group or {@code until}). */
    protected Command intakeOn() {
        return IntakeCommands.collect(intake);
    }

    /** Reverses the intake (sustained). */
    protected Command eject() {
        return IntakeCommands.eject(intake);
    }

    /** Stops the intake immediately. */
    protected Command intakeOff() {
        return IntakeCommands.stop(intake);
    }

    /** Runs the transfer toward the shooter (sustained). */
    protected Command transferOn() {
        return TransferCommands.feed(transfer);
    }

    /** Reverses the transfer (sustained). */
    protected Command transferReverse() {
        return TransferCommands.reverse(transfer);
    }

    /** Stops the transfer immediately. */
    protected Command transferOff() {
        return TransferCommands.stop(transfer);
    }

    protected Command openLaunchCap() {
        return ServoCommands.open(launchCap);
    }

    protected Command closeLaunchCap() {
        return ServoCommands.close(launchCap);
    }

    protected Command openRamp() {
        return ServoCommands.open(rampServo);
    }

    protected Command closeRamp() {
        return ServoCommands.close(rampServo);
    }

    protected Command openUmbrella() {
        return ServoCommands.open(umbrellaServo);
    }

    protected Command closeUmbrella() {
        return ServoCommands.close(umbrellaServo);
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /** Straight path, heading interpolated from {@code from} to {@code to}. */
    protected static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    /** Bézier path bowed by {@code control}, heading interpolated end to end. */
    protected static Path curve(Pose from, Pose control, Pose to) {
        return Paths.curve(from, control, to).linear(from, to);
    }

    protected double poseX() {
        return follower.pose().x();
    }

    protected double poseY() {
        return follower.pose().y();
    }

    protected double poseHeading() {
        return follower.pose().heading();
    }

    private void reportTelemetry() {
        telemetry.addData("Pose", String.format("%.1f, %.1f, %.0f deg",
                poseX(), poseY(), Math.toDegrees(poseHeading())));
        telemetry.addData("Follower", "%s / path %d", follower.mode(), follower.pathIndex());
        telemetry.addData("Turret", "%.1f deg aimed=%s",
                turret.getCurrentAngle(), turret.isAimed(poseX(), poseY(), poseHeading()));
        telemetry.addData("Shooter", "%.0f rpm", launcher.getRPM());
        telemetry.addData("Launch cap", launchCap.isOpen() ? "open" : "closed");
        telemetry.update();
    }
}
