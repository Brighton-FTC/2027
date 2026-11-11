package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.OpenCV.OpenCVComponent;
import org.firstinspires.ftc.teamcode.OpenCV.PollenPaths;
import org.firstinspires.ftc.teamcode.OpenCV.PoseCreatorComponent;
import org.firstinspires.ftc.teamcode.Servo.ServoComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.AprilTag.TiltEstimateComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotControls;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Vision.HiveCell;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;
import org.firstinspires.ftc.teamcode.controls.Binding;
import org.firstinspires.ftc.teamcode.controls.ControlAction;
import org.firstinspires.ftc.teamcode.controls.DriverProfile;
import org.firstinspires.ftc.teamcode.controls.DriverProfiles;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.EnumMap;
import java.util.Map;

/**
 * Shared TeleOp implementation. Subclasses pick one {@link Start} and one
 * initial {@link Goal} from {@link RobotConfig.Field} (BIOBUZZ manual TU04) —
 * this class never hardcodes a red/blue choice itself.
 *
 * <p>Goals/starts stay dynamic: {@link #activeGoal} is re-read from
 * {@link RobotConfig} every loop and pushed into the turret/launcher, so
 * dashboard retunes and mid-match switches (HIVE TIP -> other CELL, auto
 * routines, gamepad overrides) take effect immediately. Call
 * {@link #setActiveGoal(Goal)} / {@link #switchGoalForHiveTip()} from subclass
 * code or future game-logic hooks.
 *
 * <p>All tuning lives in {@link RobotConfig}; driver bindings live in
 * {@link org.firstinspires.ftc.teamcode.controls.DriverProfiles} — a named
 * {@link DriverProfile} per driver, mapping the nine {@link ControlAction}s to
 * any button/trigger on either pad. The active profile is chosen by
 * {@link DriverProfiles#ACTIVE}, so swapping drivers never touches this class.
 * {@link RobotControls} survives only for the tuning doubles it still owns
 * (fire duration, collect-feeds-kicker, slow-mode toggle preference).
 *
 * <p>HOLD actions are level-driven (release auto-stops, nothing latches on);
 * TOGGLE/PULSE actions latch or fire per style. Every action now drives real
 * hardware: ramp, umbrella and switch-goal are wired to the servos and the
 * live goal setters.
 */
public abstract class GenericTeleop extends OpMode {

    protected Follower follower;
    protected TelemetryManager panels;

    private GamepadEx driver;
    private GamepadEx operator;

    private TurretPIDComponent turret;
    private DynamicAngleComponent launcher;
    private ServoComponent launchCap;
    private ServoComponent rampServo;
    private ServoComponent umbrellaServo;
    private IntakeMotorComponent intake;
    private AprilTagLocalization aprilTags;
    private TiltEstimateComponent tiltEstimator;

    private Pose startingPose;

    // ---- control mapping (swappable per driver) ----
    private DriverProfile profile;
    private final Map<ControlAction, TriggerReader> triggerReaders =
            new EnumMap<>(ControlAction.class);

    // Per-action runtime state, indexed by ControlAction.ordinal().
    private final boolean[] actionActive = new boolean[ControlAction.values().length];
    private final boolean[] actionPressed = new boolean[ControlAction.values().length];

    // Latched state. Hold-to-run actions intentionally keep no latch — they are
    // recomputed from the gamepad every loop.
    private boolean fieldCentric;
    private boolean slowModeToggle = false;
    private boolean slowActive = false;
    private double firingUntilS = 0.0;
    private boolean rampOpen = false;
    private boolean umbrellaOpen = false;

    /** Dynamic target — switch mid-match as the HIVE tips / game evolves. */
    protected volatile Goal activeGoal;

    /** Which wall-touching start this OpMode uses (one of the 4 manual spots). */
    protected abstract Start getStart();

    /** Goal aimed at init; becomes {@link #activeGoal} (switchable later). */
    protected abstract Goal getInitialGoal();

    protected abstract OpenCVComponent.Target getColour();
    private PoseCreatorComponent pollenLocator;

    // ---- pollen pathing (OpenCV) ----
    /** Below this travel the "path" is a point — nothing to follow. */
    private static final double MIN_POLLEN_PATH_IN = 0.5;
    /** True when the last pollen path attempt ended without following. */
    private boolean pollenPathBlocked;
    /** Why the last pollen path attempt followed or not (telemetry). */
    private String pollenPathStatus = "never run";
    /** Latched teleauton state — mirrors TOGGLE_TELEAUTON's actionActive. */
    private boolean teleautonActive;

    /** Current goal XY/height, always read live from {@link RobotConfig} tuning. */
    protected Goal getActiveGoal() {
        return activeGoal;
    }

    /** Retarget mid-match (e.g. HIVE TIP, auto routine, gamepad override). */
    protected void setActiveGoal(Goal goal) {
        if (goal != null) {
            activeGoal = goal;
        }
    }

    /** Flip to the other CELL on the same HIVE (SCORING <-> AUDIENCE). */
    protected void switchGoalForHiveTip() {
        if (activeGoal != null) {
            activeGoal = activeGoal.otherEndOfHive();
        }
    }

    protected boolean isRedAlliance() {
        return getStart().isRed();
    }

    /** Tilt cell follows the active goal so vision stays consistent. */
    protected HiveCell getTargetCell() {
        return activeGoal != null ? activeGoal.cell() : getInitialGoal().cell();
    }

    /** Calibrated upright pitch (shared tuning, override only if needed per alliance). */
    protected double getNormalPitch() {
        return RobotConfig.Vision.NORMAL_PITCH_DEG;
    }

    /** Tilt threshold in deg (shared tuning, override only if needed per alliance). */
    protected double getTiltThreshold() {
        return RobotConfig.Vision.TILT_THRESHOLD_DEG;
    }

    /** Where localization is seeded at init — built live from {@link #getStart()} so dashboard edits apply. */
    protected Pose getStartingPose() {
        Start s = getStart();
        return new Pose(s.x(), s.y(), s.headingRad());
    }

    @Override
    public void init() {
        fieldCentric = RobotConfig.Drive.FIELD_CENTRIC_DEFAULT;
        activeGoal = getInitialGoal();
        startingPose = getStartingPose();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose == null ? Pose.zero() : startingPose);
        follower.update();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        turret = new TurretPIDComponent(
                hardwareMap,
                RobotConfig.Hardware.TURRET_MOTOR,
                RobotConfig.Turret.DEGREES_PER_TICK,
                activeGoal.x(), activeGoal.y(),
                telemetry);
        launcher = new DynamicAngleComponent(
                hardwareMap,
                RobotConfig.Hardware.FLYWHEEL_MOTOR,
                activeGoal.x(), activeGoal.y(),
                activeGoal.height());
        launchCap = new ServoComponent(hardwareMap, RobotConfig.Hardware.LAUNCH_CAP_SERVO, RobotConfig.ServoPose.LAUNCH_CAP_OPEN_POSITION, RobotConfig.ServoPose.LAUNCH_CAP_CLOSED_POSITION, RobotConfig.ServoPose.LAUNCH_CAP_CLOSED_POSITION);
        intake = new IntakeMotorComponent(hardwareMap, RobotConfig.Hardware.INTAKE_MOTOR);
        rampServo = new ServoComponent(hardwareMap, RobotConfig.Hardware.RAMP_SERVO, RobotConfig.ServoPose.RAMP_OPEN_POSITION, RobotConfig.ServoPose.RAMP_CLOSED_POSITION, RobotConfig.ServoPose.RAMP_CLOSED_POSITION);
        umbrellaServo = new ServoComponent(hardwareMap, RobotConfig.Hardware.UMBRELLA_SERVO, RobotConfig.ServoPose.UMBRELLA_OPEN_POSITION, RobotConfig.ServoPose.UMBRELLA_CLOSED_POSITION, RobotConfig.ServoPose.UMBRELLA_CLOSED_POSITION);

        try {
            aprilTags = new AprilTagLocalization(hardwareMap, telemetry);
        } catch (Exception e) {
            aprilTags = null;
            telemetry.addData("AprilTag vision", "init failed: %s", e.getMessage());
        }
        tiltEstimator = new TiltEstimateComponent(getNormalPitch(), getTiltThreshold());

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        profile = DriverProfiles.active();
        rebuildTriggerReaders();
        // Drive-mode starts latched to the configured default, so the first
        // press flips away from it instead of silently forcing robot-centric.
        actionActive[ControlAction.TOGGLE_DRIVE_MODE.ordinal()] = fieldCentric;

        turret.resetTurretEncoder();

        pollenLocator = new PoseCreatorComponent(getColour(), follower.pose());
        // Stream from init so the teleauton tracker has frames by match start.
        pollenLocator.startCamera();
    }

    /**
     * Command that drives a Pedro straight line from the robot's pose at
     * start to the selected pollen (x, y), arriving facing {@code heading} —
     * feed it {@link PoseCreatorComponent#createPose()} output.
     *
     * <p>The path is built when the command starts (fresh pose, fresh vision
     * snapshot) and then checked against every OTHER detected object: if any
     * lies within {@link RobotConfig.Field#POLLEN_PATH_CLEARANCE_IN} of the
     * line, the path is rejected and the command ends without driving.
     *
     * <p>{@code follower.update()} already runs in {@link #loop()}; while the
     * teleauton toggle is latched, {@link #handleDrive()} stands aside so
     * driver manual input cannot cancel the follow.
     */
    public Command pathToPollen(double x, double y, double heading) {
        return Command.build()
                .setStart(() -> followPollenPath(x, y, heading))
                .setDone(() -> pollenPathBlocked
                        || !follower.isBusy()
                        || !follower.following())
                .requiring(follower);
    }

    /** Build, collision-check and (when clear) follow the line to a pollen. */
    private void followPollenPath(double x, double y, double heading) {
        pollenPathBlocked = true;
        pollenPathStatus = "no usable target";

        Pose start = follower.pose();
        Pose end = new Pose(x, y, heading);
        if (Double.isNaN(end.x()) || Double.isNaN(end.y())) {
            return;
        }
        double travel = start.distance(end);
        if (travel < MIN_POLLEN_PATH_IN) {
            pollenPathStatus = String.format("already at target (%.1f in)", travel);
            return;
        }
        // Heading is interpolated along the line by PollenPaths.plan(); the
        // 9in keep-out around every other detected object is checked there.
        Path path = PollenPaths.plan(start, end, pollenLocator.getObstaclePositions(),
                RobotConfig.Field.POLLEN_PATH_CLEARANCE_IN, MIN_POLLEN_PATH_IN);
        if (path == null) {
            pollenPathStatus = String.format(
                    "blocked: object within %.1f in of path",
                    RobotConfig.Field.POLLEN_PATH_CLEARANCE_IN);
            return;
        }
        pollenPathBlocked = false;
        pollenPathStatus = String.format("following (%.1f in)", travel);
        follower.follow(path);
    }

    // ---- teleauton (auto pollen tracking) ----

    /**
     * Runs the teleauton system while {@code TOGGLE_TELEAUTON} is latched:
     * whenever no path is in progress, re-reads the webcam (yellow pollen
     * pipeline), picks the largest detection and drives a collision-checked
     * line to it — repeating until the toggle goes off. Manual drive is
     * suspended for the duration (see {@link #handleDrive()}).
     */
    private void handleTeleauton() {
        boolean tracking = actionActive[ControlAction.TOGGLE_TELEAUTON.ordinal()];
        if (tracking != teleautonActive) {
            teleautonActive = tracking;
            // Pollens are the yellow artifact — switch the pipeline over for
            // the duration and restore the alliance colour afterwards.
            pollenLocator.setTarget(tracking
                    ? OpenCVComponent.Target.YELLOW : getColour());
        }
        if (!tracking) {
            return;
        }

        if (follower.isBusy()) {
            return; // leg in progress — let it finish before re-acquiring
        }

        // Fresh vision, re-anchored to the pose we are actually at.
        pollenLocator.refresh(follower.pose());
        Pose target = pollenLocator.createPose();
        if (target == null) {
            pollenPathStatus = "teleauton: searching";
            return; // nothing visible this frame — retry next tick
        }
        followPollenPath(target.x(), target.y(), target.heading());
    }

    @Override
    public void start() {
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        driver.readButtons();
        operator.readButtons();

        // Live profile swap support: if the dashboard changed
        // DriverProfiles.ACTIVE, adopt the new profile (and its trigger
        // readers) immediately.
        updateActiveProfile();
        readActions();

        updateShooterGoal();
        handleActions();
        handleTeleauton();
        handleDrive();
        reportActionTelemetry();
        reportTiltTelemetry();
        reportTelemetry();

        panels.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTags != null) {
            try {
                aprilTags.close();
            } catch (Exception ignored) {
            }
        }
        if (pollenLocator != null) {
            pollenLocator.closeCamera();
        }
    }

    // ---- control mapping plumbing ----

    /** Adopt a dashboard-changed profile for the very next loop tick. */
    private void updateActiveProfile() {
        if (profile == null) {
            profile = DriverProfiles.active();
            rebuildTriggerReaders();
        } else if (DriverProfiles.active() != profile) {
            profile = DriverProfiles.active();
            rebuildTriggerReaders();
        }
    }

    /** Trigger bindings need FTCLib {@link TriggerReader}s for edge detection. */
    private void rebuildTriggerReaders() {
        triggerReaders.clear();
        if (profile == null) {
            return;
        }
        for (Map.Entry<ControlAction, Binding> e : profile.all().entrySet()) {
            Binding b = e.getValue();
            if (b.trigger.isPresent()) {
                triggerReaders.put(e.getKey(),
                        new TriggerReader(b.pad == Binding.Pad.OPERATOR ? operator : driver,
                                b.trigger.get()));
            }
        }
    }

    /** Poll every bound action, filling per-action active/pressed state. */
    private void readActions() {
        // Advance trigger edge state first, so justPressed() below compares
        // against this tick's fresh sample.
        for (TriggerReader reader : triggerReaders.values()) {
            reader.readValue();
        }
        java.util.Arrays.fill(actionPressed, false);
        if (profile == null) {
            return;
        }
        for (Map.Entry<ControlAction, Binding> e : profile.all().entrySet()) {
            ControlAction action = e.getKey();
            Binding b = e.getValue();
            boolean pressed = justPressed(action);
            boolean active = actionActive[action.ordinal()];

            switch (b.style) {
                case HOLD:
                    active = held(action);
                    break;
                case TOGGLE:
                    if (pressed) {
                        active = !active;
                    }
                    break;
                case PULSE:
                    if (pressed) {
                        active = true; // one tick only
                    } else {
                        active = false;
                    }
                    break;
            }
            actionActive[action.ordinal()] = active;
            actionPressed[action.ordinal()] = pressed;
        }

        // Universal keys: polled outside the profile map so they work on
        // every profile — including driver J's, which has no drive-mode
        // binding at all — and alongside the profile's own bindings.
        pollUniversalToggle(DriverProfiles.UNIVERSAL_DRIVE_MODE,
                ControlAction.TOGGLE_DRIVE_MODE);
        pollUniversalToggle(DriverProfiles.UNIVERSAL_TELEAUTON,
                ControlAction.TOGGLE_TELEAUTON);
    }

    /** One universal TOGGLE key: press flips the action's latched state. */
    private void pollUniversalToggle(Binding binding, ControlAction action) {
        if (binding.button.isPresent()
                && pad(binding).wasJustPressed(binding.button.get())) {
            int i = action.ordinal();
            actionActive[i] = !actionActive[i];
            actionPressed[i] = true;
        }
    }

    /** Which pad a binding reads from. */
    private GamepadEx pad(Binding binding) {
        return binding.pad == Binding.Pad.OPERATOR ? operator : driver;
    }

    private boolean justPressed(ControlAction action) {
        Binding b = profile.get(action);
        if (b == null) {
            return false;
        }
        if (b.trigger.isPresent()) {
            TriggerReader reader = triggerReaders.get(action);
            return reader != null && reader.wasJustPressed();
        }
        return pad(b).wasJustPressed(b.button.get());
    }

    private boolean held(ControlAction action) {
        Binding b = profile.get(action);
        if (b == null) {
            return false;
        }
        if (b.trigger.isPresent()) {
            TriggerReader reader = triggerReaders.get(action);
            return reader != null && reader.isDown();
        }
        // GamepadEx.isDown() is the FTCLib hold-check (edge state comes from
        // readButtons()); getButton() would just re-read the same field.
        return pad(b).isDown(b.button.get());
    }

    // ---- per-action dispatch (_robot behavior_, independent of keys) ----

    private void handleActions() {
        if (profile == null) {
            return;
        }

        // The feed path is one subsystem: resolve it once per tick, with flush
        // winning over intake (matching the old unclog behavior).
        boolean flushing = actionActive[ControlAction.FLUSH_STORAGE.ordinal()];
        boolean collecting = !flushing && actionActive[ControlAction.TOGGLE_INTAKE.ordinal()];
        handleFeed(flushing, collecting);

        for (ControlAction action : ControlAction.values()) {
            boolean active = actionActive[action.ordinal()];
            switch (action) {
                case SHOOT:
                    // Flush wins: reversing the path must not also feed a shot.
                    if (active && !flushing) {
                        // Momentary kick; the timer below auto-closes it.
                        launchCap.open();
                        firingUntilS = getRuntime()
                                + Math.max(0.0, RobotControls.FIRE_KICK_SECONDS);
                    }
                    break;

                case TOGGLE_AUTOAIM:
                    if (active) {
                        turret.aimToObject(poseX(), poseY(), poseHeading());
                    } else {
                        turret.stop();
                    }
                    break;

                case TOGGLE_SHOOT_ANY_DISTANCE:
                    if (active) {
                        launcher.dynamicMotorPower(poseX(), poseY());
                    } else {
                        launcher.stop();
                    }
                    break;

                case TOGGLE_RAMP:
                    if (active != rampOpen) {
                        rampOpen = active;
                        if (active) {
                            rampServo.open();
                        } else {
                            rampServo.close();
                        }
                    }
                    break;

                case TOGGLE_UMBRELLA:
                    if (active != umbrellaOpen) {
                        umbrellaOpen = active;
                        if (active) {
                            umbrellaServo.open();
                        } else {
                            umbrellaServo.close();
                        }
                    }
                    break;

                case SWITCH_GOAL:
                    // PULSE/TOGGLE both work: act on the press edge only, so
                    // holding the key retargets once.
                    if (actionPressed[action.ordinal()]) {
                        switchGoalForHiveTip();
                    }
                    break;

                case TOGGLE_DRIVE_MODE:
                    fieldCentric = active;
                    break;

                case TOGGLE_TELEAUTON:
                    // consumed by handleTeleauton() (owns the drivetrain
                    // while latched) and handleDrive() (stands aside)
                    break;

                case TOGGLE_INTAKE:
                case FLUSH_STORAGE:
                    // handled above by handleFeed(), once per tick
                    break;

                case SLOW_MODE:
                    // consumed by handleDrive()
                    break;
            }
        }

        // Fire timer: auto-close the launchCap when the pulse laps.
        if (firingUntilS != 0.0 && getRuntime() >= firingUntilS) {
            launchCap.close();
            firingUntilS = 0.0;
        }
    }
    private void handleFeed(boolean flushing, boolean collecting) {
        if (flushing) {
            // Unclog wins over everything: reverse the path, force launchCap shut.
            intake.reverseMotor();
            launchCap.close();
            firingUntilS = 0.0;
        } else if (collecting) {
            intake.startMotor();
            if (RobotControls.COLLECT_OPENS_KICKER) {
                launchCap.open();
            }
            firingUntilS = 0.0;
        } else {
            intake.stopMotor();
            if (firingUntilS == 0.0) {
                // Safe default: launchCap stays shut when idle so staged balls
                // can't dribble into the wheel.
                launchCap.close();
            }
        }
    }

    private void updateShooterGoal() {
        // Push the (possibly re-switched / dashboard-retuned) goal every loop
        // so turret + shooter track it with no restart.
        if (isTilt()) {
            switchGoalForHiveTip();
        }
        if (activeGoal != null) {
            launcher.setGoal(activeGoal);
            turret.setGoal(activeGoal);
        }
    }

    // ---- drive ----

    private void handleDrive() {
        if (teleautonActive) {
            // Teleauton owns the drivetrain this tick: a manual() call here
            // would cancel the follow. Toggling teleauton off restores it.
            telemetry.addData("Slow mode", slowActive);
            telemetry.addData("Field centric", fieldCentric);
            return;
        }
        // Slow mode: re-honors the legacy RobotControls.SLOW_IS_TOGGLE toggle
        // preference even under the new profile system.
        boolean slow;
        if (RobotControls.SLOW_IS_TOGGLE) {
            if (actionActive[ControlAction.SLOW_MODE.ordinal()]
                    && actionPressed[ControlAction.SLOW_MODE.ordinal()]) {
                slowModeToggle = !slowModeToggle;
            }
            slow = slowModeToggle;
        } else {
            slow = actionActive[ControlAction.SLOW_MODE.ordinal()];
        }
        slowActive = slow;
        double scale = slow ? RobotConfig.Drive.SLOW_MODE_MULTIPLIER : 1.0;

        double forward = driver.getLeftY() * scale;
        double strafe = -driver.getLeftX() * scale;
        double turn = -driver.getRightX() * scale;

        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward, strafe, turn, follower.pose().heading());
            follower.manual(powers);
        } else {
            follower.manual(forward, strafe, turn);
        }

        telemetry.addData("Slow mode", slow);
        telemetry.addData("Field centric", fieldCentric);
    }

    // ---- telemetry ----

    private void reportTiltTelemetry() {
        HiveCell target = getTargetCell();
        if (aprilTags == null) {
            telemetry.addData("Goal tilt", "%s: vision unavailable", target);
            telemetry.addData("Goal isTilt", "UNKNOWN");
            panels.debug("tilt/status", target + ": vision unavailable");
            panels.debug("tilt/isTilt", "UNKNOWN");
            return;
        }
        double pitch = aprilTags.getClusterPitch(target);
        boolean visible = pitch != AprilTagLocalization.UNKNOWN_PITCH;
        boolean isTilt = visible && tiltEstimator.isTilted(pitch);
        String status = visible
                ? String.format("%s pitch %.1f deg", target, pitch)
                : String.format("%s not visible", target);
        String tiltStatus = !visible ? "UNKNOWN" : (isTilt ? "TILTED" : "upright");
        // Driver Station.
        telemetry.addData("Goal tilt", status);
        telemetry.addData("Goal tilt (raw)", visible ? pitch : "UNKNOWN");
        telemetry.addData("Goal isTilt", "%s (normal %.1f, thr %.1f)", tiltStatus,
                tiltEstimator.getNormalPitch(), getTiltThreshold());
        // Panels (uses debug(key, value) — same channel style as position/velocity).
        panels.debug("tilt/status", status);
        panels.debug("tilt/isTilt", tiltStatus);
        if (visible) {
            panels.debug("tilt/pitchDeg", pitch);
        }
    }

    private void reportTelemetry() {
        telemetry.addData("Control profile (Panels: DriverProfiles.ACTIVE)", profile == null ? "?" : profile.name);
        if (profile != null) {
            for (Map.Entry<ControlAction, Binding> e : profile.all().entrySet()) {
                telemetry.addData("  " + e.getKey().label, e.getValue().toString());
            }
        }
        telemetry.addData("Universal drive mode",
                "%s (always live)", DriverProfiles.UNIVERSAL_DRIVE_MODE);
        telemetry.addData("Start", "%s (%.1f, %.1f, %.0f deg)",
                getStart(), getStart().x(), getStart().y(),
                getStart().headingDeg());
        if (activeGoal != null) {
            telemetry.addData("Goal", "%s (%.1f, %.1f, h=%.1f) -> %s",
                    activeGoal, activeGoal.x(), activeGoal.y(),
                    activeGoal.height(), activeGoal.cell());
        }
        telemetry.addData("Teleauton", teleautonActive
                ? "TRACKING (manual suspended)" : "off (SHARE)");
        telemetry.addData("Pollen path", pollenPathStatus);
        panels.debug("teleauton", teleautonActive);
        panels.debug("pollen/path", pollenPathStatus);
        panels.debug("position", follower.pose());
        panels.debug("velocity", follower.velocity());
    }

    /** Status of every bound action this tick (unbound actions are skipped). */
    private void reportActionTelemetry() {
        if (profile == null) {
            return;
        }
        for (ControlAction action : ControlAction.values()) {
            if (profile.get(action) == null) {
                continue; // unbound — don't spam telemetry with dead rows
            }
            boolean active = actionActive[action.ordinal()];
            String status;
            switch (action) {
                case SHOOT:
                    status = active || firingUntilS != 0.0 ? "FIRING" : "ready";
                    break;
                case TOGGLE_AUTOAIM:
                    status = active ? "ON" : "off";
                    break;
                case TOGGLE_SHOOT_ANY_DISTANCE:
                    status = active ? "ON" : "off";
                    break;
                case FLUSH_STORAGE:
                    status = active ? "FLUSHING" : "idle";
                    break;
                case TOGGLE_INTAKE:
                    status = active ? "ON" : "off";
                    break;
                case TOGGLE_DRIVE_MODE:
                    status = fieldCentric ? "FIELD centric" : "robot centric";
                    break;
                case TOGGLE_RAMP:
                    status = rampOpen ? "OPEN" : "closed";
                    break;
                case TOGGLE_UMBRELLA:
                    status = umbrellaOpen ? "OPEN" : "closed";
                    break;
                case SWITCH_GOAL:
                    status = "target " + getTargetCell();
                    break;
                case SLOW_MODE:
                    status = slowActive ? "ON" : "off";
                    break;
                default:
                    status = active ? "ON" : "off";
                    break;
            }
            telemetry.addData("Action: " + action.label, status);
        }
        telemetry.addData("Shooter RPM (target)", launcher.getRPM());
        telemetry.addData("Turret deg", turret.getCurrentAngle());
        telemetry.addData("launchCap open", launchCap.isOpen());
    }

    private double poseX() {
        return follower.pose().x();
    }

    private double poseY() {
        return follower.pose().y();
    }

    private double poseHeading() {
        return follower.pose().heading();
    }

    public boolean isTilt() {
        HiveCell target = getTargetCell();
        if (aprilTags == null) {
            return false; // vision failed at init — never auto-flip the goal
        }
        double pitch = aprilTags.getClusterPitch(target);
        boolean visible = pitch != AprilTagLocalization.UNKNOWN_PITCH;
        return visible && tiltEstimator.isTilted(pitch);
    }
}
