package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.Servo.ServoComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.AprilTag.TiltEstimateComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotControls;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Vision.HiveCell;
import org.firstinspires.ftc.teamcode.controls.Binding;
import org.firstinspires.ftc.teamcode.controls.ControlAction;
import org.firstinspires.ftc.teamcode.controls.DriverProfile;
import org.firstinspires.ftc.teamcode.controls.DriverProfiles;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.EnumMap;
import java.util.Map;

/**
 * Shared TeleOp implementation. Subclasses provide all alliance-specific
 * picks (goal position, tilt cell, starting pose) from {@link RobotConfig} —
 * this class never hardcodes a red/blue choice itself.
 *
 * <p>All tuning lives in {@link RobotConfig}; bindings live in
 * {@link org.firstinspires.ftc.teamcode.controls.DriverProfiles} — a
 * named {@link DriverProfile} per driver, mapping the nine
 * {@link ControlAction}s to any button/trigger on either pad. The active
 * profile is chosen by {@link DriverProfiles#ACTIVE}, so swapping drivers
 * never touches this class.
 *
 * <p>HOLD actions are level-driven (release auto-stops, nothing latches on);
 * TOGGLE/PULSE actions latch or fire per style. Actions with no hardware
 * behind them yet (ramp, umbrella, switch goal) report their state in
 * telemetry instead of controlling hardware.
 */
public abstract class GenericTeleop extends OpMode {

    protected Follower follower;
    protected TelemetryManager panels;

    private GamepadEx driver;
    private GamepadEx operator;

    private TurretPIDComponent turret;
    private DynamicAngleComponent launcher;
    private FlyWheelMotorComponent transfer;
    private ServoComponent kicker;
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
    private double firingUntilS = 0.0;

    /** Alliance goal X in inches (field frame), picked from {@link RobotConfig} by the subclass. */
    protected abstract double getGoalX();

    /** Alliance goal Y in inches (field frame), picked from {@link RobotConfig} by the subclass. */
    protected abstract double getGoalY();

    /** Alliance goal height in inches, picked from {@link RobotConfig} by the subclass. */
    protected abstract double getGoalHeight();

    /** Which HIVE CELL to report tilt for, picked from {@link RobotConfig} by the subclass. */
    protected abstract HiveCell getTargetCell();

    /** Calibrated upright pitch (shared tuning, override only if needed per alliance). */
    protected double getNormalPitch() {
        return RobotConfig.Vision.NORMAL_PITCH_DEG;
    }

    /** Tilt threshold in deg (shared tuning, override only if needed per alliance). */
    protected double getTiltThreshold() {
        return RobotConfig.Vision.TILT_THRESHOLD_DEG;
    }

    /** Where localization is seeded at init. */
    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        fieldCentric = RobotConfig.Drive.FIELD_CENTRIC_DEFAULT;
        startingPose = getStartingPose();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose == null ? Pose.zero() : startingPose);
        follower.update();

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        double goalX = getGoalX();
        double goalY = getGoalY();
        double goalHeight = getGoalHeight();

        turret = new TurretPIDComponent(
                hardwareMap,
                RobotConfig.Hardware.TURRET_MOTOR,
                RobotConfig.Turret.DEGREES_PER_TICK,
                goalX, goalY,
                telemetry);
        launcher = new DynamicAngleComponent(
                hardwareMap,
                RobotConfig.Hardware.FLYWHEEL_MOTOR,
                goalX, goalY,
                goalHeight);
        transfer = new FlyWheelMotorComponent(hardwareMap, RobotConfig.Hardware.TRANSFER_MOTOR);
        kicker = new ServoComponent(hardwareMap, RobotConfig.Hardware.LAUNCH_CAP_SERVO);
        intake = new IntakeMotorComponent(hardwareMap, RobotConfig.Hardware.INTAKE_MOTOR);
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

        turret.resetTurretEncoder();
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
        handleDrive();
        handleActions();
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
        return pad(b).isDown(b.button.get()) || pad(b).getButton(b.button.get());
    }

    // ---- per-action dispatch (_robot behavior_, independent of keys) ----

    private void handleActions() {
        if (profile == null) {
            return;
        }

        // Flush wins over intake, matching the old unclog behavior.
        boolean flushing = actionActive[ControlAction.FLUSH_STORAGE.ordinal()];
        boolean collecting = !flushing && actionActive[ControlAction.TOGGLE_INTAKE.ordinal()];

        for (ControlAction action : ControlAction.values()) {
            boolean active = actionActive[action.ordinal()];
            switch (action) {
                case SHOOT:
                    if (active) {
                        // Momentary kick; the timer below auto-closes it.
                        kicker.open();
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

                case TOGGLE_INTAKE:
                case FLUSH_STORAGE:
                    if (flushing) {
                        // Same as old UNCLOG: reverse the path, force kicker shut.
                        intake.reverseMotor();
                        transfer.runMotorAt(RobotConfig.Transfer.REVERSE_POWER);
                        kicker.close();
                        firingUntilS = 0.0;
                    } else if (collecting) {
                        intake.startMotor();
                        transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
                    } else {
                        intake.stopMotor();
                        transfer.stopMotor();
                        if (firingUntilS == 0.0) {
                            // Safe default: kicker stays shut when idle so
                            // staged balls can't dribble into the wheel.
                            kicker.close();
                        }
                    }
                    break;

                case TOGGLE_DRIVE_MODE:
                    fieldCentric = active;
                    break;

                case TOGGLE_RAMP:
                case TOGGLE_UMBRELLA:
                case SWITCH_GOAL:
                    // NOT WIRED YET: no ramp component, no umbrella component,
                    // and no runtime goal setters on
                    // TurretPIDComponent / DynamicAngleComponent. The latch is
                    // captured in actionActive and shown in telemetry, so the
                    // key is "live" the moment the hardware lands — add control
                    // logic here.
                    break;

                case SLOW_MODE:
                    // consumed by handleDrive()
                    break;
            }
        }

        // Fire timer: auto-close the kicker when the pulse laps.
        if (firingUntilS != 0.0 && getRuntime() >= firingUntilS) {
            kicker.close();
            firingUntilS = 0.0;
        }
    }

    // ---- drive ----

    private void handleDrive() {
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
        panels.debug("position", follower.pose());
        panels.debug("velocity", follower.velocity());
    }

    /** Status of every bound action this tick (shows unwired ones live too). */
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
                default:
                    status = active ? "ON (not wired yet)" : "off (not wired yet)";
                    break;
            }
            telemetry.addData("Action: " + action.label, status);
        }
        telemetry.addData("Shooter RPM (target)", launcher.getRPM());
        telemetry.addData("Turret deg", turret.getCurrentAngle());
        telemetry.addData("Kicker open", kicker.isOpen());
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
}
