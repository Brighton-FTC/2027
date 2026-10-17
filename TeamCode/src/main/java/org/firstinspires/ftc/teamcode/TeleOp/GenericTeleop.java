package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotControls;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Vision.HiveCell;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Goal;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Field.Start;
import org.firstinspires.ftc.teamcode.pedro.Constants;

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
 * <p>All tuning lives in {@link RobotConfig} and all
 * bindings live in {@link RobotControls}.
 *
 * <p>Simplified controls (see {@link RobotControls} to rebind, PlayStation names):
 * <ul>
 *   <li>Driver: left stick = drive/strafe, right stick X = turn,
 *       hold SLOW (RIGHT_BUMPER) for slow mode, press FIELD_CENTRIC (SQUARE) to toggle centricity.</li>
 *   <li>Operator: hold COLLECT (CROSS) to run intake + transfer + launchCap together,
 *       press AIM_AND_SPIN (CIRCLE) to toggle turret auto-aim + flywheel together,
 *       press FIRE (TRIANGLE) to kick one shot, hold UNCLOG (DPAD_DOWN) to reverse the feed.</li>
 * </ul>
 *
 * <p>HOLD actions are level-driven (release auto-stops, nothing latches on);
 * only AIM_AND_SPIN (and SLOW when {@code SLOW_IS_TOGGLE}) latch.
 */
public abstract class GenericTeleop extends OpMode {

    protected Follower follower;
    protected TelemetryManager panels;

    private GamepadEx driver;
    private GamepadEx operator;

    private TurretPIDComponent turret;
    private DynamicAngleComponent launcher;
    private FlyWheelMotorComponent transfer;
    private ServoComponent launchCap;
    private ServoComponent rampServo;
    private ServoComponent umbrellaServo;
    private IntakeMotorComponent intake;
    private AprilTagLocalization aprilTags;
    private TiltEstimateComponent tiltEstimator;

    private Pose startingPose;

    // Latched state. Hold-to-run actions (collect/unclog) intentionally keep no
    // latch — they are recomputed from the gamepad every loop.
    private boolean fieldCentric;
    private boolean slowModeToggle = false;
    private boolean aimingAndSpinning = false;
    private double firingUntilS = 0.0;
    private boolean toggledUmbrella = false;

    /** Dynamic target — switch mid-match as the HIVE tips / game evolves. */
    protected volatile Goal activeGoal;

    /** Which wall-touching start this OpMode uses (one of the 4 manual spots). */
    protected abstract Start getStart();

    /** Goal aimed at init; becomes {@link #activeGoal} (switchable later). */
    protected abstract Goal getInitialGoal();

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
        transfer = new FlyWheelMotorComponent(hardwareMap, RobotConfig.Hardware.TRANSFER_MOTOR);
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

        updateShooterGoal();
        handleUmbrella();
        handleDrive();
        handleAimAndSpin();
        handleFeed();
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

    // ---- input helpers: every binding resolves through RobotControls.
    // Gamepads are fixed by role: driver actions on gamepad1, operator on gamepad2.

    private boolean driverJustPressed(GamepadKeys.Button button) {
        return driver.wasJustPressed(button);
    }

    private boolean driverHeld(GamepadKeys.Button button) {
        return driver.getButton(button);
    }

    private boolean operatorJustPressed(GamepadKeys.Button button) {
        return operator.wasJustPressed(button);
    }

    private boolean operatorHeld(GamepadKeys.Button button) {
        return operator.getButton(button);
    }

    // ---- drive ----

    private void handleDrive() {
        if (RobotControls.SLOW_IS_TOGGLE && driverJustPressed(RobotControls.SLOW_BUTTON)) {
            slowModeToggle = !slowModeToggle;
        }
        boolean slow = RobotControls.SLOW_IS_TOGGLE
                ? slowModeToggle
                : driverHeld(RobotControls.SLOW_BUTTON);
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

        if (driverJustPressed(RobotControls.FIELD_CENTRIC_BUTTON)) {
            fieldCentric = !fieldCentric;
        }

        telemetry.addData("Slow mode", slow);
        telemetry.addData("Field centric", fieldCentric);
    }

    // ---- turret + flywheel (linked) ----

    private void handleAimAndSpin() {
        if (operatorJustPressed(RobotControls.AIM_AND_SPIN_BUTTON)) {
            aimingAndSpinning = !aimingAndSpinning;
            if (!aimingAndSpinning) {
                launcher.stop();
                turret.stop();
            }
        }
        if (aimingAndSpinning) {
            turret.aimToObject(poseX(), poseY(), poseHeading());
            launcher.dynamicMotorPower(poseX(), poseY());
        }
        telemetry.addData("Aiming+Spinning", aimingAndSpinning);
        telemetry.addData("Shooter RPM (target)", launcher.getRPM());
        telemetry.addData("Turret deg", turret.getCurrentAngle());
    }

    // ---- intake / transfer / launchCap (one feed path) ----

    private void handleFeed() {
        boolean unclog = operatorHeld(RobotControls.UNCLOG_BUTTON);
        boolean collect = !unclog && operatorHeld(RobotControls.COLLECT_BUTTON);
        if (operatorHeld(RobotControls.RAMP_BUTTON)) {
            rampServo.open();
        } else {
            rampServo.close();
        }
        if (unclog) {
            // Unclog wins over everything: reverse the path, force launchCap shut.
            intake.reverseMotor();
            transfer.runMotorAt(RobotConfig.Transfer.REVERSE_POWER);
            launchCap.close();
            firingUntilS = 0.0;
        } else if (collect) {
            intake.startMotor();
            transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
            if (RobotControls.COLLECT_OPENS_KICKER) {
                launchCap.open();
            }
            firingUntilS = 0.0;
        } else {
            intake.stopMotor();
            transfer.stopMotor();

            if (operatorJustPressed(RobotControls.FIRE_BUTTON)) {
                launchCap.open();
                firingUntilS = getRuntime() + Math.max(0.0, RobotControls.FIRE_KICK_SECONDS);
            } else if (firingUntilS != 0.0) {
                if (getRuntime() >= firingUntilS) {
                    launchCap.close();
                    firingUntilS = 0.0;
                }
            } else {
                // Safe default: launchCap stays shut when idle (e.g. after COLLECT
                // is released) so staged balls can't dribble into the wheel.
                launchCap.close();
            }
        }

        telemetry.addData("Collecting", collect);
        telemetry.addData("Unclogging", unclog);
        telemetry.addData("launchCap open", launchCap.isOpen());
    }

    private void updateShooterGoal(){
        // Push the (possibly re-switched / dashboard-retuned) goal every loop
        // so turret + shooter track it with no restart.
        if (isTilt()){
            switchGoalForHiveTip();
        }
        if (activeGoal != null) {
            launcher.setGoal(activeGoal);
            turret.setGoal(activeGoal);
        }
    }

    private void handleUmbrella(){
        if (operatorJustPressed(RobotControls.UMBRELLA_BUTTON) && !toggledUmbrella){
            umbrellaServo.open();
            toggledUmbrella = !toggledUmbrella;
        }
        else if (operatorJustPressed(RobotControls.UMBRELLA_BUTTON) && toggledUmbrella){
            umbrellaServo.close();
            toggledUmbrella = !toggledUmbrella;
        }
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
        telemetry.addData("Controls",
                "Slow=%s Field=%s | Collect=%s AimSpin=%s Fire=%s Unclog=%s",
                RobotControls.SLOW_BUTTON, RobotControls.FIELD_CENTRIC_BUTTON,
                RobotControls.COLLECT_BUTTON, RobotControls.AIM_AND_SPIN_BUTTON,
                RobotControls.FIRE_BUTTON, RobotControls.UNCLOG_BUTTON);
        telemetry.addData("Start", "%s (%.1f, %.1f, %.0f deg)",
                getStart(), getStart().x(), getStart().y(),
                getStart().headingDeg());
        if (activeGoal != null) {
            telemetry.addData("Goal", "%s (%.1f, %.1f, h=%.1f) -> %s",
                    activeGoal, activeGoal.x(), activeGoal.y(),
                    activeGoal.height(), activeGoal.cell());
        }
        panels.debug("position", follower.pose());
        panels.debug("velocity", follower.velocity());
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

    public boolean isTilt(){
        HiveCell target = getTargetCell();
        double pitch = aprilTags.getClusterPitch(target);
        boolean visible = pitch != AprilTagLocalization.UNKNOWN_PITCH;
        return visible && tiltEstimator.isTilted(pitch);
    }
}
