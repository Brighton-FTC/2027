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
import org.firstinspires.ftc.teamcode.FlyWheel.ServoKickComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotControls;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shared TeleOp implementation. Subclasses only provide the alliance-specific
 * goal X and starting pose — all tuning lives in {@link RobotConfig} and all
 * bindings live in {@link RobotControls}.
 *
 * <p>Simplified controls (see {@link RobotControls} to rebind):
 * <ul>
 *   <li>Driver: left stick = drive/strafe, right stick X = turn,
 *       hold SLOW for slow mode, press FIELD_CENTRIC to toggle centricity.</li>
 *   <li>Operator: hold COLLECT to run intake + transfer + kicker together,
 *       press AIM_AND_SPIN to toggle turret auto-aim + flywheel together,
 *       press FIRE to kick one shot, hold UNCLOG to reverse the feed.</li>
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
    private ServoKickComponent kicker;
    private IntakeMotorComponent intake;

    private Pose startingPose;

    // Latched state. Hold-to-run actions (collect/unclog) intentionally keep no
    // latch — they are recomputed from the gamepad every loop.
    private boolean fieldCentric;
    private boolean slowModeToggle = false;
    private boolean aimingAndSpinning = false;
    private double firingUntilS = 0.0;

    /** Goal X in inches (field frame). Y/height come from {@link RobotConfig}. */
    protected abstract double getGoalX();

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
        double goalY = RobotConfig.Field.GOAL_Y;

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
                RobotConfig.Field.GOAL_HEIGHT);
        transfer = new FlyWheelMotorComponent(hardwareMap, RobotConfig.Hardware.TRANSFER_MOTOR);
        kicker = new ServoKickComponent(hardwareMap, RobotConfig.Hardware.LAUNCH_CAP_SERVO);
        intake = new IntakeMotorComponent(hardwareMap, RobotConfig.Hardware.INTAKE_MOTOR);

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

        handleDrive();
        handleAimAndSpin();
        handleFeed();
        reportTelemetry();

        panels.update();
        telemetry.update();
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

    // ---- intake / transfer / kicker (one feed path) ----

    private void handleFeed() {
        boolean unclog = operatorHeld(RobotControls.UNCLOG_BUTTON);
        boolean collect = !unclog && operatorHeld(RobotControls.COLLECT_BUTTON);

        if (unclog) {
            // Unclog wins over everything: reverse the path, force kicker shut.
            intake.reverseMotor();
            transfer.runMotorAt(RobotConfig.Transfer.REVERSE_POWER);
            kicker.close();
            firingUntilS = 0.0;
        } else if (collect) {
            intake.startMotor();
            transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
            if (RobotControls.COLLECT_OPENS_KICKER) {
                kicker.open();
            }
            firingUntilS = 0.0;
        } else {
            intake.stopMotor();
            transfer.stopMotor();

            if (operatorJustPressed(RobotControls.FIRE_BUTTON)) {
                kicker.open();
                firingUntilS = getRuntime() + Math.max(0.0, RobotControls.FIRE_KICK_SECONDS);
            } else if (firingUntilS != 0.0) {
                if (getRuntime() >= firingUntilS) {
                    kicker.close();
                    firingUntilS = 0.0;
                }
            } else {
                // Safe default: kicker stays shut when idle (e.g. after COLLECT
                // is released) so staged balls can't dribble into the wheel.
                kicker.close();
            }
        }

        telemetry.addData("Collecting", collect);
        telemetry.addData("Unclogging", unclog);
        telemetry.addData("Kicker open", kicker.isOpen());
    }

    // ---- telemetry ----

    private void reportTelemetry() {
        telemetry.addData("Controls",
                "Slow=%s Field=%s | Collect=%s AimSpin=%s Fire=%s Unclog=%s",
                RobotControls.SLOW_BUTTON, RobotControls.FIELD_CENTRIC_BUTTON,
                RobotControls.COLLECT_BUTTON, RobotControls.AIM_AND_SPIN_BUTTON,
                RobotControls.FIRE_BUTTON, RobotControls.UNCLOG_BUTTON);
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
}
