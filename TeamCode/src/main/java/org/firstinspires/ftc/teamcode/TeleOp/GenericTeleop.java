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
import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shared TeleOp implementation. Subclasses only provide the alliance-specific
 * goal X and starting pose — all tuning lives in {@link RobotConfig}.
 *
 * <p>Controls (gamepad1 = drive + shooter, gamepad2 = intake/transfer/kicker):
 * <ul>
 *   <li>Left stick: drive/strafe, right stick X: turn.</li>
 *   <li>RIGHT_BUMPER: slow-mode toggle. SQUARE: field/robot-centric toggle.</li>
 *   <li>LEFT_BUMPER: turret auto-aim toggle. CIRCLE (gp1): shooter toggle.</li>
 *   <li>CIRCLE (gp2): intake toggle. TRIANGLE (gp2): reverse intake+transfer toggle.</li>
 *   <li>CROSS (gp2): transfer toggle. DPAD_LEFT (gp1): intake+transfer+kicker combo.</li>
 *   <li>DPAD_UP/DOWN (gp2): kicker open/close.</li>
 * </ul>
 */
public abstract class GenericTeleop extends OpMode {

    protected Follower follower;
    protected TelemetryManager panels;

    private GamepadEx gp1;
    private GamepadEx gp2;

    private TurretPIDComponent turret;
    private DynamicAngleComponent launcher;
    private FlyWheelMotorComponent transfer;
    private ServoKickComponent kicker;
    private IntakeMotorComponent intake;

    private Pose startingPose;

    // Toggle state — each flag mirrors its mechanism, applied immediately.
    private boolean slowMode = false;
    private boolean fieldCentric;
    private boolean aiming = false;
    private boolean shooting = false;
    private boolean intakeRunning = false;
    private boolean transferRunning = false;

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

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        turret.resetTurretEncoder();
    }

    @Override
    public void start() {
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        gp1.readButtons();
        gp2.readButtons();

        handleDrive();
        handleAim();
        handleShooter();
        handleIntakeAndTransfer();
        handleKicker();
        reportTelemetry();

        panels.update();
        telemetry.update();
    }

    // ---- drive ----

    private void handleDrive() {
        double scale = slowMode ? RobotConfig.Drive.SLOW_MODE_MULTIPLIER : 1.0;
        double forward = gp1.getLeftY() * scale;
        double strafe = -gp1.getLeftX() * scale;
        double turn = -gp1.getRightX() * scale;

        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward, strafe, turn, follower.pose().heading());
            follower.manual(powers);
        } else {
            follower.manual(forward, strafe, turn);
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            slowMode = !slowMode;
        }
        if (gp1.wasJustPressed(PSButtons.SQUARE)) {
            fieldCentric = !fieldCentric;
        }
    }

    // ---- turret + shooter ----

    private void handleAim() {
        if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            aiming = !aiming;
        }
        if (aiming) {
            turret.aimToObject(poseX(), poseY(), poseHeading());
        }
    }

    private void handleShooter() {
        if (gp1.wasJustPressed(PSButtons.CIRCLE)) {
            shooting = !shooting;
            if (!shooting) {
                launcher.stop();
            }
        }
        if (shooting) {
            launcher.dynamicMotorPower(poseX(), poseY());
        }
    }

    // ---- intake / transfer / kicker ----

    private void handleIntakeAndTransfer() {
        // Reverse intake + reverse transfer (unclog).
        if (gp2.wasJustPressed(PSButtons.TRIANGLE)) {
            if (intakeRunning || transferRunning) {
                intake.stopMotor();
                transfer.stopMotor();
                intakeRunning = false;
                transferRunning = false;
            } else {
                intake.reverseMotor();
                transfer.runMotorAt(RobotConfig.Transfer.REVERSE_POWER);
                intakeRunning = true;
                transferRunning = true;
            }
        }

        // Transfer roller alone.
        if (gp2.wasJustPressed(PSButtons.CROSS)) {
            transferRunning = !transferRunning;
            if (transferRunning) {
                transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
            } else {
                transfer.stopMotor();
            }
        }

        // Intake roller alone.
        if (gp2.wasJustPressed(PSButtons.CIRCLE)) {
            intakeRunning = !intakeRunning;
            if (intakeRunning) {
                intake.startMotor();
            } else {
                intake.stopMotor();
            }
        }

        // Combo: intake + transfer + kicker open (one-button collect-and-feed).
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (intakeRunning || transferRunning) {
                intake.stopMotor();
                transfer.stopMotor();
                kicker.close();
                intakeRunning = false;
                transferRunning = false;
            } else {
                intake.startMotor();
                transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER);
                kicker.open();
                intakeRunning = true;
                transferRunning = true;
            }
        }
    }

    private void handleKicker() {
        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            kicker.open();
        }
        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            kicker.close();
        }
    }

    // ---- telemetry ----

    private void reportTelemetry() {
        telemetry.addData("Shooter RPM (target)", launcher.getRPM());
        telemetry.addData("Turret deg", turret.getCurrentAngle());
        telemetry.addData("Aiming", aiming);
        telemetry.addData("Shooting", shooting);
        telemetry.addData("Slow mode", slowMode);
        telemetry.addData("Field centric", fieldCentric);
        telemetry.addData("Intake", intakeRunning);
        telemetry.addData("Transfer", transferRunning);
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
