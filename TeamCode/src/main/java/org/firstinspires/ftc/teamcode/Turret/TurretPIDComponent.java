package org.firstinspires.ftc.teamcode.Turret;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Turret yaw controlled by a PIDF loop on encoder ticks.
 *
 * <p>Angles are in degrees, relative to the robot forward (0 = straight
 * ahead, + = left). Turn commands are NOT end-stop clamped: the wrap-around
 * logic keeps the accumulated rotation within +/- 180 deg, and when a
 * command would push the turret past that seam the command is shunted by
 * +/- 360 deg so the turret turns the other way around to the equivalent
 * angle instead of stopping at a maximum angle.
 *
 * <p>Gains and limits live in {@link RobotConfig.Turret} and are re-applied
 * on every update, so dashboard edits take effect immediately.
 *
 * <p>This class only writes telemetry keys — it never calls
 * {@code telemetry.update()}; the OpMode owns the update loop.
 */
@Configurable
public class TurretPIDComponent {

    private final Motor turretMotor;
    private final PIDFController controller = new PIDFController(0, 0, 0, 0);
    private final Telemetry telemetry;

    private final double goalX;
    private final double goalY;
    private final double degreesPerTick;

    // Reference-style wrap-around tracking: turretGlobalAngle accumulates the
    // total rotation since power-on (unwrapped from the encoder via deltas)
    // and must stay within +/- 180 deg. Commands that would cross that seam
    // are shunted by +/- 360 deg so the turret turns the other way around
    // instead of stopping at a maximum angle.
    private double lastTurretAngle = 0;
    private double turretGlobalAngle = 0;

    /** Uses hardware/field constants from {@link RobotConfig}. */
    public TurretPIDComponent(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap,
                RobotConfig.Hardware.TURRET_MOTOR,
                RobotConfig.Turret.DEGREES_PER_TICK,
                RobotConfig.Field.GOAL_X,
                RobotConfig.Field.GOAL_Y,
                telemetry);
    }

    public TurretPIDComponent(HardwareMap hardwareMap, String motorId,
                              double degreesPerTick,
                              double goalX, double goalY,
                              Telemetry telemetry) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(motorId, "motorId");
        Objects.requireNonNull(telemetry, "telemetry");
        if (!(degreesPerTick > 0)) {
            throw new IllegalArgumentException("degreesPerTick must be > 0");
        }
        this.turretMotor = new Motor(hardwareMap, motorId);
        this.turretMotor.stopAndResetEncoder();
        this.turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.turretMotor.setRunMode(Motor.RunMode.RawPower);
        this.degreesPerTick = degreesPerTick;
        this.goalX = goalX;
        this.goalY = goalY;
        this.telemetry = telemetry;
        applyGains();
    }

    public void resetTurretEncoder() {
        turretMotor.stopAndResetEncoder();
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /** Current turret angle in degrees, relative to robot forward. */
    public double getCurrentAngle() {
        return encoderTicksToAngle(turretMotor.getCurrentPosition());
    }

    public double encoderTicksToAngle(int ticks) {
        return ticks * degreesPerTick;
    }

    public int angleToEncoderTicks(double degrees) {
        return (int) Math.round(degrees / degreesPerTick);
    }

    /**
     * Rotates the turret by a relative offset in degrees using the
     * wrap-around turning logic: the command is shunted by +/- 360 deg when
     * it would push the accumulated rotation past +/- 180 deg, so the turret
     * turns the other way around instead of stopping at an end-stop.
     */
    public void turnTurretBy(double deltaDegrees) {
        applyGains();
        trackRotation();
        double toTurn = wrapTurn(deltaDegrees);
        driveToAngle(getCurrentAngle() + toTurn);
    }

    /**
     * Aims at the goal from the given robot pose (inches + heading radians,
     * field frame). When the goal crosses the +/- 180 deg seam the turret
     * turns the other way around to the equivalent angle instead of stopping
     * at a maximum angle.
     */
    public void aimToObject(double robotX, double robotY, double robotHeadingRad) {
        if (isUnknownPose(robotX, robotY)) {
            return;
        }
        double robotDeg = Math.toDegrees(robotHeadingRad);
        double goalBearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        double relative = normalize180(goalBearing - robotDeg);
        double toTurn = relative - getCurrentAngle();

        telemetry.addData("Turret/toTurnDeg", toTurn);
        telemetry.addData("Turret/relativeDeg", relative);
        telemetry.addData("Turret/globalAngleDeg", turretGlobalAngle);
        telemetry.addData("Turret/targetDeg", getCurrentAngle() + toTurn);
        telemetry.addData("Turret/currentDeg", getCurrentAngle());
        telemetry.addData("Turret/errorTicks", controller.getPositionError());

        turnTurretBy(toTurn);
    }

    /** True when the turret is within tolerance of the goal bearing. */
    public boolean isAimed(double robotX, double robotY, double robotHeadingRad) {
        if (isUnknownPose(robotX, robotY)) {
            return false;
        }
        double robotDeg = Math.toDegrees(robotHeadingRad);
        double goalBearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        double relative = normalize180(goalBearing - robotDeg);
        double toTurn = wrapTurn(relative - getCurrentAngle());
        return Math.abs(toTurn) <= RobotConfig.Turret.AIM_TOLERANCE_DEG;
    }

    public double getPIDSetPoint() {
        return controller.getSetPoint();
    }

    // ---- internals ----

    private void driveToAngle(double targetAngleDeg) {
        double targetTicks = angleToEncoderTicks(targetAngleDeg);
        controller.setSetPoint(targetTicks);
        double power = clampPower(controller.calculate(turretMotor.getCurrentPosition()));
        telemetry.addData("Turret/power", power);
        turretMotor.set(power);
    }

    private void applyGains() {
        controller.setPIDF(
                RobotConfig.Turret.KP,
                RobotConfig.Turret.KI,
                RobotConfig.Turret.KD,
                RobotConfig.Turret.KF);
    }

    /**
     * Updates the accumulated rotation tracking (~turretGlobalAngle in the
     * reference implementation): the unwrapped total rotation since power-on,
     * built from per-sample encoder deltas.
     */
    private void trackRotation() {
        double turretAngle = getCurrentAngle();
        turretGlobalAngle += turretAngle - lastTurretAngle;
        lastTurretAngle = turretAngle;
    }

    /**
     * Wrap-around turn logic (mirrors the reference implementation):
     * if the command would push the accumulated rotation past +/- 180 deg,
     * shunt it by 360 deg so the turret turns the other way around to the
     * equivalent angle instead of stopping at the maximum angle.
     */
    private double wrapTurn(double toTurnDeg) {
        if (Math.abs(turretGlobalAngle + toTurnDeg) >= 180 && toTurnDeg > 0) {
            toTurnDeg -= 360;
        } else if (Math.abs(turretGlobalAngle + toTurnDeg) >= 180 && toTurnDeg < 0) {
            toTurnDeg += 360;
        }
        return toTurnDeg;
    }

    private static double clampPower(double power) {
        double limit = Math.abs(RobotConfig.Turret.MAX_POWER);
        return Math.max(-limit, Math.min(limit, power));
    }

    private static double normalize180(double angleDeg) {
        while (angleDeg > 180) {
            angleDeg -= 360;
        }
        while (angleDeg <= -180) {
            angleDeg += 360;
        }
        return angleDeg;
    }

    private static boolean isUnknownPose(double x, double y) {
        double sentinel = RobotConfig.Launcher.UNKNOWN_POSE;
        return x == sentinel || y == sentinel;
    }
}
