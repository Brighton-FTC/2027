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
 * ahead, + = left). Commands are software-clamped to
 * {@code +/- MAX_ANGLE_DEG} so the turret can never wrap cables.
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
     * Rotates the turret by a relative offset in degrees.
     * The resulting absolute target is clamped to the software end-stops.
     */
    public void turnTurretBy(double deltaDegrees) {
        applyGains();
        double targetAngle = clampAngle(getCurrentAngle() + deltaDegrees);
        driveToAngle(targetAngle);
    }

    /**
     * Aims at the goal from the given robot pose (inches + heading radians,
     * field frame). If the goal is behind the robot the turret holds at the
     * nearest end-stop instead of spinning around.
     */
    public void aimToObject(double robotX, double robotY, double robotHeadingRad) {
        if (isUnknownPose(robotX, robotY)) {
            return;
        }
        double robotDeg = Math.toDegrees(robotHeadingRad);
        double goalBearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        double relative = normalize180(goalBearing - robotDeg);
        double clampedTarget = clampAngle(relative);
        double toTurn = clampedTarget - getCurrentAngle();

        telemetry.addData("Turret/toTurnDeg", toTurn);
        telemetry.addData("Turret/relativeDeg", relative);
        telemetry.addData("Turret/targetDeg", clampedTarget);
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
        double clampedTarget = clampAngle(relative);
        return Math.abs(clampedTarget - getCurrentAngle()) <= RobotConfig.Turret.AIM_TOLERANCE_DEG;
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

    private static double clampAngle(double angleDeg) {
        double limit = Math.abs(RobotConfig.Turret.MAX_ANGLE_DEG);
        return Math.max(-limit, Math.min(limit, angleDeg));
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
