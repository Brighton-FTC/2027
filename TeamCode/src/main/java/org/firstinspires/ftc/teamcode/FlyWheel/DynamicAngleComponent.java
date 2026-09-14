package org.firstinspires.ftc.teamcode.FlyWheel;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Ballistic shooter: converts robot pose into a flywheel speed that lobs the
 * ball into the goal at a fixed launch angle.
 *
 * <p>Assumes the turret already faces the goal, so only 2D range matters.
 * Geometry and physics constants live in {@link RobotConfig.Field} and
 * {@link RobotConfig.Launcher} — tune there, not here.
 *
 * <p>NOTE on units: the computed wheel speed (RPM) is passed straight to
 * {@link FlyWheelMotorPIDComponent} as its velocity setpoint. Keep the PID
 * gains matched to that convention (see dashboard tuning).
 */
public class DynamicAngleComponent {

    private final FlyWheelMotorPIDComponent flywheel;
    private final double goalX;
    private final double goalY;
    private final double goalHeight;

    private double targetRpm = 0.0;

    /** Uses hardware/field constants from {@link RobotConfig}. */
    public DynamicAngleComponent(HardwareMap hardwareMap) {
        this(hardwareMap,
                RobotConfig.Hardware.FLYWHEEL_MOTOR,
                RobotConfig.Field.GOAL_X,
                RobotConfig.Field.GOAL_Y,
                RobotConfig.Field.GOAL_HEIGHT);
    }

    public DynamicAngleComponent(HardwareMap hardwareMap, String flywheelMotorId,
                                 double goalX, double goalY, double goalHeight) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(flywheelMotorId, "flywheelMotorId");
        this.flywheel = new FlyWheelMotorPIDComponent(hardwareMap, flywheelMotorId);
        this.goalX = goalX;
        this.goalY = goalY;
        this.goalHeight = goalHeight;
    }

    /**
     * Updates flywheel speed for the given robot pose (inches, field frame).
     * If the pose is unknown ({@link RobotConfig.Launcher#UNKNOWN_POSE}) or the
     * goal is unreachable at the fixed angle, the wheel is commanded to 0.
     */
    public void dynamicMotorPower(double robotX, double robotY) {
        if (isUnknownPose(robotX, robotY)) {
            commandRpm(0.0);
            return;
        }

        double range = Math.hypot(goalX - robotX, goalY - robotY);
        double launchAngleRad = Math.toRadians(RobotConfig.Launcher.FIXED_LAUNCH_ANGLE_DEG);
        double gravity = RobotConfig.Launcher.GRAVITY_IN_PER_S2;

        double denominator = range * Math.tan(launchAngleRad) - goalHeight;
        if (denominator <= 0 || range < 1e-6) {
            // No real velocity solution at this angle — hold the wheel stopped.
            commandRpm(0.0);
            return;
        }

        double cos = Math.cos(launchAngleRad);
        double exitVelocity = Math.sqrt((gravity * range * range) / (2 * cos * cos * denominator));
        double adjustedVelocity = exitVelocity / Math.max(1e-6, RobotConfig.Launcher.EFFICIENCY);

        // Linear (in/s) -> wheel (rev/min) via v = omega * r.
        double rpm = (60.0 / (2.0 * Math.PI * RobotConfig.Launcher.FLYWHEEL_RADIUS_IN)) * adjustedVelocity;
        rpm = Math.max(0.0, Math.min(RobotConfig.Launcher.MAX_RPM, rpm));
        commandRpm(rpm);
    }

    public void stop() {
        commandRpm(0.0);
        flywheel.stopMotor();
    }

    /** Last commanded wheel speed in RPM (0 when holding stopped). */
    public double getRPM() {
        return targetRpm;
    }

    /** Last commanded wheel speed in RPM. */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** Current wheel velocity in encoder ticks/s. */
    public double getWheelVelocity() {
        return flywheel.getVelocity();
    }

    private void commandRpm(double rpm) {
        targetRpm = rpm;
        flywheel.runMotorAt(rpm);
    }

    private static boolean isUnknownPose(double x, double y) {
        double sentinel = RobotConfig.Launcher.UNKNOWN_POSE;
        return x == sentinel || y == sentinel;
    }
}
