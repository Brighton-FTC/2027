package org.firstinspires.ftc.teamcode.FlyWheel;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Velocity-controlled flywheel using FTCLib {@link PIDFController}.
 *
 * <p>Gains live in {@link RobotConfig.Flywheel} so they are tunable from
 * the dashboard. They are re-applied on every update, so dashboard edits
 * take effect immediately.
 */
@Configurable
public class FlyWheelMotorPIDComponent {

    private final Motor motor;
    private final PIDFController controller = new PIDFController(0, 0, 0, 0);

    private double targetVelocity = 0.0;
    private double lastPower = 0.0;

    public FlyWheelMotorPIDComponent(HardwareMap hardwareMap, String motorId) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(motorId, "motorId");
        motor = new Motor(hardwareMap, motorId);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        applyGains();
    }

    /**
     * Spins the wheel toward the requested velocity (encoder ticks/s).
     * Output power is clamped to {@code +/- MAX_POWER}.
     */
    public void runMotorAt(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        applyGains();
        controller.setSetPoint(targetVelocity);
        double power = controller.calculate(getVelocity());
        lastPower = clamp(power, RobotConfig.Flywheel.MAX_POWER);
        motor.set(lastPower);
    }

    /** Cuts power and clears the PID setpoint. Call every loop while shooting. */
    public void stopMotor() {
        targetVelocity = 0.0;
        lastPower = 0.0;
        controller.setSetPoint(0.0);
        motor.stopMotor();
    }

    /** Current wheel velocity in encoder ticks/s. */
    public double getVelocity() {
        return motor.getCorrectedVelocity();
    }

    /** Last power sent to the motor. */
    public double getPower() {
        return lastPower;
    }

    /** Requested velocity in encoder ticks/s. */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** True when the wheel is close to stopped. */
    public boolean isStopped() {
        return Math.abs(getVelocity()) < RobotConfig.Flywheel.STOPPED_VELOCITY;
    }

    private void applyGains() {
        controller.setPIDF(
                RobotConfig.Flywheel.KP,
                RobotConfig.Flywheel.KI,
                RobotConfig.Flywheel.KD,
                RobotConfig.Flywheel.KF);
    }

    private static double clamp(double value, double limit) {
        double bound = Math.abs(limit);
        return Math.max(-bound, Math.min(bound, value));
    }
}
