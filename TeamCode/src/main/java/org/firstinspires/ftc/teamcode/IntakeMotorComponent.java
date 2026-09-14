package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Intake roller. Powers live in {@link RobotConfig.Intake}.
 */
public class IntakeMotorComponent {

    private final Motor motor;
    private double lastPower = 0.0;

    public IntakeMotorComponent(HardwareMap hardwareMap, String motorId) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(motorId, "motorId");
        motor = new Motor(hardwareMap, motorId);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void startMotor() {
        setPower(RobotConfig.Intake.INTAKE_POWER);
    }

    public void reverseMotor() {
        setPower(RobotConfig.Intake.REVERSE_POWER);
    }

    public void stopMotor() {
        lastPower = 0.0;
        motor.stopMotor();
    }

    /** Direct power control, clamped to [-1, 1]. */
    public void setPower(double power) {
        lastPower = Math.max(-1.0, Math.min(1.0, power));
        motor.set(lastPower);
    }

    /** Last commanded power (0 after {@link #stopMotor()}). */
    public double getPower() {
        return lastPower;
    }

    public boolean isRunning() {
        return lastPower != 0.0;
    }
}
