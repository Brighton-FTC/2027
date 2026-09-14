package org.firstinspires.ftc.teamcode.FlyWheel;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

/**
 * Simple open-loop roller (e.g. transfer). For the velocity-controlled
 * shooter wheel see {@link FlyWheelMotorPIDComponent}.
 */
public class FlyWheelMotorComponent {

    private final Motor motor;
    private double lastPower = 0.0;

    public FlyWheelMotorComponent(HardwareMap hardwareMap, String motorId) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(motorId, "motorId");
        motor = new Motor(hardwareMap, motorId);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    /** Sets power, clamped to [-1, 1]. */
    public void runMotorAt(double power) {
        lastPower = clamp(power);
        motor.set(lastPower);
    }

    public void stopMotor() {
        lastPower = 0.0;
        motor.stopMotor();
    }

    /** Last commanded power (0 after {@link #stopMotor()}). */
    public double getPower() {
        return lastPower;
    }

    private static double clamp(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
