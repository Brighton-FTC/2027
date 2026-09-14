package org.firstinspires.ftc.teamcode.FlyWheel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Kicker servo that pushes a ball into the spinning flywheel.
 * Positions come from {@link RobotConfig.Kicker}.
 */
public class ServoKickComponent {

    private final Servo servo;
    private boolean open = false;

    public ServoKickComponent(HardwareMap hardwareMap, String servoId) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(servoId, "servoId");
        servo = hardwareMap.get(Servo.class, servoId);
        close();
    }

    public void open() {
        servo.setPosition(clamp(RobotConfig.Kicker.OPEN_POSITION));
        open = true;
    }

    public void close() {
        servo.setPosition(clamp(RobotConfig.Kicker.CLOSED_POSITION));
        open = false;
    }

    /** Toggles and returns the new state (true = open). */
    public boolean toggle() {
        if (open) {
            close();
        } else {
            open();
        }
        return open;
    }

    public boolean isOpen() {
        return open;
    }

    private static double clamp(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
