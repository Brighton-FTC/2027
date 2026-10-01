package org.firstinspires.ftc.teamcode.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Servo with open, closed and default positions. */
public class ServoComponent {

    private final Servo servo;
    private final double openPos;
    private final double closedPos;
    private final double defaultPos;
    private boolean open = false;

    /** Kicker servo, using RobotConfig.Kicker positions. Starts closed. */
    public ServoComponent(HardwareMap hardwareMap, String servoId) {
        this(hardwareMap, servoId,
                RobotConfig.Kicker.OPEN_POSITION,
                RobotConfig.Kicker.CLOSED_POSITION,
                RobotConfig.Kicker.CLOSED_POSITION);
    }

    /** Any servo with its own positions. Starts at default. */
    public ServoComponent(HardwareMap hardwareMap, String servoId,
                          double openPos, double closedPos, double defaultPos) {
        servo = hardwareMap.get(Servo.class, servoId);
        this.openPos = openPos;
        this.closedPos = closedPos;
        this.defaultPos = defaultPos;
        toDefault();
    }

    public void open() {
        servo.setPosition(openPos);
        open = true;
    }

    public void close() {
        servo.setPosition(closedPos);
        open = false;
    }

    public void toDefault() {
        servo.setPosition(defaultPos);
        open = false;
    }

    public boolean toggle() {
        if (open) close();
        else open();
        return open;
    }

    public boolean isOpen() {
        return open;
    }
}