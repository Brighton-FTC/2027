package org.firstinspires.ftc.teamcode.FlyWheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class ServoKickComponent {

    private final Servo servo;

    public static double openPosition = 1.0;
    public static double closedPosition = 0.0;

    public ServoKickComponent(HardwareMap hardwareMap, String servoID) {
        servo = hardwareMap.get(Servo.class, servoID);
        close();
    }

    public void open() {
        servo.setPosition(openPosition);
    }

    public void close() {
        servo.setPosition(closedPosition);
    }
}
