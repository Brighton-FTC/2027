package org.firstinspires.ftc.teamcode.FlyWheel;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class FlyWheelMotorComponent {

    private final Motor motor;

    public FlyWheelMotorComponent(HardwareMap hardwareMap, String motorID) {
        motor = new Motor(hardwareMap, motorID);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void runMotorAt(double power) {
        motor.set(power);
    }

    public void stopMotor() {
        motor.stopMotor();
    }
}
