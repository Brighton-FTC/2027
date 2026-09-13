package org.firstinspires.ftc.teamcode.FlyWheel;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class FlyWheelMotorPIDComponent {

    private final Motor motor;

    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF =  0.00065;

    private double power;
    private PIDFController controller = new PIDFController(kP, kI, kD, kF);



    public FlyWheelMotorPIDComponent(HardwareMap hardwareMap, String motorID){
        motor = new Motor(hardwareMap, motorID);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void runMotorAt(double velocity){
        controller.setSetPoint(velocity);
        power = controller.calculate(getVel());
        motor.set(power);

    }

    public double getVel(){
        return motor.getCorrectedVelocity();
    }

    public double getPower(){return power;}

    public void stopMotor(){
        motor.stopMotor();
    }
}