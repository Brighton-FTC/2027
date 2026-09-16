package org.firstinspires.ftc.teamcode.transfer;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class TransferComponent {

    private final Motor motor;
    public TransferComponent(HardwareMap hardwareMap, String transferMotorID){
        motor = new Motor(hardwareMap, transferMotorID);
    }

    public void runTransferAt(double power){
        motor.set(power);
    }

    public void stopTransfer(){
        motor.stopMotor();
    }

}
