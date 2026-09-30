package org.firstinspires.ftc.teamcode.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Servo.ColourServo;

@TeleOp(name = "Colour Servo Test", group = "Test")
public class ColourServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ColourServo colourServo = new ColourServo(hardwareMap, "compression", "sensor_color_distance");

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            colourServo.update(telemetry);
            telemetry.update();
        }
    }
}