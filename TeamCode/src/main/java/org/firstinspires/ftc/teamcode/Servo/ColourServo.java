package org.firstinspires.ftc.teamcode.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColourServo {
    private final NormalizedColorSensor colorSensor;
    private final ServoComponent compression;

    public enum detectedColor {RED, BLUE, YELLOW, UNKNOWN}

    public ColourServo(HardwareMap hardwareMap, String servoID, String sensorID) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorID);
        compression = new ServoComponent(hardwareMap, servoID);
    }

    public detectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        if (normRed > 0.35 && normGreen > 0.35 && normBlue < 0.2) return detectedColor.YELLOW;
        if (normRed > 0.35 && normGreen < 0.3 && normBlue < 0.3) return detectedColor.RED;
        if (normBlue > 0.35 && normRed < 0.3) return detectedColor.BLUE;

        return detectedColor.UNKNOWN;
    }

    public void update(Telemetry telemetry) {
        detectedColor colour = getDetectedColor(telemetry);
        if (colour == detectedColor.RED || colour == detectedColor.BLUE) compression.open();  // wider, less compression
        else if (colour == detectedColor.YELLOW) compression.close();                         // narrower, more compression
        telemetry.addData("colour", colour);
    }
}
