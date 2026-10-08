package org.firstinspires.ftc.teamcode.Servo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Colour sensor that sets the shooter compression servo.
 * Red or blue opens it (wider), then it holds open until yellow is seen,
 * which sends it back to the default position.
 */
@Configurable
public class ColourServo {

    // Compression positions, tunable from Panels
    public static double OPEN_POSITION = 1.0;    // wider, less compression
    public static double DEFAULT_POSITION = 0.5; // resting position
    public static double CLOSED_POSITION = 0.0;  // narrowest, not used by update()

    private final NormalizedColorSensor colorSensor;
    private final ServoComponent compression;

    public enum detectedColor {RED, BLUE, YELLOW, UNKNOWN}

    public ColourServo(HardwareMap hardwareMap, String servoID, String sensorID) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorID);
        compression = new ServoComponent(hardwareMap, servoID,
                OPEN_POSITION,
                CLOSED_POSITION,
                DEFAULT_POSITION);
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

    public detectedColor update(Telemetry telemetry) {
        detectedColor colour = getDetectedColor(telemetry);

        if (colour == detectedColor.RED || colour == detectedColor.BLUE) {
            compression.open();                 // wider, stays open until yellow
        } else if (colour == detectedColor.YELLOW && compression.isOpen()) {
            compression.toDefault();            // back to default, not closed
        }
        // UNKNOWN, or yellow while not open: do nothing, servo holds position

        telemetry.addData("colour", colour);
        telemetry.addData("compression", compression.isOpen() ? "OPEN (waiting for yellow)" : "DEFAULT");
        return colour;
    }
}