package org.firstinspires.ftc.teamcode.Servo;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Colour Compression Test (No Servo OK)", group = "Test")
public class ColourCompressionTest extends LinearOpMode {

    // Device names from your robot configuration
    private static final String SENSOR_NAME = "colorSensor";
    private static final String SERVO_NAME  = "compressionServo";

    // Compression positions (tune these on the real robot)
    private static final double WIDE_POS    = 0.80; // red or blue
    private static final double NARROW_POS  = 0.40; // yellow
    private static final double DEFAULT_POS = 0.60; // nothing detected

    private static final float SENSOR_GAIN = 2.0f;

    // A sample counts as "present" if it's closer than this
    private static final double PRESENT_CM = 4.0;

    // Hue ranges (0 to 360). Tune using the telemetry readings.
    private static final float RED_MAX_HUE      = 30;   // red is 0 to 30 ...
    private static final float RED_WRAP_HUE     = 330;  // ... or 330 to 360
    private static final float YELLOW_MIN_HUE   = 30;
    private static final float YELLOW_MAX_HUE   = 110;
    private static final float BLUE_MIN_HUE     = 180;
    private static final float BLUE_MAX_HUE     = 260;

    private enum SampleColour { RED, BLUE, YELLOW, NONE }

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor; // null if not supported
    private Servo compressionServo;        // null if not found

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, SENSOR_NAME);
        colorSensor.setGain(SENSOR_GAIN);

        if (colorSensor instanceof DistanceSensor) {
            distanceSensor = (DistanceSensor) colorSensor;
        }

        try {
            compressionServo = hardwareMap.get(Servo.class, SERVO_NAME);
        } catch (Exception e) {
            compressionServo = null;
        }

        telemetry.addData("Servo", compressionServo == null
                ? "NOT FOUND, running in simulation mode"
                : "found");
        telemetry.addData("Distance", distanceSensor == null
                ? "not supported, using brightness instead"
                : "supported");
        telemetry.update();

        waitForStart();

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            NormalizedRGBA rgba = colorSensor.getNormalizedColors();
            Color.colorToHSV(rgba.toColor(), hsv);

            double distanceCm = distanceSensor != null
                    ? distanceSensor.getDistance(DistanceUnit.CM)
                    : -1;

            SampleColour colour = classify(hsv, distanceCm);
            double target = positionFor(colour);

            if (compressionServo != null) {
                compressionServo.setPosition(target);
            }

            telemetry.addData("Mode", compressionServo == null ? "SIMULATED" : "LIVE");
            telemetry.addData("Detected", colour);
            telemetry.addData("Target servo pos", "%.2f", target);
            telemetry.addLine();
            telemetry.addData("Hue", "%.1f", hsv[0]);
            telemetry.addData("Sat", "%.2f", hsv[1]);
            telemetry.addData("Val", "%.2f", hsv[2]);
            telemetry.addData("Distance (cm)", "%.1f", distanceCm);
            telemetry.addData("R / G / B", "%.3f / %.3f / %.3f",
                    rgba.red, rgba.green, rgba.blue);
            telemetry.update();
        }
    }

    private SampleColour classify(float[] hsv, double distanceCm) {
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Is anything in front of the sensor?
        boolean present;
        if (distanceCm >= 0) {
            present = distanceCm < PRESENT_CM;
        } else {
            present = val > 0.10f && sat > 0.15f;
        }
        if (!present) return SampleColour.NONE;

        if (hue < RED_MAX_HUE || hue > RED_WRAP_HUE)        return SampleColour.RED;
        if (hue >= YELLOW_MIN_HUE && hue <= YELLOW_MAX_HUE) return SampleColour.YELLOW;
        if (hue >= BLUE_MIN_HUE && hue <= BLUE_MAX_HUE)     return SampleColour.BLUE;

        return SampleColour.NONE;
    }

    private double positionFor(SampleColour colour) {
        switch (colour) {
            case RED:
            case BLUE:
                return WIDE_POS;
            case YELLOW:
                return NARROW_POS;
            default:
                return DEFAULT_POS;
        }
    }
}