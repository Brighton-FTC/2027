package org.firstinspires.ftc.teamcode.AprilTag;


/**
 * Compares a measured tag pitch against the calibrated upright pitch.
 *
 * <p>Pitch is expected to drop as the goal fills and tips forward, so
 * tilted = {@code (normalPitch - measuredPitch) > threshold}.
 * Returns false for the {@code UNKNOWN} sentinel — callers must gate on
 * visibility first.
 */
public class TiltEstimateComponent {

    /** Sentinel passthrough: matches {@link AprilTagLocalization#UNKNOWN_PITCH}. */
    private static final double UNKNOWN = 1000.0;

    private double normalPitch;
    private double thresholdDeg;

    public TiltEstimateComponent(double normalPitch){
        this(normalPitch, 3.0);
    }

    public TiltEstimateComponent(double normalPitch, double thresholdDeg){
        this.normalPitch = normalPitch;
        this.thresholdDeg = thresholdDeg;
    }

    public boolean isDown(double degrees){
        return isTilted(degrees);
    }

    public boolean isTilted(double measuredPitch){
        if (measuredPitch == UNKNOWN) {
            return false;
        }
        return (normalPitch - measuredPitch) > thresholdDeg;
    }

    public double getNormalPitch() {
        return normalPitch;
    }

    public void setNormalPitch(double normalPitch) {
        this.normalPitch = normalPitch;
    }
}
