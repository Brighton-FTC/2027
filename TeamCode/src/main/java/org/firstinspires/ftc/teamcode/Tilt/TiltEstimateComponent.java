package org.firstinspires.ftc.teamcode.Tilt;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Decides whether the goal (HIVE CELL) is tipped, from its measured height.
 *
 * <p>AprilTag-free tilt sensing: {@link
 * org.firstinspires.ftc.teamcode.OpenCV.GoalTiltPipeline} isolates the goal
 * marker by colour and reports its height above the ground; the CELL sits at
 * a known height when the HIVE is upright ({@code
 * RobotConfig.Vision#NORMAL_GOAL_HEIGHT_IN}), and a HIVE tip lowers the
 * marker — so tilted = {@code (normalHeight - measuredHeight) > threshold}.
 * The old AprilTag {@code ftcPose.pitch} source is gone; the drop-over-
 * threshold shape is unchanged.
 *
 * <p>Returns false for the {@link #UNKNOWN_HEIGHT} sentinel (and any NaN) —
 * callers may treat "unknown" as "not tilted" without a visibility gate.
 */
public class TiltEstimateComponent {

    /** Sentinel meaning "no goal marker with a solvable height is visible". */
    public static final double UNKNOWN_HEIGHT = RobotConfig.Launcher.UNKNOWN_POSE;

    private double normalHeightIn;
    private double thresholdIn;

    public TiltEstimateComponent(double normalHeightIn) {
        this(normalHeightIn, RobotConfig.Vision.TILT_HEIGHT_THRESHOLD_IN);
    }

    public TiltEstimateComponent(double normalHeightIn, double thresholdIn) {
        this.normalHeightIn = normalHeightIn;
        this.thresholdIn = thresholdIn;
    }

    public boolean isTilted(double measuredHeightIn) {
        if (measuredHeightIn == UNKNOWN_HEIGHT || Double.isNaN(measuredHeightIn)) {
            return false;
        }
        return (normalHeightIn - measuredHeightIn) > thresholdIn;
    }

    public double getNormalHeightIn() {
        return normalHeightIn;
    }

    public void setNormalHeightIn(double normalHeightIn) {
        this.normalHeightIn = normalHeightIn;
    }
}
