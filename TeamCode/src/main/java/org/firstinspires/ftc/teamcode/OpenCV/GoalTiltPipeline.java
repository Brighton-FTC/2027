package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Scalar;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * HSV detector for the goal (HIVE CELL) marker — the AprilTag-free tilt
 * signal.
 *
 * <p>The tilt estimator used to read {@code ftcPose.pitch} off the CELL's
 * AprilTag cluster; this pipeline replaces that with pure OpenCV geometry:
 * it isolates the marker by colour, and the shared {@link SamplePipeline}
 * math turns the blob into a distance (from the marker's known width) and a
 * <b>height above the ground</b> ({@link SamplePipeline.Detection#heightIn}).
 * When the HIVE tips, the marker physically drops, so
 * {@code NORMAL_GOAL_HEIGHT_IN - measured height} is the tilt signal —
 * no AprilTags, no second camera stack.
 *
 * <p>Everything tunable lives in {@link RobotConfig.Vision} ({@code GOAL_*}
 * HSV bounds, marker width, upright height, threshold) and is re-read every
 * frame, so it can be calibrated live from Panels.
 */
public class GoalTiltPipeline extends SamplePipeline {

    public GoalTiltPipeline() {
        // Placeholder ranges; refreshThresholds() fills the real ones from
        // config before the first frame is processed.
        super(new Scalar(0, 0, 0), new Scalar(0, 0, 0));
        refreshThresholds();
    }

    @Override
    protected void refreshThresholds() {
        lowers = new Scalar[]{new Scalar(
                RobotConfig.Vision.GOAL_HUE_MIN,
                RobotConfig.Vision.GOAL_SAT_MIN,
                RobotConfig.Vision.GOAL_VAL_MIN)};
        uppers = new Scalar[]{new Scalar(
                RobotConfig.Vision.GOAL_HUE_MAX,
                RobotConfig.Vision.GOAL_SAT_MAX,
                RobotConfig.Vision.GOAL_VAL_MAX)};
    }

    @Override
    protected double knownWidthIn() {
        return RobotConfig.Vision.GOAL_KNOWN_WIDTH_IN;
    }

    @Override
    protected Scalar contourColor() {
        return new Scalar(0, 215, 255); // warm amber (BGR) for the overlay
    }
}
