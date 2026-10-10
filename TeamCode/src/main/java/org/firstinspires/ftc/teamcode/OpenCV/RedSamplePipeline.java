package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Scalar;

/**
 * Red needs two HSV ranges: hue wraps at 0/180 in OpenCV, so the old
 * single 170-180 band missed reds in the 0-10 band.
 */
public class RedSamplePipeline extends SamplePipeline {
    public static final Scalar LOWER_LOW = new Scalar(0, 120, 70);
    public static final Scalar UPPER_LOW = new Scalar(10, 255, 255);
    public static final Scalar LOWER_HIGH = new Scalar(170, 120, 70);
    public static final Scalar UPPER_HIGH = new Scalar(180, 255, 255);

    public RedSamplePipeline() {
        super(new Scalar[]{LOWER_LOW, LOWER_HIGH}, new Scalar[]{UPPER_LOW, UPPER_HIGH});
    }

    @Override
    protected Scalar contourColor() {
        return new Scalar(0, 0, 255); // red (BGR)
    }
}
