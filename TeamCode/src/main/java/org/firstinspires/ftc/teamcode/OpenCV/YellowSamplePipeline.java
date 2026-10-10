package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Scalar;

public class YellowSamplePipeline extends SamplePipeline {
    public static final Scalar LOWER = new Scalar(20, 100, 100);
    public static final Scalar UPPER = new Scalar(30, 255, 255);

    public YellowSamplePipeline() {
        super(LOWER, UPPER);
    }

    @Override
    protected Scalar contourColor() {
        return new Scalar(0, 255, 255); // yellow (BGR)
    }
}
