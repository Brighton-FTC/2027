package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Scalar;

public class BlueSamplePipeline extends SamplePipeline {
    public static final Scalar LOWER = new Scalar(100, 100, 50);
    public static final Scalar UPPER = new Scalar(130, 255, 255);

    public BlueSamplePipeline() {
        super(LOWER, UPPER);
    }

    @Override
    protected Scalar contourColor() {
        return new Scalar(255, 0, 0); // blue (BGR)
    }
}
