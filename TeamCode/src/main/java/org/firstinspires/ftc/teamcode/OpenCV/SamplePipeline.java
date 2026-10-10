package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Base HSV threshold + largest-contour detector.
 *
 * <p>Fixes vs the copied-in version: correct package, no per-frame {@code Mat}
 * allocation (reused buffers, so no native-memory leak), contour snapshots are
 * copied under lock so OpMode-thread reads are safe, blur kernel is odd-sized,
 * and multiple HSV ranges are supported (needed for red hue wrap-around).
 *
 * <p>Resolution comes from {@link RobotConfig.Vision#CAMERA_WIDTH} /
 * {@link RobotConfig.Vision#CAMERA_HEIGHT} via the component; area threshold
 * from {@link RobotConfig.Vision#OPENCV_MIN_AREA_PX} (live-tunable).
 */
public abstract class SamplePipeline extends OpenCvPipeline {

    private final Scalar[] lowers;
    private final Scalar[] uppers;

    // Reused on the camera thread only. OpenCV auto-sizes Mat outputs, so
    // these allocate once and never leak.
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat rangeMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();

    // Published snapshot for the OpMode thread.
    private final Object lock = new Object();
    private MatOfPoint largestContourCopy;
    private volatile double largestAreaPx = 0.0;
    private volatile double centroidXPx = Double.NaN;
    private volatile int frameWidthPx = 0;

    /** Single HSV range. */
    protected SamplePipeline(Scalar lower, Scalar upper) {
        this(new Scalar[]{lower}, new Scalar[]{upper});
    }

    /** Multiple HSV ranges OR-ed together (use for red's hue wrap-around). */
    protected SamplePipeline(Scalar[] lowers, Scalar[] uppers) {
        if (lowers == null || uppers == null || lowers.length == 0
                || lowers.length != uppers.length) {
            throw new IllegalArgumentException("lowers/uppers must be non-empty and equal length");
        }
        this.lowers = lowers.clone();
        this.uppers = uppers.clone();
    }

    /** BGR color used to outline the detected contour. */
    protected abstract Scalar contourColor();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        int blur = Math.max(1, RobotConfig.Vision.OPENCV_BLUR_SIZE);
        if (blur % 2 == 0) {
            blur += 1; // GaussianBlur requires an odd kernel
        }
        Imgproc.GaussianBlur(hsv, hsv, new Size(blur, blur), 0);

        mask.setTo(new Scalar(0));
        for (int i = 0; i < lowers.length; i++) {
            Core.inRange(hsv, lowers[i], uppers[i], rangeMask);
            Core.bitwise_or(mask, rangeMask, mask);
        }

        // Release previous native contour storage before dropping references.
        for (MatOfPoint c : contours) {
            c.release();
        }
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double minArea = RobotConfig.Vision.OPENCV_MIN_AREA_PX;
        MatOfPoint best = null;
        double bestArea = 0.0;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area >= minArea && area > bestArea) {
                bestArea = area;
                best = c;
            }
        }

        double cx = Double.NaN;
        if (best != null) {
            Moments m = Imgproc.moments(best);
            if (m.get_m00() != 0) {
                cx = m.get_m10() / m.get_m00();
            } else {
                best = null;
                bestArea = 0.0;
            }
        }

        synchronized (lock) {
            if (largestContourCopy != null) {
                largestContourCopy.release();
                largestContourCopy = null;
            }
            if (best != null) {
                largestContourCopy = new MatOfPoint(best.toArray());
            }
        }
        largestAreaPx = bestArea;
        centroidXPx = cx;
        frameWidthPx = input.cols();

        annotate(input);
        return input;
    }

    /** Default overlay: contour outline + centroid dot + frame-center marker. */
    protected void annotate(Mat output) {
        MatOfPoint contour = copyLargestContour();
        if (contour == null) {
            drawCenterMarker(output);
            return;
        }
        try {
            List<MatOfPoint> list = new ArrayList<>(1);
            list.add(contour);
            Imgproc.drawContours(output, list, -1, contourColor(), 2);
            Moments m = Imgproc.moments(contour);
            if (m.get_m00() != 0) {
                Imgproc.circle(output,
                        new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00()),
                        6, contourColor(), -1);
            }
        } finally {
            contour.release();
        }
        drawCenterMarker(output);
    }

    private static void drawCenterMarker(Mat output) {
        Imgproc.circle(output,
                new Point(output.cols() / 2.0, output.rows() / 2.0),
                15, new Scalar(0, 255, 0), 3);
    }

    /** True when a contour at/above the area threshold is currently held. */
    public boolean hasTarget() {
        return !Double.isNaN(centroidXPx);
    }

    /** Area (px^2) of the published contour, or 0 when none. */
    public double getLargestArea() {
        return largestAreaPx;
    }

    /** Centroid X (px) of the published contour, or NaN when none. */
    public double getCentroidX() {
        return centroidXPx;
    }

    /** Width (px) of the last processed frame, or 0 before the first frame. */
    public int getFrameWidth() {
        return frameWidthPx;
    }

    /**
     * Returns a copy of the published contour (caller must {@code release()}
     * it), or null when none. The pipeline keeps ownership of its snapshot.
     */
    public MatOfPoint copyLargestContour() {
        synchronized (lock) {
            return largestContourCopy != null
                    ? new MatOfPoint(largestContourCopy.toArray()) : null;
        }
    }
}
