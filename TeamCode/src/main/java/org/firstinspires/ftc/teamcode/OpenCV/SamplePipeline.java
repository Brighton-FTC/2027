package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Base HSV threshold + contour detector.
 *
 * <p>Only the closest contour (largest area above threshold — same-size
 * game elements, bigger blob = nearer) gets full centroid and geometry math
 * and is published as a single-element {@link Detection} list.
 * <ul>
 *   <li><b>bearing</b> from horizontal pixel offset via the pinhole model
 *       ({@link RobotConfig.Vision#OPENCV_HFOV_DEG}). Negative = left of
 *       center, positive = right.</li>
 *   <li><b>distance</b> from apparent width vs the known object width
 *       ({@link RobotConfig.Vision#OPENCV_KNOWN_OBJECT_WIDTH_IN}).
 *       NaN when that width is not configured or the box has no width.</li>
 *   <li><b>robot-relative position</b> (forward/lateral inches, camera assumed
 *       on the robot centerline facing forward):
 *       forward = dist * cos(bearing), lateral = dist * sin(bearing).</li>
 * </ul>
 *
 * <p>Buffers are reused (no per-frame allocation, no native leak) and the
 * published snapshot is an immutable copy, so OpMode-thread reads are safe.
 */
public abstract class SamplePipeline extends OpenCvPipeline {

    /** One detected object, in robot-relative terms. All fields are final. */
    public static final class Detection {
        /** Centroid X/Y (px). */
        public final double centroidXPx;
        public final double centroidYPx;
        /** Contour area (px^2). */
        public final double areaPx;
        /** Bounding-box width/height (px). */
        public final int widthPx;
        public final int heightPx;
        /** Bearing (deg): 0 = straight ahead, + = right, - = left. */
        public final double bearingDeg;
        /**
         * Elevation (deg) above the camera optical axis: + = above axis.
         * Needs no size calibration.
         */
        public final double elevationDeg;
        /** Straight-line distance (in), or NaN when not computable. */
        public final double distanceIn;
        /** Forward component (in) relative to the robot, or NaN. */
        public final double forwardIn;
        /** Lateral component (in), + = right of robot, or NaN. */
        public final double lateralIn;
        /**
         * Object-center height above the ground (in), or NaN.
         * Needs distance plus the configured camera mount height/pitch.
         */
        public final double heightIn;

        Detection(double centroidXPx, double centroidYPx, double areaPx,
                  int widthPx, int heightPx, double bearingDeg, double elevationDeg,
                  double distanceIn, double forwardIn, double lateralIn, double heightIn) {
            this.centroidXPx = centroidXPx;
            this.centroidYPx = centroidYPx;
            this.areaPx = areaPx;
            this.widthPx = widthPx;
            this.heightPx = heightPx;
            this.bearingDeg = bearingDeg;
            this.elevationDeg = elevationDeg;
            this.distanceIn = distanceIn;
            this.forwardIn = forwardIn;
            this.lateralIn = lateralIn;
            this.heightIn = heightIn;
        }
    }

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
    private volatile List<Detection> detections = Collections.emptyList();
    private volatile double largestAreaPx = 0.0;
    private volatile double centroidXPx = Double.NaN;
    private volatile double centroidYPx = Double.NaN;
    private volatile int frameWidthPx = 0;
    private volatile int frameHeightPx = 0;

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

    /** BGR color used to outline detected contours. */
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

        int width = input.cols();
        int height = input.rows();
        double focalPx = focalLengthPx(width);
        double focalYPx = verticalFocalLengthPx(width, height);
        double knownWidthIn = RobotConfig.Vision.OPENCV_KNOWN_OBJECT_WIDTH_IN;
        double camHeightIn = RobotConfig.Vision.OPENCV_CAMERA_HEIGHT_IN;
        double camPitchDeg = RobotConfig.Vision.OPENCV_CAMERA_PITCH_DEG;
        double minArea = RobotConfig.Vision.OPENCV_MIN_AREA_PX;

        List<Detection> found = new ArrayList<>(1);
        // Pass 1 (cheap): largest area above threshold wins. Area is the
        // closest-object proxy — same-size game elements, bigger blob = nearer.
        MatOfPoint best = null;
        double bestArea = 0.0;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area >= minArea && area > bestArea) {
                bestArea = area;
                best = c;
            }
        }

        // Pass 2 (full math): centroid, box, bearing, distance — best only.
        double cx = Double.NaN;
        double cy = Double.NaN;
        if (best != null) {
            Moments m = Imgproc.moments(best);
            if (m.get_m00() == 0) {
                best = null;
                bestArea = 0.0;
            } else {
                cx = m.get_m10() / m.get_m00();
                cy = m.get_m01() / m.get_m00();
                Rect box = Imgproc.boundingRect(best);

                double bearingDeg = Math.toDegrees(Math.atan((cx - width / 2.0) / focalPx));
                double elevationDeg =
                        Math.toDegrees(Math.atan((height / 2.0 - cy) / focalYPx));
                double distanceIn = Double.NaN;
                if (knownWidthIn > 0 && box.width > 0) {
                    distanceIn = knownWidthIn * focalPx / box.width;
                }
                double bearingRad = Math.toRadians(bearingDeg);
                double forwardIn = Double.isNaN(distanceIn)
                        ? Double.NaN : distanceIn * Math.cos(bearingRad);
                double lateralIn = Double.isNaN(distanceIn)
                        ? Double.NaN : distanceIn * Math.sin(bearingRad);
                // Ray leaves the camera hCam above ground, pitched camPitchDeg down;
                // elevationDeg is measured up from that axis.
                double heightIn = Double.isNaN(distanceIn) ? Double.NaN
                        : camHeightIn + distanceIn
                        * Math.sin(Math.toRadians(elevationDeg - camPitchDeg));

                found.add(new Detection(cx, cy, bestArea, box.width, box.height,
                        bearingDeg, elevationDeg, distanceIn, forwardIn, lateralIn, heightIn));
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
        detections = Collections.unmodifiableList(found);
        largestAreaPx = bestArea;
        centroidXPx = cx;
        centroidYPx = cy;
        frameWidthPx = width;
        frameHeightPx = height;

        annotate(input);
        return input;
    }

    /**
     * Pinhole focal length (px) from frame width and configured HFOV.
     * Falls back to width/2 when HFOV is not configured.
     */
    public static double focalLengthPx(int frameWidthPx) {
        double hfov = RobotConfig.Vision.OPENCV_HFOV_DEG;
        if (hfov <= 0 || hfov >= 180) {
            return Math.max(1.0, frameWidthPx / 2.0);
        }
        return (frameWidthPx / 2.0) / Math.tan(Math.toRadians(hfov / 2.0));
    }

    /**
     * Vertical focal length (px), derived from HFOV and the frame aspect
     * ratio — no extra calibration needed.
     */
    public static double verticalFocalLengthPx(int frameWidthPx, int frameHeightPx) {
        double hfov = RobotConfig.Vision.OPENCV_HFOV_DEG;
        if (hfov <= 0 || hfov >= 180 || frameWidthPx <= 0 || frameHeightPx <= 0) {
            return Math.max(1.0, frameHeightPx / 2.0);
        }
        double vfovRad = 2.0 * Math.atan(
                Math.tan(Math.toRadians(hfov / 2.0)) * frameHeightPx / frameWidthPx);
        return (frameHeightPx / 2.0) / Math.tan(vfovRad / 2.0);
    }

    /** Overlay: closest detection outlined + centroid, plus frame-center marker. */
    protected void annotate(Mat output) {
        Scalar color = contourColor();
        MatOfPoint contour = copyLargestContour();
        if (contour == null) {
            drawCenterMarker(output);
            return;
        }
        try {
            List<MatOfPoint> list = new ArrayList<>(1);
            list.add(contour);
            Imgproc.drawContours(output, list, -1, color, 2);
            Moments m = Imgproc.moments(contour);
            if (m.get_m00() != 0) {
                Point center = new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00());
                Imgproc.circle(output, center, 6, color, -1);
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

    /**
     * Immutable snapshot holding the closest detected object this frame, or
     * empty when nothing is visible. Only ever 0–1 entries: full centroid and
     * geometry math runs on the closest contour alone.
     */
    public List<Detection> getDetections() {
        return detections;
    }

    /** Number of detected objects in the latest frame. */
    public int getDetectionCount() {
        return detections.size();
    }

    /** True when at least one contour at/above the area threshold is held. */
    public boolean hasTarget() {
        return !detections.isEmpty();
    }

    /** Area (px^2) of the largest contour, or 0 when none. */
    public double getLargestArea() {
        return largestAreaPx;
    }

    /** Centroid X (px) of the largest contour, or NaN when none. */
    public double getCentroidX() {
        return centroidXPx;
    }

    /** Centroid Y (px) of the largest contour, or NaN when none. */
    public double getCentroidY() {
        return centroidYPx;
    }

    /** Width (px) of the last processed frame, or 0 before the first frame. */
    public int getFrameWidth() {
        return frameWidthPx;
    }

    /** Height (px) of the last processed frame, or 0 before the first frame. */
    public int getFrameHeight() {
        return frameHeightPx;
    }



    /**
     * Returns a copy of the largest contour (caller must {@code release()}
     * it), or null when none. The pipeline keeps ownership of its snapshot.
     */
    public MatOfPoint copyLargestContour() {
        synchronized (lock) {
            return largestContourCopy != null
                    ? new MatOfPoint(largestContourCopy.toArray()) : null;
        }
    }
}
