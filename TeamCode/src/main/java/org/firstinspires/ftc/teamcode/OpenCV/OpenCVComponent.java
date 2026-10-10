package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * EasyOpenCV webcam wrapper for sample detection.
 *
 * <p>Fixes vs the copied-in version: correct package, no
 * {@code FtcDashboard} (not a project dependency — Panels is used instead),
 * no drivetrain mixed into vision (centering is reported as data; the OpMode
 * drives), telemetry is constructor-injected and this class never calls
 * {@code telemetry.update()}, streaming resolution comes from
 * {@link RobotConfig.Vision}, the yellow-pipeline copy/paste bug is fixed,
 * and the camera has a real lifecycle ({@link #startStreaming},
 * {@link #stopStreaming}, {@link #close}).
 *
 * <p>Call {@link #close()} from the OpMode's stop path to release the camera.
 */
public class OpenCVComponent {

    /** Which color pipeline is active. */
    public enum Target {
        RED,
        BLUE,
        YELLOW
    }

    private final Telemetry telemetry;
    private final OpenCvWebcam webcam;
    private final Map<Target, SamplePipeline> pipelines = new EnumMap<>(Target.class);

    /** Max per-object lines in telemetry; remainder collapses to "+N more". */
    private static final int MAX_TELEMETRY_OBJECTS = 4;

    private volatile Target target = Target.BLUE;
    private volatile boolean streaming = false;
    private volatile int openErrorCode = 0;

    /** Default webcam from config. */
    public OpenCVComponent(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME, telemetry);
    }

    public OpenCVComponent(HardwareMap hardwareMap, String webcamId, Telemetry telemetry) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(webcamId, "webcamId");
        Objects.requireNonNull(telemetry, "telemetry");
        this.telemetry = telemetry;

        pipelines.put(Target.RED, new RedSamplePipeline());
        pipelines.put(Target.BLUE, new BlueSamplePipeline());
        pipelines.put(Target.YELLOW, new YellowSamplePipeline());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamId), cameraMonitorViewId);
        webcam.setPipeline(pipelines.get(target));
    }

    /** Switches the active color pipeline. Takes effect immediately, streaming or not. */
    public void setTarget(Target target) {
        Objects.requireNonNull(target, "target");
        this.target = target;
        webcam.setPipeline(pipelines.get(target));
    }

    public Target getTarget() {
        return target;
    }

    private SamplePipeline currentPipeline() {
        return pipelines.get(target);
    }

    /** Opens the camera (async) and starts streaming. Safe to call repeatedly. */
    public void startStreaming() {
        if (streaming) {
            return;
        }
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(
                        RobotConfig.Vision.CAMERA_WIDTH,
                        RobotConfig.Vision.CAMERA_HEIGHT,
                        OpenCvCameraRotation.UPRIGHT);
                streaming = true;
            }

            @Override
            public void onError(int errorCode) {
                openErrorCode = errorCode;
                streaming = false;
            }
        });
    }

    /** Stops streaming; no-op unless streaming. */
    public void stopStreaming() {
        if (streaming) {
            webcam.stopStreaming();
            streaming = false;
        }
    }

    /** Stops streaming and releases the camera device. */
    public void close() {
        stopStreaming();
        webcam.closeCameraDevice();
    }

    public boolean isStreaming() {
        return streaming;
    }

    /** Last async-open error code, or 0 when open succeeded / not yet attempted. */
    public int getOpenErrorCode() {
        return openErrorCode;
    }

    /** True when the active pipeline currently holds a contour. */
    public boolean hasTarget() {
        return currentPipeline().hasTarget();
    }

    /**
     * Every detected object this frame, largest area first. Empty when nothing
     * is visible. Each entry carries bearing (deg), distance (in) and
     * robot-relative forward/lateral (in) — see {@link SamplePipeline.Detection}.
     */
    public List<SamplePipeline.Detection> getDetections() {
        return currentPipeline().getDetections();
    }

    /** Number of detected objects in the latest frame. */
    public int getDetectionCount() {
        return currentPipeline().getDetectionCount();
    }

    /**
     * Largest (usually closest) detection, or null when nothing is visible.
     * Index 0 of {@link #getDetections()} — provided for readability.
     */
    public SamplePipeline.Detection getBestDetection() {
        List<SamplePipeline.Detection> dets = getDetections();
        return dets.isEmpty() ? null : dets.get(0);
    }

    /**
     * Raw centroid X pixels, one per detected object, largest first.
     * Empty when nothing is visible. Pixel-space view of the detections for
     * custom logic that wants unprocessed coordinates.
     */
    public List<Double> getObjectPixelXs() {
        List<SamplePipeline.Detection> dets = getDetections();
        List<Double> xs = new ArrayList<>(dets.size());
        for (SamplePipeline.Detection d : dets) {
            xs.add(d.centroidXPx);
        }
        return xs;
    }

    /**
     * Raw centroid Y pixels, one per detected object, largest first.
     * Empty when nothing is visible. Pairs with {@link #getObjectPixelXs()}.
     */
    public List<Double> getObjectPixelYs() {
        List<SamplePipeline.Detection> dets = getDetections();
        List<Double> ys = new ArrayList<>(dets.size());
        for (SamplePipeline.Detection d : dets) {
            ys.add(d.centroidYPx);
        }
        return ys;
    }

    /**
     * Raw bounding-box widths in pixels, one per detected object, largest first.
     * Empty when nothing is visible. Pairs index-for-index with
     * {@link #getObjectPixelXs()} / {@link #getObjectPixelYs()} — same object,
     * same order. This is the raw width the distance math is built on.
     */
    public List<Integer> getObjectPixelWidths() {
        List<SamplePipeline.Detection> dets = getDetections();
        List<Integer> widths = new ArrayList<>(dets.size());
        for (SamplePipeline.Detection d : dets) {
            widths.add(d.widthPx);
        }
        return widths;
    }

    /** Area (px^2) of the active pipeline's contour, or 0 when none. */
    public double getLargestArea() {
        return currentPipeline().getLargestArea();
    }

    // NOTE (2026-10-10): center-error helpers disabled for now — kept for later.
    // Re-enable by uncommenting getCenterErrorX / getCenterErrorY / isCentered
    // and their telemetry call sites below.

    /**
     * Horizontal error (px): contour centroid X minus frame center.
     * Negative = target is left of center. NaN when no target visible.
     * */

    // Use this for X - x_obj = x_robot + Zcos theta + X sin Theta
    public double getCenterErrorX() {
        SamplePipeline pipeline = currentPipeline();
        double cx = pipeline.getCentroidX();
        int width = pipeline.getFrameWidth();
        if (Double.isNaN(cx) || width <= 0) {
            return Double.NaN;
        }
        return cx - width / 2.0;
    }



    /**
     * Vertical error (px): contour centroid Y minus frame center.
     * Positive = target is below center (lower in frame). NaN when no target visible.
     * */
    public double getCenterErrorY() {
        SamplePipeline pipeline = currentPipeline();
        double cy = pipeline.getCentroidY();
        int height = pipeline.getFrameHeight();
        if (Double.isNaN(cy) || height <= 0) {
            return Double.NaN;
        }
        return cy - height / 2.0;
    }

    // Calculate the raw x and y using this - x_obj = x_robot + getDistance()*cos(bearing)
    public double getDistance(){
        return getBestDetection().distanceIn;
    }

    /**
     * True when a target is visible and within
     * {@link RobotConfig.Vision#OPENCV_CENTER_TOLERANCE_PX} of frame center.
     * */
    public boolean isCentered() {
        double err = getCenterErrorX();
        return Math.abs(err) <= RobotConfig.Vision.OPENCV_CENTER_TOLERANCE_PX;
    }


    /** Detection state for Driver Station / Panels (never calls update). */
    public void reportTelemetry() {
        telemetry.addData("OpenCV target", target);
        telemetry.addData("OpenCV streaming", streaming);
        if (openErrorCode != 0) {
            telemetry.addData("OpenCV open error", openErrorCode);
        }
        List<SamplePipeline.Detection> dets = getDetections();
        telemetry.addData("OpenCV objects", dets.size());
        if (dets.isEmpty()) {
            telemetry.addData("OpenCV contour", "none");
            return;
        }
        // Per-object relative position + angle, largest first. Capped so one
        // noisy frame can't flood the driver station.
        int shown = Math.min(dets.size(), MAX_TELEMETRY_OBJECTS);
        for (int i = 0; i < shown; i++) {
            SamplePipeline.Detection d = dets.get(i);
            telemetry.addData("OpenCV obj" + i,
                    "brg %+.1f elev %+.1f | dist %s | fwd %s lat %s h %s",
                    d.bearingDeg, d.elevationDeg, fmtIn(d.distanceIn),
                    fmtIn(d.forwardIn), fmtIn(d.lateralIn), fmtIn(d.heightIn));
        }
        if (dets.size() > shown) {
            telemetry.addData("OpenCV", "+%d more", dets.size() - shown);
        }
        // Center-error lines disabled with the helpers above.

        double errX = getCenterErrorX();
        double errY = getCenterErrorY();
        telemetry.addData("OpenCV X centered",
                isCentered() + (Double.isNaN(errX) ? "" : String.format(" (err %+.0f px)", errX)));
        telemetry.addData("OpenCV Y centered",
                isCentered() + (Double.isNaN(errY) ? "" : String.format(" (err %+.0f px)", errY)));

    }

    private static String fmtIn(double inches) {
        return Double.isNaN(inches) ? "n/a" : String.format("%.1f in", inches);
    }
}
