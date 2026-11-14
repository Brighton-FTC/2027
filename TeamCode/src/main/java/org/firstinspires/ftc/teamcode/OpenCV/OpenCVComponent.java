package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Tilt.TiltEstimateComponent;
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
import java.util.concurrent.TimeUnit;

/**
 * EasyOpenCV webcam wrapper for sample detection.
 *
 * <p>Everything needed to stream is configured here and in
 * {@link RobotConfig.Vision} — resolution, rotation, stream format, live
 * view, exposure/gain/white balance. No camera model sheet is required: any
 * UVC webcam works, and {@link RobotConfig.Vision#AUTO_DETECT_CAMERA} makes
 * the component fall back to the first attached webcam when the configured
 * name is missing, so a naming mismatch cannot crash OpMode init.
 *
 * <p>What still needs real-world numbers (none of it blocks streaming):
 * <ul>
 *   <li>{@link RobotConfig.Vision#OPENCV_HFOV_DEG} or, better, a measured
 *       {@link RobotConfig.Vision#OPENCV_FOCAL_PX} for exact bearing;</li>
 *   <li>{@link RobotConfig.Vision#OPENCV_KNOWN_OBJECT_WIDTH_IN} for
 *       distance;</li>
 *   <li>{@link RobotConfig.Vision#OPENCV_CAMERA_HEIGHT_IN} and
 *       {@link RobotConfig.Vision#OPENCV_CAMERA_PITCH_DEG} for height.</li>
 * </ul>
 *
 * <p>Telemetry is constructor-injected and this class never calls
 * {@code telemetry.update()}. Call {@link #close()} from the OpMode's stop
 * path to release the camera.
 */
public class OpenCVComponent {

    /** Which color pipeline is active. */
    public enum Target {
        RED,
        BLUE,
        YELLOW,
        /** Goal (HIVE CELL) marker — feeds the tilt estimator, not pathing. */
        GOAL
    }

    /** Camera lifecycle, for telemetry and safe repeated calls. */
    public enum CameraState {
        /** No webcam in the RC config / nothing attached. */
        NO_CAMERA,
        /** Configured but not opened yet. */
        IDLE,
        /** Async open in flight. */
        OPENING,
        /** Streaming frames. */
        STREAMING,
        /** Open or start failed — see {@link #getOpenErrorCode()}. */
        ERROR
    }

    /** Max per-object lines in telemetry; remainder collapses to "+N more". */
    private static final int MAX_TELEMETRY_OBJECTS = 4;

    /** Stop retrying camera controls after this many loop ticks. */
    private static final int MAX_CONTROL_ATTEMPTS = 30;

    private final Telemetry telemetry;
    private final OpenCvWebcam webcam; // null when no camera could be resolved
    private final String resolvedCameraName;
    private final String cameraNote;
    private final Map<Target, SamplePipeline> pipelines = new EnumMap<>(Target.class);

    private volatile Target target;
    private volatile CameraState state = CameraState.IDLE;
    private volatile int openErrorCode = 0;
    private volatile boolean controlsApplied = false;
    private volatile int controlAttempts = 0;
    private volatile String controlStatus = "not applied";

    public OpenCVComponent(HardwareMap hardwareMap, String webcamId, Telemetry telemetry, Target target) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(webcamId, "webcamId");
        Objects.requireNonNull(telemetry, "telemetry");
        this.telemetry = telemetry;
        this.target = target;

        pipelines.put(Target.RED, new RedSamplePipeline());
        pipelines.put(Target.BLUE, new BlueSamplePipeline());
        pipelines.put(Target.YELLOW, new YellowSamplePipeline());
        pipelines.put(Target.GOAL, new GoalTiltPipeline());

        String[] resolved = new String[]{webcamId};
        WebcamName cameraName = findCamera(hardwareMap, webcamId, resolved);
        this.resolvedCameraName = resolved[0] == null ? webcamId : resolved[0];
        if (cameraName == null) {
            this.cameraNote = "no webcam in the RC config — add a Webcam and name it '"
                    + webcamId + "'";
            this.webcam = null;
            this.state = CameraState.NO_CAMERA;
        } else {
            this.cameraNote = resolvedCameraName.equals(webcamId)
                    ? "" : "using '" + resolvedCameraName + "' (config says '" + webcamId + "')";
            this.webcam = createWebcam(hardwareMap, cameraName);
            if (webcam != null) {
                webcam.setPipeline(pipelines.get(target));
                if (RobotConfig.Vision.SHOW_FPS_OVERLAY) {
                    webcam.showFpsMeterOnViewport(true);
                }
            } else {
                this.state = CameraState.NO_CAMERA;
            }
        }
    }

    /**
     * Resolve the webcam by the configured name, falling back to the first
     * attached one when {@link RobotConfig.Vision#AUTO_DETECT_CAMERA} is on.
     * Writes the name that was actually used into {@code resolvedOut[0]}.
     */
    private static WebcamName findCamera(HardwareMap hardwareMap, String preferred,
                                         String[] resolvedOut) {
        WebcamName preferredCam = hardwareMap.tryGet(WebcamName.class, preferred);
        if (!RobotConfig.Vision.AUTO_DETECT_CAMERA) {
            resolvedOut[0] = preferred;
            return preferredCam;
        }
        if (preferredCam != null && preferredCam.isAttached()) {
            resolvedOut[0] = preferred;
            return preferredCam;
        }
        // Configured name missing or unplugged: use the first attached camera.
        WebcamName firstConfigured = preferredCam;
        String firstName = preferredCam != null ? preferred : null;
        for (String name : hardwareMap.getAllNames(WebcamName.class)) {
            WebcamName candidate = hardwareMap.tryGet(WebcamName.class, name);
            if (candidate == null) {
                continue;
            }
            if (candidate.isAttached()) {
                resolvedOut[0] = name;
                return candidate;
            }
            if (firstConfigured == null) {
                firstConfigured = candidate;
                firstName = name;
            }
        }
        resolvedOut[0] = firstName != null ? firstName : preferred;
        return firstConfigured;
    }

    private static OpenCvWebcam createWebcam(HardwareMap hardwareMap, WebcamName cameraName) {
        try {
            if (!RobotConfig.Vision.ENABLE_LIVE_VIEW) {
                return OpenCvCameraFactory.getInstance().createWebcam(cameraName);
            }
            int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id",
                            hardwareMap.appContext.getPackageName());
            return OpenCvCameraFactory.getInstance()
                    .createWebcam(cameraName, cameraMonitorViewId);
        } catch (RuntimeException e) {
            return null;
        }
    }

    /** Switches the active color pipeline. Takes effect immediately, streaming or not. */
    public void setTarget(Target target) {
        Objects.requireNonNull(target, "target");
        this.target = target;
        if (webcam != null) {
            webcam.setPipeline(pipelines.get(target));
        }
    }

    public Target getTarget() {
        return target;
    }

    private SamplePipeline currentPipeline() {
        return pipelines.get(target);
    }

    /** Opens the camera (async) and starts streaming. Safe to call repeatedly. */
    public void startStreaming() {
        if (webcam == null || state == CameraState.OPENING || state == CameraState.STREAMING) {
            return;
        }
        state = CameraState.OPENING;
        openErrorCode = 0;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                try {
                    webcam.startStreaming(
                            RobotConfig.Vision.CAMERA_WIDTH,
                            RobotConfig.Vision.CAMERA_HEIGHT,
                            rotation(),
                            streamFormat());
                    state = CameraState.STREAMING;
                    applyCameraControls();
                } catch (RuntimeException e) {
                    openErrorCode = -1;
                    controlStatus = "stream start failed: " + e.getMessage();
                    state = CameraState.ERROR;
                }
            }

            @Override
            public void onError(int errorCode) {
                openErrorCode = errorCode;
                state = CameraState.ERROR;
            }
        });
    }

    /** Stops streaming; no-op unless streaming. */
    public void stopStreaming() {
        if (webcam != null && state == CameraState.STREAMING) {
            webcam.stopStreaming();
            state = CameraState.IDLE;
        }
    }

    /** Stops streaming and releases the camera device. */
    public void close() {
        stopStreaming();
        if (webcam != null) {
            try {
                webcam.closeCameraDevice();
            } catch (RuntimeException ignored) {
                // Device already gone — nothing to release.
            }
        }
        if (state != CameraState.NO_CAMERA) {
            state = CameraState.IDLE;
        }
    }

    public boolean isStreaming() {
        return state == CameraState.STREAMING;
    }

    /** True when a webcam was found in the RC configuration. */
    public boolean isCameraAvailable() {
        return webcam != null;
    }

    public CameraState getCameraState() {
        return state;
    }

    /** Last async-open error code, or 0 when open succeeded / not yet attempted. */
    public int getOpenErrorCode() {
        return openErrorCode;
    }

    /** Name actually used, which can differ from the configured name. */
    public String getResolvedCameraName() {
        return resolvedCameraName;
    }

    /** One-line camera status for driver-station telemetry. */
    public String getCameraStatus() {
        if (webcam == null) {
            return "NO CAMERA — " + cameraNote;
        }
        return resolvedCameraName + " [" + state + "]"
                + (cameraNote.isEmpty() ? "" : " " + cameraNote);
    }

    /** Live stream FPS (0 before the first frame). */
    public float getFps() {
        return webcam == null ? 0f : webcam.getFps();
    }

    // ---- camera controls ----

    /**
     * Apply exposure / gain / white-balance settings once the stream is live.
     * Called automatically after the camera opens; {@link #reportTelemetry()}
     * retries for a few ticks because some hubs expose controls slightly
     * after {@code startStreaming()} returns.
     */
    private synchronized void applyCameraControls() {
        if (webcam == null || controlsApplied) {
            return;
        }
        controlAttempts++;
        StringBuilder status = new StringBuilder();
        status.append("exp ").append(safeApply(this::applyExposure));
        status.append(" | gain ").append(safeApply(this::applyGain));
        status.append(" | wb ").append(safeApply(this::applyWhiteBalance));
        controlStatus = status.toString();
        if (!status.toString().contains("error") || controlAttempts >= MAX_CONTROL_ATTEMPTS) {
            controlsApplied = true;
        }
    }

    private String safeApply(java.util.function.Supplier<String> action) {
        try {
            return action.get();
        } catch (RuntimeException e) {
            String message = e.getMessage();
            return "error" + (message == null ? "" : ": " + message);
        }
    }

    private String applyExposure() {
        ExposureControl control = webcam.getExposureControl();
        if (control == null || !control.isExposureSupported()) {
            return "unsupported";
        }
        if (RobotConfig.Vision.CAMERA_EXPOSURE_MODE
                == RobotConfig.Vision.ExposureMode.MANUAL) {
            ExposureControl.Mode mode = ExposureControl.Mode.Manual;
            if (!control.isModeSupported(mode)) {
                return "manual unsupported";
            }
            if (!control.setMode(mode)) {
                return "manual rejected";
            }
            long targetUs = TimeUnit.MILLISECONDS.toMicros(
                    Math.max(1, RobotConfig.Vision.CAMERA_EXPOSURE_MS));
            long minUs = control.getMinExposure(TimeUnit.MICROSECONDS);
            long maxUs = control.getMaxExposure(TimeUnit.MICROSECONDS);
            if (minUs > 0) {
                targetUs = Math.max(targetUs, minUs);
            }
            if (maxUs > 0) {
                targetUs = Math.min(targetUs, maxUs);
            }
            return control.setExposure(targetUs, TimeUnit.MICROSECONDS)
                    ? (targetUs / 1000) + " ms" : "value rejected";
        }
        if (control.isModeSupported(ExposureControl.Mode.ContinuousAuto)) {
            control.setMode(ExposureControl.Mode.ContinuousAuto);
        }
        return "auto";
    }

    private String applyGain() {
        GainControl control = webcam.getGainControl();
        if (control == null) {
            return "unsupported";
        }
        if (RobotConfig.Vision.CAMERA_EXPOSURE_MODE
                != RobotConfig.Vision.ExposureMode.MANUAL
                || RobotConfig.Vision.CAMERA_GAIN <= 0) {
            return "auto";
        }
        int value = clamp(RobotConfig.Vision.CAMERA_GAIN,
                control.getMinGain(), control.getMaxGain());
        return control.setGain(value) ? Integer.toString(value) : "value rejected";
    }

    private String applyWhiteBalance() {
        WhiteBalanceControl control = webcam.getWhiteBalanceControl();
        if (control == null) {
            return "unsupported";
        }
        if (!RobotConfig.Vision.CAMERA_LOCK_WHITE_BALANCE) {
            control.setMode(WhiteBalanceControl.Mode.AUTO);
            return "auto";
        }
        if (!control.setMode(WhiteBalanceControl.Mode.MANUAL)) {
            return "manual rejected";
        }
        int kelvin = clamp(RobotConfig.Vision.CAMERA_WHITE_BALANCE_K,
                control.getMinWhiteBalanceTemperature(),
                control.getMaxWhiteBalanceTemperature());
        return control.setWhiteBalanceTemperature(kelvin)
                ? kelvin + " K" : "value rejected";
    }

    private static int clamp(int value, int min, int max) {
        if (max > min) {
            return Math.max(min, Math.min(max, value));
        }
        return value;
    }

    private static OpenCvCameraRotation rotation() {
        switch (RobotConfig.Vision.CAMERA_ROTATION) {
            case SIDEWAYS_LEFT:
                return OpenCvCameraRotation.SIDEWAYS_LEFT;
            case SIDEWAYS_RIGHT:
                return OpenCvCameraRotation.SIDEWAYS_RIGHT;
            case UPSIDE_DOWN:
                return OpenCvCameraRotation.UPSIDE_DOWN;
            case UPRIGHT:
            default:
                return OpenCvCameraRotation.UPRIGHT;
        }
    }

    private static OpenCvWebcam.StreamFormat streamFormat() {
        return RobotConfig.Vision.CAMERA_STREAM_FORMAT
                == RobotConfig.Vision.StreamFormat.MJPEG
                ? OpenCvWebcam.StreamFormat.MJPEG
                : OpenCvWebcam.StreamFormat.YUY2;
    }

    // ---- detection accessors ----

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
     * Height above the ground (in) of the goal marker seen by the GOAL
     * pipeline — the tilt estimator's input ({@link
     * org.firstinspires.ftc.teamcode.Tilt.TiltEstimateComponent}). Returns
     * {@link TiltEstimateComponent#UNKNOWN_HEIGHT} when the GOAL pipeline is
     * not active, nothing is visible, or the marker's distance is not
     * computable (no known width / degenerate box).
     */
    public double getGoalHeightIn() {
        if (target != Target.GOAL) {
            return TiltEstimateComponent.UNKNOWN_HEIGHT;
        }
        SamplePipeline.Detection d = getBestDetection();
        if (d == null || Double.isNaN(d.heightIn)) {
            return TiltEstimateComponent.UNKNOWN_HEIGHT;
        }
        return d.heightIn;
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

    /** Width (px) of the last processed frame, or 0 before the first frame. */
    public int getFrameWidth() {
        return currentPipeline().getFrameWidth();
    }

    /** Height (px) of the last processed frame, or 0 before the first frame. */
    public int getFrameHeight() {
        return currentPipeline().getFrameHeight();
    }

    /**
     * Horizontal error (px): contour centroid X minus frame center.
     * Negative = target is left of center. NaN when no target visible.
     */
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
     */
    public double getCenterErrorY() {
        SamplePipeline pipeline = currentPipeline();
        double cy = pipeline.getCentroidY();
        int height = pipeline.getFrameHeight();
        if (Double.isNaN(cy) || height <= 0) {
            return Double.NaN;
        }
        return cy - height / 2.0;
    }

    /** Distance (in) to the best detection, or NaN when nothing is visible. */
    public double getDistance() {
        SamplePipeline.Detection best = getBestDetection();
        return best == null ? Double.NaN : best.distanceIn;
    }

    /**
     * True when a target is visible and within
     * {@link RobotConfig.Vision#OPENCV_CENTER_TOLERANCE_PX} of frame center.
     */
    public boolean isCentered() {
        double err = getCenterErrorX();
        return !Double.isNaN(err)
                && Math.abs(err) <= RobotConfig.Vision.OPENCV_CENTER_TOLERANCE_PX;
    }

    /** True when a target is visible and vertically centered. */
    public boolean isCenteredY() {
        double err = getCenterErrorY();
        return !Double.isNaN(err)
                && Math.abs(err) <= RobotConfig.Vision.OPENCV_CENTER_TOLERANCE_PX;
    }

    /** Detection state for Driver Station / Panels (never calls update). */
    public void reportTelemetry() {
        // Some hubs only expose camera controls after the first frames.
        if (state == CameraState.STREAMING && !controlsApplied
                && controlAttempts < MAX_CONTROL_ATTEMPTS) {
            applyCameraControls();
        }

        telemetry.addData("OpenCV camera", getCameraStatus());
        if (state == CameraState.ERROR) {
            telemetry.addData("OpenCV open error", openErrorCode);
        }
        int frameWidth = currentPipeline().getFrameWidth();
        int frameHeight = currentPipeline().getFrameHeight();
        if (frameWidth <= 0 || frameHeight <= 0) {
            frameWidth = RobotConfig.Vision.CAMERA_WIDTH;
            frameHeight = RobotConfig.Vision.CAMERA_HEIGHT;
        }
        telemetry.addData("OpenCV stream", "%dx%d @ %.1f fps (%s %s)",
                frameWidth, frameHeight, getFps(),
                RobotConfig.Vision.CAMERA_STREAM_FORMAT,
                RobotConfig.Vision.CAMERA_ROTATION);
        telemetry.addData("OpenCV controls", controlStatus);
        telemetry.addData("OpenCV target", target);

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

        double errX = getCenterErrorX();
        double errY = getCenterErrorY();
        telemetry.addData("OpenCV X centered", isCentered()
                ? String.format("yes (err %+.0f px)", errX)
                : Double.isNaN(errX) ? "n/a"
                : String.format("no (err %+.0f px)", errX));
        telemetry.addData("OpenCV Y centered", isCenteredY()
                ? String.format("yes (err %+.0f px)", errY)
                : Double.isNaN(errY) ? "n/a"
                : String.format("no (err %+.0f px)", errY));
    }

    private static String fmtIn(double inches) {
        return Double.isNaN(inches) ? "n/a" : String.format("%.1f in", inches);
    }
}
