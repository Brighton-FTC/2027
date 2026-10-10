package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.EnumMap;
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

    /** Area (px^2) of the active pipeline's contour, or 0 when none. */
    public double getLargestArea() {
        return currentPipeline().getLargestArea();
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
     * True when a target is visible and within
     * {@link RobotConfig.Vision#OPENCV_CENTER_TOLERANCE_PX} of frame center.
     */
    public boolean isCentered() {
        double err = getCenterErrorX();
        return !Double.isNaN(err)
                && Math.abs(err) <= RobotConfig.Vision.OPENCV_CENTER_TOLERANCE_PX;
    }

    /** Detection state for Driver Station / Panels (never calls update). */
    public void reportTelemetry() {
        telemetry.addData("OpenCV target", target);
        telemetry.addData("OpenCV streaming", streaming);
        if (openErrorCode != 0) {
            telemetry.addData("OpenCV open error", openErrorCode);
        }
        if (!hasTarget()) {
            telemetry.addData("OpenCV contour", "none");
            return;
        }
        telemetry.addData("OpenCV area (px^2)", "%.0f", getLargestArea());
        double err = getCenterErrorX();
        telemetry.addData("OpenCV centerErr (px)",
                Double.isNaN(err) ? "n/a" : String.format("%.0f", err));
        telemetry.addData("OpenCV centered", isCentered());
    }
}
