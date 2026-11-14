package org.firstinspires.ftc.teamcode.OpenCV;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Projects the OpenCV detections into field space using a robot pose.
 *
 * <p>Snapshot semantics: detections are frozen against the robot pose they
 * were read with — the constructor takes the first snapshot, and
 * {@link #refresh(Pose)} re-reads the pipeline and re-anchors the projection
 * to a new pose (same camera, no reopen).
 *
 * <p>The camera is constructor-injected and shared: the same webcam feeds
 * this locator AND the TeleOp's tilt estimator, so callers that do NOT own
 * the camera must not call {@link #closeCamera()}. Call {@link #setTarget}
 * to switch which pipeline is active (only one runs at a time).
 */
public class PoseCreatorComponent {

    private final OpenCVComponent camera;

    private double theta;

    private double robotX;

    private double robotY;

    private SamplePipeline.Detection best;

    private List<SamplePipeline.Detection> detections;

    /**
     * Wraps an already-created camera. Takes the first snapshot of the active
     * pipeline against {@code robotPose}; the stream must have produced
     * frames for anything to be visible ({@link #startCamera()} if the caller
     * owns the camera and has not started it yet).
     */
    public PoseCreatorComponent(OpenCVComponent camera, Pose robotPose){
        this.camera = camera;
        this.detections = camera.getDetections();
        // Derive from the same snapshot, not a second pipeline read.
        this.best = detections.isEmpty() ? null : detections.get(0);
        this.theta = robotPose.heading();
        this.robotX = robotPose.x();
        this.robotY = robotPose.y();
    }

    /**
     * Re-reads the pipeline and re-anchors the projection to the given pose —
     * the cheap way to get fresh data mid-run (no camera reopen).
     */
    public void refresh(Pose robotPose) {
        this.detections = camera.getDetections();
        this.best = detections.isEmpty() ? null : detections.get(0);
        this.theta = robotPose.heading();
        this.robotX = robotPose.x();
        this.robotY = robotPose.y();
    }

    /** Opens the camera and starts streaming (idempotent). */
    public void startCamera() {
        camera.startStreaming();
    }

    /**
     * Stops streaming and releases the camera device. Only call this when
     * this locator owns the camera (e.g. the pollen autonomous); a shared
     * camera is closed by its owner.
     */
    public void closeCamera() {
        camera.close();
    }

    /** Switches the active colour pipeline (takes effect on the next frames). */
    public void setTarget(OpenCVComponent.Target target) {
        camera.setTarget(target);
    }

    /** The currently active colour pipeline. */
    public OpenCVComponent.Target getTarget() {
        return camera.getTarget();
    }

    /** True when the selected ball has a usable field position. */
    public boolean hasTarget() {
        return best != null && usable(best);
    }

    /** Field X of the selected ball (NaN when there is no usable target). */
    public double getX(){
        return fieldX(best);
    }

    /** Field Y of the selected ball (NaN when there is no usable target). */
    public double getY(){
        return fieldY(best);
    }

    /** Heading that faces the selected ball from the robot (NaN when none). */
    public double getHeading(){
        return Math.atan2(getY() - robotY, getX() - robotX);
    }

    /** Selected ball as a Pedro pose, or null when nothing usable is visible. */
    public Pose createPose(){
        return hasTarget() ? new Pose(getX(), getY(), getHeading()) : null;
    }

    /**
     * Field position of the selected ball, or null when nothing usable is
     * visible — position twin of {@link #createPose()} (a ball has no heading).
     */
    public Vector2D getTargetPosition() {
        return hasTarget() ? Vector2D.cartesian(getX(), getY()) : null;
    }

    /**
     * Field positions of every usable detection EXCEPT the selected ball —
     * the obstacles a path to the target must not clip.
     */
    public List<Vector2D> getObstaclePositions() {
        List<Vector2D> obstacles = new ArrayList<>(detections.size());
        for (SamplePipeline.Detection d : detections) {
            if (d != best && usable(d)) {
                obstacles.add(Vector2D.cartesian(fieldX(d), fieldY(d)));
            }
        }
        return obstacles;
    }

    /** A detection is usable when its robot-relative offset is solvable. */
    private static boolean usable(SamplePipeline.Detection d) {
        return !Double.isNaN(d.forwardIn) && !Double.isNaN(d.lateralIn);
    }

    /**
     * Robot-relative (forwardIn, lateralIn) -> field frame. Lateral is
     * positive to the robot's RIGHT (image +X — see {@link SamplePipeline}).
     */
    private double fieldX(SamplePipeline.Detection d) {
        return d == null || !usable(d) ? Double.NaN
                : robotX + d.forwardIn * Math.cos(theta) + d.lateralIn * Math.sin(theta);
    }

    private double fieldY(SamplePipeline.Detection d) {
        return d == null || !usable(d) ? Double.NaN
                : robotY + d.forwardIn * Math.sin(theta) - d.lateralIn * Math.cos(theta);
    }

}
