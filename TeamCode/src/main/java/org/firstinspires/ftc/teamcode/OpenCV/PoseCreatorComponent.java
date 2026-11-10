package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * Projects the OpenCV detections into field space using the robot pose the
 * component was built with.
 *
 * <p>Snapshot semantics: the constructor opens the camera, grabs whatever
 * detections the pipeline holds at that instant and freezes them against
 * {@code robotPose} — later robot motion does NOT re-project. Callers wanting
 * fresh data construct a new instance. The camera stream must have produced
 * frames before construction for any detections to exist.
 */
public class PoseCreatorComponent {

    private final double theta;

    private final double robotX;

    private final double robotY;

    private final SamplePipeline.Detection best;

    private final List<SamplePipeline.Detection> detections;

    public PoseCreatorComponent(OpenCVComponent.Target target, Pose robotPose){
        OpenCVComponent camera = new OpenCVComponent(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME, telemetry, target);
        this.detections = camera.getDetections();
        // Derive from the same snapshot, not a second pipeline read.
        this.best = detections.isEmpty() ? null : detections.get(0);
        this.theta = robotPose.heading();
        this.robotX = robotPose.x();
        this.robotY = robotPose.y();
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
