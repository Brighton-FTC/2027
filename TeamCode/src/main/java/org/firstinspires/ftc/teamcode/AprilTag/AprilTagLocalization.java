package org.firstinspires.ftc.teamcode.AprilTag;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Vision.HiveCell;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Objects;

/**
 * AprilTag tag-tilt sensing (FTC SDK 12.0 / BIOBUZZ season).
 *
 * <p>Per FTC docs
 * (<a href="https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_pose/apriltag-pose.html">AprilTag Pose</a>),
 * each {@link AprilTagDetection} carries an {@code ftcPose} with the tag's pose
 * relative to the camera:
 * position (x, y, z), orientation (pitch, roll, yaw), and derived
 * (range, bearing, elevation).
 *
 * <ul>
 *   <li>{@code ftcPose.pitch} = rotation about the FTC X axis. This is the tag's
 *       own tilt as seen by the camera — the value that changes while the match
 *       runs as the tag mechanism tips forward/back.</li>
 *   <li>Do not confuse it with {@code ftcPose.elevation}, which is the
 *       up/down angle the camera would have to tilt to center the tag. That
 *       changes when the <em>robot</em> moves, not when the tag tilts.</li>
 * </ul>
 *
 * <p>{@code ftcPose} is declared on the {@link AprilTagDetection} base class, so
 * it needs no cast and works for both {@link AprilTagSingleDetection} and
 * {@link AprilTagClusterDetection}. A single-tag detection only has a pose when
 * the tag is in the library with a known size (otherwise {@code ftcPose} is
 * null); always null-check before reading it.
 *
 * <p>No {@code robotPose} / field localization is done here on purpose: BIOBUZZ
 * tags move, so absolute robot pose from tags is invalid. Accordingly this class
 * never calls {@code setCameraPose(...)} — {@code ftcPose} only needs tag size
 * from the game tag library plus camera calibration.
 *
 * <p>SDK 12.0 breaking change: {@link AprilTagDetection} is a base class.
 * Always {@code instanceof}-check before reading id/metadata/center.
 *
 * <p>This class never calls {@code telemetry.update()} — the OpMode owns it.
 * Call {@link #close()} when done to release the camera.
 */
public class AprilTagLocalization {

    /** Sentinel meaning "no tag with a valid pitch is visible". */
    public static final double UNKNOWN_PITCH = RobotConfig.Launcher.UNKNOWN_POSE;

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    /** Default webcam from config. */
    public AprilTagLocalization(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME, telemetry);
    }

    public AprilTagLocalization(HardwareMap hardwareMap, String webcamName,
                                Telemetry telemetry) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(webcamName, "webcamName");
        Objects.requireNonNull(telemetry, "telemetry");
        this.telemetry = telemetry;

        // NOTE: no setCameraPose() — that input is only used to compute
        // detection.robotPose (field localization), which is intentionally
        // disabled because the tags move. ftcPose (relative tag pose) does
        // not need it.
        // Uses the BIOBUZZ library directly so the processor reports the 4
        // HIVE CELL clusters (RED SCORING 30-33, RED AUDIENCE 34-37,
        // BLUE AUDIENCE 38-41, BLUE SCORING 42-45) as AprilTagClusterDetection.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .enableLiveView(RobotConfig.Vision.ENABLE_LIVE_VIEW)
                .setCameraResolution(new Size(
                        RobotConfig.Vision.CAMERA_WIDTH,
                        RobotConfig.Vision.CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTag)
                .build();
    }

    public void startStreaming() {
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    /** Releases the camera. Call from the OpMode's stop path. */
    public void close() {
        visionPortal.close();
    }

    /** Live detections in the current frame (singles + clusters). */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /** Number of tags in the current frame. */
    public int getNumDetections() {
        return aprilTag.getDetections().size();
    }

    /** First detection in the current frame with a valid relative pose, or null. */
    private AprilTagDetection firstDetectionWithFtcPose() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection != null && detection.ftcPose != null) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Pitch (deg) of the first detected tag with a valid {@code ftcPose},
     * or {@link #UNKNOWN_PITCH} when nothing usable is visible.
     */
    public double getTagPitch() {
        AprilTagDetection detection = firstDetectionWithFtcPose();
        return detection != null ? detection.ftcPose.pitch : UNKNOWN_PITCH;
    }

    /**
     * Pitch (deg) of a specific single tag id, or {@link #UNKNOWN_PITCH} when
     * that id is not visible or has no pose (e.g. unknown tag size).
     *
     * <p>Note: BIOBUZZ member IDs 30-45 belong to clusters, so the SDK never
     * returns them as single detections — use {@link #getClusterPitch(HiveCell)}
     * for those. This method is kept for non-cluster tags only.
     */
    public double getTagPitch(int tagId) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.id == tagId) {
                    return single.ftcPose != null ? single.ftcPose.pitch : UNKNOWN_PITCH;
                }
            }
        }
        return UNKNOWN_PITCH;
    }

    /**
     * Pitch (deg) of the chosen HIVE CELL, or {@link #UNKNOWN_PITCH} when that
     * cell is not visible or has no pose.
     *
     * <p>Loops the current detections and matches them against the chosen
     * cell's member IDs from {@link HiveCell}:
     * <ul>
     *   <li>Cluster detections (the normal BIOBUZZ case — one
     *       {@link AprilTagClusterDetection} per cell): matched when the
     *       cluster's {@code metadata.name} / {@code metadata.shortName}
     *       resolves to the chosen cell via {@link HiveCell#forClusterName}.
     *       The cluster {@code ftcPose.pitch} is already the fused pitch of
     *       its member tags, so it is returned directly.</li>
     *   <li>Single detections (fallback, e.g. custom library): matched when
     *       {@code single.id} is one of the chosen cell's member IDs
     *       ({@link HiveCell#contains}).</li>
     * </ul>
     */
    public double getClusterPitch(HiveCell cell) {
        Objects.requireNonNull(cell, "cell");
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection == null || detection.ftcPose == null) {
                continue;
            }
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata == null) {
                    continue;
                }
                HiveCell seen = HiveCell.forClusterName(cluster.metadata.name);
                if (seen == null) {
                    seen = HiveCell.forClusterName(cluster.metadata.shortName);
                }
                if (seen == cell) {
                    return cluster.ftcPose.pitch;
                }
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (cell.contains(single.id)) {
                    return single.ftcPose.pitch;
                }
            }
        }
        return UNKNOWN_PITCH;
    }

    /** Pitch (deg) of the red CELL opposite the audience (IDs 30-33). */
    public double getRedScoringPitch() {
        return getClusterPitch(HiveCell.RED_SCORING);
    }

    /** Pitch (deg) of the red CELL on the audience side (IDs 34-37). */
    public double getRedAudiencePitch() {
        return getClusterPitch(HiveCell.RED_AUDIENCE);
    }

    /** Pitch (deg) of the blue CELL on the audience side (IDs 38-41). */
    public double getBlueAudiencePitch() {
        return getClusterPitch(HiveCell.BLUE_AUDIENCE);
    }

    /** Pitch (deg) of the blue CELL opposite the audience (IDs 42-45). */
    public double getBlueScoringPitch() {
        return getClusterPitch(HiveCell.BLUE_SCORING);
    }
    /** True when at least one detection currently has a valid {@code ftcPose}. */
    public boolean hasTagPose() {
        return firstDetectionWithFtcPose() != null;
    }

    /** Full per-detection telemetry (single tags + clusters), pitch-focused. */
    public void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            describeDetection(detection);
        }
        telemetry.addLine("key: PRY = Pitch, Roll & Yaw (tag tilt, deg). "
                + "Pitch = tag tilt about X; use ftcPose.pitch.");
    }

    /** Compact one-line-per-tag telemetry for driver debugging. */
    public void checkCase() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.ftcPose == null) {
                continue;
            }
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                String label = single.metadata != null ? single.metadata.name : ("ID " + single.id);
                telemetry.addData("Obj", "%s pitch %.1f", label, single.ftcPose.pitch);
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String label = cluster.metadata != null ? cluster.metadata.name : "cluster";
                HiveCell cell = cluster.metadata != null
                        ? HiveCell.forClusterName(cluster.metadata.name) : null;
                if (cell == null && cluster.metadata != null) {
                    cell = HiveCell.forClusterName(cluster.metadata.shortName);
                }
                String ids = cell != null ? java.util.Arrays.toString(cell.memberIds) : "";
                telemetry.addData("Obj", "%s %s pitch %.1f (%d%% found)",
                        label, ids, cluster.ftcPose.pitch, cluster.percentClusterFound);
            }
        }
    }

    private void describeDetection(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            if (single.metadata == null) {
                telemetry.addData("ID", single.id);
                return;
            }
            telemetry.addData("Detection", single.id + " " + single.metadata.name);
            if (single.ftcPose != null) {
                telemetry.addData("XYZ (inch)", "%.1f, %.1f, %.1f",
                        single.ftcPose.x, single.ftcPose.y, single.ftcPose.z);
                telemetry.addData("PRY (deg)", "%.1f, %.1f, %.1f",
                        single.ftcPose.pitch, single.ftcPose.roll, single.ftcPose.yaw);
                telemetry.addData("RBE", "%.1f, %.1f, %.1f",
                        single.ftcPose.range, single.ftcPose.bearing, single.ftcPose.elevation);
            } else {
                telemetry.addData("Pose", "no ftcPose for tag " + single.id
                        + " (not in library / unknown size)");
            }
        } else if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            telemetry.addData("Cluster",
                    cluster.metadata != null ? cluster.metadata.name : "unknown");
            telemetry.addData("Percent found", cluster.percentClusterFound);
            if (cluster.ftcPose != null) {
                telemetry.addData("XYZ (inch)", "%.1f, %.1f, %.1f",
                        cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z);
                telemetry.addData("PRY (deg)", "%.1f, %.1f, %.1f",
                        cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw);
            } else {
                telemetry.addData("Pose", "no ftcPose for cluster");
            }
        } else if (detection != null) {
            telemetry.addData("Detection", detection.getClass().getSimpleName());
            if (detection.ftcPose != null) {
                telemetry.addData("PRY (deg)", "%.1f, %.1f, %.1f",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
            }
        }
    }
}
