package org.firstinspires.ftc.teamcode.AprilTag;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Objects;

/**
 * AprilTag-based localization (FTC SDK 12.0 / BIOBUZZ season).
 *
 * <p>SDK 12.0 breaking change: {@link AprilTagDetection} is a base class.
 * Concrete detections are {@link AprilTagSingleDetection} or
 * {@link AprilTagClusterDetection} — always {@code instanceof}-check before
 * reading id/metadata/center.
 *
 * <p>BIOBUZZ tags move, so they are not suitable for absolute field
 * localization. Use these helpers for aiming; they return
 * {@link RobotConfig.Launcher#UNKNOWN_POSE} when no solvable pose exists.
 *
 * <p>This class never calls {@code telemetry.update()} — the OpMode owns it.
 * Call {@link #close()} when done to release the camera.
 */
public class AprilTagLocalization {

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    /** Default camera pose (horizontal, facing forward) + webcam from config. */
    public AprilTagLocalization(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap,
                new Position(DistanceUnit.INCH, 0, 0, 0, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0),
                RobotConfig.Hardware.WEBCAM_NAME,
                telemetry);
    }

    public AprilTagLocalization(HardwareMap hardwareMap, Position cameraPosition,
                                YawPitchRollAngles cameraOrientation, String webcamId,
                                Telemetry telemetry) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(cameraPosition, "cameraPosition");
        Objects.requireNonNull(cameraOrientation, "cameraOrientation");
        Objects.requireNonNull(webcamId, "webcamId");
        Objects.requireNonNull(telemetry, "telemetry");
        this.telemetry = telemetry;

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamId))
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

    /** First detection in the current frame with a solvable robot pose, or null. */
    private AprilTagDetection firstDetectionWithPose() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection != null && detection.robotPose != null) {
                return detection;
            }
        }
        return null;
    }

    /** Robot X in inches, or UNKNOWN_POSE when no tag is solvable. */
    public double returnXPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        return detection != null
                ? detection.robotPose.getPosition().x
                : RobotConfig.Launcher.UNKNOWN_POSE;
    }

    /** Robot Y in inches, or UNKNOWN_POSE when no tag is solvable. */
    public double returnYPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        return detection != null
                ? detection.robotPose.getPosition().y
                : RobotConfig.Launcher.UNKNOWN_POSE;
    }

    /** Robot yaw in degrees, or 0 when no tag is solvable. */
    public double returnYawPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        return detection != null
                ? detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                : 0.0;
    }

    /** Full per-detection telemetry (single tags + clusters). */
    public void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            describeDetection(detection);
        }
    }

    /** Compact one-line-per-tag telemetry for driver debugging. */
    public void checkCase() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection == null || detection.robotPose == null) {
                continue;
            }
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                String label = single.metadata != null ? single.metadata.name : ("ID " + single.id);
                telemetry.addData("Obj", label);
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String label = cluster.metadata != null ? cluster.metadata.name : "cluster";
                telemetry.addData("Obj", label + " (" + cluster.percentClusterFound + "% found)");
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
            if (single.robotPose != null) {
                telemetry.addData("Pos", "%.1f, %.1f, %.1f",
                        single.robotPose.getPosition().x,
                        single.robotPose.getPosition().y,
                        single.robotPose.getPosition().z);
                telemetry.addData("YawDeg", single.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            } else {
                telemetry.addData("Pose", "not solvable for tag " + single.id);
            }
        } else if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            telemetry.addData("Cluster",
                    cluster.metadata != null ? cluster.metadata.name : "unknown");
            telemetry.addData("Percent found", cluster.percentClusterFound);
            if (cluster.robotPose != null) {
                telemetry.addData("Pos", "%.1f, %.1f",
                        cluster.robotPose.getPosition().x,
                        cluster.robotPose.getPosition().y);
                telemetry.addData("YawDeg",
                        cluster.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            } else {
                telemetry.addData("Pose", "not solvable for cluster");
            }
        } else if (detection != null) {
            telemetry.addData("Detection", detection.getClass().getSimpleName());
            if (detection.robotPose != null) {
                telemetry.addData("Pos", "%.1f, %.1f",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y);
            }
        }
    }
}
