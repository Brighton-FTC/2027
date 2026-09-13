package org.firstinspires.ftc.teamcode.AprilTag;
import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * AprilTag-based localization, updated for FTC SDK 12.0 (2026-2027 BIOBUZZ season).
 *
 * <p>SDK 12.0 breaking change: {@link AprilTagDetection} is now a base class. Concrete
 * detections are either {@link AprilTagSingleDetection} (standalone tag, carries
 * {@code id}/{@code metadata}/{@code center}) or {@link AprilTagClusterDetection}
 * (co-planar tag group, carries cluster {@code metadata}). Code must {@code instanceof}
 * check before accessing those fields. See
 * https://ftc-docs.firstinspires.org/apriltag-clusters
 *
 * <p>NOTE (per SDK 12.0 release notes): BIOBUZZ AprilTags move, so they are not suitable
 * for absolute field localization. The helpers below return robot-pose estimates derived
 * from visible tags/clusters (best used for aiming), or 1000 when no usable pose exists.
 */
@Configurable
public class AprilTagLocalization {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private final Telemetry telemetry;

    /* Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    public AprilTagLocalization(HardwareMap hardwareMap, Position cameraPosition, YawPitchRollAngles cameraOrientation, String webcamID, Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // BIOBUZZ (2026-2027) tag library; getCurrentGameTagLibrary()
                // tracks the active season game.
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        builder.setCamera(hardwareMap.get(WebcamName.class, webcamID));
        builder.enableLiveView(true);
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);


        builder.addProcessor(aprilTag);


        visionPortal = builder.build();


    }

    public void startStreaming(){
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
        telemetry.update();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    /**
     * Returns the first detection in the current frame that has a solvable
     * robot pose, or null if none is available. Works for both single tags
     * and tag clusters (SDK 12.0+).
     */
    private AprilTagDetection firstDetectionWithPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.robotPose != null) {
                return detection;
            }
        }
        return null;
    }

    public double returnYPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        if (detection != null) {
            return detection.robotPose.getPosition().y;
        }
        return 1000;
    }

    public double returnXPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        if (detection != null) {
            return detection.robotPose.getPosition().x;
        }
        return 1000;
    }

    public double returnYawPosition() {
        AprilTagDetection detection = firstDetectionWithPose();
        if (detection != null) {
            return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return 0;
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());



        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if (singleDet.metadata != null) {
                    telemetry.addData("Detection item", singleDet.id + singleDet.metadata.name);
                    // Only use tags that don't have Obelisk in them

                    if (singleDet.robotPose != null) {
                        telemetry.addData("PositionX",
                                singleDet.robotPose.getPosition().x);
                        telemetry.addData("PosY",
                                singleDet.robotPose.getPosition().y);
                        telemetry.addData("PosZ",
                                singleDet.robotPose.getPosition().z);
                        telemetry.addData("Pitch", singleDet.robotPose.getOrientation().getPitch(AngleUnit.DEGREES));
                        telemetry.addData("Roll", singleDet.robotPose.getOrientation().getRoll(AngleUnit.DEGREES));
                        telemetry.addData("Yaw", singleDet.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                    } else {
                        telemetry.addData("Pose", "not solvable for tag " + singleDet.id);
                    }
                    telemetry.update();
                } else {
                    telemetry.addData("ID", singleDet.id);
                    telemetry.addData("Center", singleDet.center.x + singleDet.center.y);
                    telemetry.addData("metadata", singleDet.metadata);
                    telemetry.update();
                }
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addData("Cluster", clusterDet.metadata != null ? clusterDet.metadata.name : "unknown");
                telemetry.addData("Percent found", clusterDet.percentClusterFound);
                if (clusterDet.robotPose != null) {
                    telemetry.addData("PositionX", clusterDet.robotPose.getPosition().x);
                    telemetry.addData("PosY", clusterDet.robotPose.getPosition().y);
                    telemetry.addData("PosZ", clusterDet.robotPose.getPosition().z);
                    telemetry.addData("Yaw", clusterDet.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                } else {
                    telemetry.addData("Pose", "not solvable for cluster");
                }
                telemetry.update();
            } else {
                // Unknown detection subtype; robotPose lives on the base class.
                telemetry.addData("Detection", detection.getClass().getSimpleName());
                if (detection.robotPose != null) {
                    telemetry.addData("PosX", detection.robotPose.getPosition().x);
                    telemetry.addData("PosY", detection.robotPose.getPosition().y);
                }
                telemetry.update();
            }
        }
        telemetry.update();// end for() loop

    }   // end method telemetryAprilTag()

    public void checkCase(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection == null || detection.robotPose == null) {
                continue;
            }
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                String label = singleDet.metadata != null ? singleDet.metadata.name : ("ID " + singleDet.id);
                telemetry.addData("Obj", label);
                telemetry.update();
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                String label = clusterDet.metadata != null ? clusterDet.metadata.name : "cluster";
                telemetry.addData("Obj", label + " (" + clusterDet.percentClusterFound + "% found)");
                telemetry.update();
            }
        }
        telemetry.update();

    }
}
