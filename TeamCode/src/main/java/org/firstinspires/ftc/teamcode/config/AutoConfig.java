package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Every pose and timing an autonomous routine needs.
 *
 * <p>All values are public statics so they are editable from the dashboard
 * (Panels) without touching logic code — same convention as
 * {@link RobotConfig}.
 *
 * <p><b>Pose convention</b> (identical to {@link RobotConfig.Field}):
 * Pedro field frame, inches, origin at the field corner, 144 x 144.
 * {@code 0 deg} = +X (toward the blue wall), {@code 90 deg} = +Y.
 * Red alliance owns the low-X half, blue the high-X half.
 *
 * <p><b>Routine geometry.</b> Every start pose below is the documented
 * wall-touching spot from {@link RobotConfig.Field.Start}, so an auton can be
 * selected for the exact place the robot is set down. The four routine pose
 * sets are the same shape, mirrored across the centre of the field
 * (x = 72, y = 72) so that a fix to one side can be copied to the others.
 *
 * <p>The {@code controlN} poses are Bézier control points used by
 * {@code GenericAutonomous#buildPaths()} to bow the drive between the scoring
 * station and each pickup.
 */
@Configurable
public final class AutoConfig {

    private AutoConfig() {}

    /** Auton-only timings, in milliseconds. */
    @Configurable
    public static final class Timing {
        private Timing() {}

        /** Longest we wait for the turret to line up before shooting anyway. */
        public static double AIM_TIMEOUT_MS = 1200;
        /** Pause between opening the launch cap and feeding the shot. */
        public static double SHOOT_SETTLE_MS = 500;
        /** How long the launch cap stays open (and the transfer runs) per shot. */
        public static double SHOOT_KICK_MS = 700;
    }

    @Configurable
    public static final class AutoPoses {
        private AutoPoses() {}

        /** Radian headings — every pose in this file passes radians. */
        public static final PoseFactory poseFactory = PoseFactory.radians();

        // ------------------------------------------------------------------
        // Shared field anchors (alliance-neutral)
        // ------------------------------------------------------------------

        public static Pose flowerAudience = poseFactory.of(72, 12, Math.PI / 2);

        public static Pose flowerBlueAlliance = poseFactory.of(132, 72, Math.PI);

        public static Pose flowerScoring = poseFactory.of(72, 132, 3 * Math.PI / 2);

        public static Pose flowerRedAlliance = poseFactory.of(12, 72, 0);

        public static Pose redGardenPollen = poseFactory.of(12, 12, 5 * Math.PI / 4);

        public static Pose blueGardenPollen = poseFactory.of(132, 132, Math.PI / 4);

        // ------------------------------------------------------------------
        // Shooting stations — one per HIVE CELL, used by autoaim/shooter.
        // RedShoot/BlueShoot are the alliance-level aliases kept from the
        // original config; they now point at real poses instead of null.
        // ------------------------------------------------------------------

        public static Pose redScoringShot = poseFactory.of(60, 104, Math.toRadians(-90));
        public static Pose redAudienceShot = poseFactory.of(60, 40, Math.toRadians(90));
        public static Pose blueScoringShot = poseFactory.of(84, 104, Math.toRadians(-90));
        public static Pose blueAudienceShot = poseFactory.of(84, 40, Math.toRadians(90));

        /** Primary shooting pose per alliance (used when no CELL is specified). */
        public static Pose RedShoot = redScoringShot;
        public static Pose BlueShoot = blueScoringShot;

        // ==================================================================
        // RED 1 — north-wall start, scores the SCORING CELL
        // ==================================================================

        public static Pose red1Start = poseFactory.of(36, 132, Math.toRadians(-59));
        public static Pose red1StartScore = poseFactory.of(42.2, 121.7, Math.toRadians(-59));
        public static Pose red1Score = redScoringShot;
        public static Pose red1Pickup1 = poseFactory.of(24, 84, Math.toRadians(180));
        public static Pose red1Pickup2 = poseFactory.of(24, 60, Math.toRadians(180));
        public static Pose red1Pickup3 = poseFactory.of(24, 36, Math.toRadians(180));
        public static Pose red1Control1 = poseFactory.of(46, 100, 0);
        public static Pose red1Control2 = poseFactory.of(38, 82, 0);
        public static Pose red1Control3 = poseFactory.of(38, 58, 0);
        public static Pose red1Park = poseFactory.of(30, 24, Math.toRadians(180));

        // ==================================================================
        // RED 2 — south-wall start, scores the AUDIENCE CELL
        // ==================================================================

        public static Pose red2Start = poseFactory.of(36, 12, Math.toRadians(59));
        public static Pose red2StartScore = poseFactory.of(42.2, 22.3, Math.toRadians(59));
        public static Pose red2Score = redAudienceShot;
        public static Pose red2Pickup1 = poseFactory.of(24, 60, Math.toRadians(180));
        public static Pose red2Pickup2 = poseFactory.of(24, 84, Math.toRadians(180));
        public static Pose red2Pickup3 = poseFactory.of(24, 108, Math.toRadians(180));
        public static Pose red2Control1 = poseFactory.of(46, 44, 0);
        public static Pose red2Control2 = poseFactory.of(38, 62, 0);
        public static Pose red2Control3 = poseFactory.of(38, 86, 0);
        public static Pose red2Park = poseFactory.of(30, 120, Math.toRadians(180));

        // ==================================================================
        // RED 2 TIP — start -> flower -> side -> shooting station
        // ==================================================================

        /**
         * Degrees headings — the *TIP routines were tuned in degrees, so their
         * sections keep the numbers exactly as they were on the field. Every
         * other section above uses {@link #poseFactory} (radians).
         */
        public static final PoseFactory tipPoseFactory = PoseFactory.degrees();

        public static Pose red2TipStart = tipPoseFactory.of(56, 8, 90);
        public static Pose red2TipStartToFlower = tipPoseFactory.of(12.75, 46.75, 180);
        public static Pose red2TipStartToFlowerControl1 = tipPoseFactory.of(34.75, 50.75, 0);
        public static Pose red2TipFlowerToSide = tipPoseFactory.of(10.75, 5.75, 270);
        public static Pose red2TipFlowerToSideSegment1Heading = tipPoseFactory.of(10.75, 5.75, 180);
        public static Pose red2TipFlowerToSideSegment2Start = tipPoseFactory.of(10.75, 5.75, 180);
        public static Pose red2TipFlowerToSideSegment2End = tipPoseFactory.of(10.75, 5.75, 270);
        public static Pose red2TipSideToShoot = tipPoseFactory.of(7.75, 102.75, 270);
        public static Pose red2TipSideToShootControl1 = tipPoseFactory.of(13.75, 54.75, 0);

        // ==================================================================
        // BLUE 1 — north-wall start, scores the SCORING CELL
        // ==================================================================

        public static Pose blue1Start = poseFactory.of(108, 132, Math.toRadians(-121));
        public static Pose blue1StartScore = poseFactory.of(101.8, 121.7, Math.toRadians(-121));
        public static Pose blue1Score = blueScoringShot;
        public static Pose blue1Pickup1 = poseFactory.of(120, 84, 0);
        public static Pose blue1Pickup2 = poseFactory.of(120, 60, 0);
        public static Pose blue1Pickup3 = poseFactory.of(120, 36, 0);
        public static Pose blue1Control1 = poseFactory.of(98, 100, 0);
        public static Pose blue1Control2 = poseFactory.of(106, 82, 0);
        public static Pose blue1Control3 = poseFactory.of(106, 58, 0);
        public static Pose blue1Park = poseFactory.of(114, 24, 0);

        // ==================================================================
        // BLUE 2 — south-wall start, scores the AUDIENCE CELL
        // ==================================================================

        public static Pose blue2Start = poseFactory.of(108, 12, Math.toRadians(121));
        public static Pose blue2StartScore = poseFactory.of(101.8, 22.3, Math.toRadians(121));
        public static Pose blue2Score = blueAudienceShot;
        public static Pose blue2Pickup1 = poseFactory.of(120, 60, 0);
        public static Pose blue2Pickup2 = poseFactory.of(120, 84, 0);
        public static Pose blue2Pickup3 = poseFactory.of(120, 108, 0);
        public static Pose blue2Control1 = poseFactory.of(98, 44, 0);
        public static Pose blue2Control2 = poseFactory.of(106, 62, 0);
        public static Pose blue2Control3 = poseFactory.of(106, 86, 0);
        public static Pose blue2Park = poseFactory.of(114, 120, 0);

        // ==================================================================
        // BLUE 2 TIP — start -> flower -> side -> shooting station
        // Exact mirror of RED 2 TIP across x = 72 (x -> 144 - x, heading ->
        // 180 - heading), so the two routines drive identical shapes on their
        // own halves of the field.
        // ==================================================================

        public static Pose blue2TipStart = tipPoseFactory.of(88, 8, 90);
        public static Pose blue2TipStartToFlower = tipPoseFactory.of(131.25, 46.75, 0);
        public static Pose blue2TipStartToFlowerControl1 = tipPoseFactory.of(109.25, 50.75, 180);
        public static Pose blue2TipFlowerToSide = tipPoseFactory.of(133.25, 5.75, 270);
        public static Pose blue2TipFlowerToSideSegment1Heading = tipPoseFactory.of(133.25, 5.75, 0);
        public static Pose blue2TipFlowerToSideSegment2Start = tipPoseFactory.of(133.25, 5.75, 0);
        public static Pose blue2TipFlowerToSideSegment2End = tipPoseFactory.of(133.25, 5.75, 270);
        public static Pose blue2TipSideToShoot = tipPoseFactory.of(136.25, 102.75, 270);
        public static Pose blue2TipSideToShootControl1 = tipPoseFactory.of(130.25, 54.75, 180);

        // ==================================================================
        // RED 3 TIP — start -> flower -> side -> shoot -> flower
        // Same first two legs as RED 2 TIP; the shot is then taken from the
        // north side and the robot goes back out to a flower.
        // ==================================================================

        public static Pose red3TipStart = tipPoseFactory.of(56, 8, 90);
        public static Pose red3TipStartToFlower = tipPoseFactory.of(12.75, 46.75, 180);
        public static Pose red3TipStartToFlowerControl1 = tipPoseFactory.of(34.75, 50.75, 0);
        public static Pose red3TipFlowerToSide = tipPoseFactory.of(10.75, 5.75, 270);
        public static Pose red3TipFlowerToSideSegment1Heading = tipPoseFactory.of(10.75, 5.75, 180);
        public static Pose red3TipFlowerToSideSegment2Start = tipPoseFactory.of(10.75, 5.75, 180);
        public static Pose red3TipFlowerToSideSegment2End = tipPoseFactory.of(10.75, 5.75, 270);
        public static Pose red3TipSideToShoot = tipPoseFactory.of(46.75, 130.75, 90);
        public static Pose red3TipSideToShootSegment1Heading = tipPoseFactory.of(46.75, 130.75, 270);
        public static Pose red3TipSideToShootSegment2Start = tipPoseFactory.of(46.75, 130.75, 270);
        public static Pose red3TipSideToShootSegment2End = tipPoseFactory.of(46.75, 130.75, 90);
        public static Pose red3TipShootToFlower = tipPoseFactory.of(14.75, 119.75, 90);

        // ==================================================================
        // BLUE 3 TIP — exact mirror of RED 3 TIP across x = 72
        // (x -> 144 - x, heading -> 180 - heading).
        // ==================================================================

        public static Pose blue3TipStart = tipPoseFactory.of(88, 8, 90);
        public static Pose blue3TipStartToFlower = tipPoseFactory.of(131.25, 46.75, 0);
        public static Pose blue3TipStartToFlowerControl1 = tipPoseFactory.of(109.25, 50.75, 180);
        public static Pose blue3TipFlowerToSide = tipPoseFactory.of(133.25, 5.75, 270);
        public static Pose blue3TipFlowerToSideSegment1Heading = tipPoseFactory.of(133.25, 5.75, 0);
        public static Pose blue3TipFlowerToSideSegment2Start = tipPoseFactory.of(133.25, 5.75, 0);
        public static Pose blue3TipFlowerToSideSegment2End = tipPoseFactory.of(133.25, 5.75, 270);
        public static Pose blue3TipSideToShoot = tipPoseFactory.of(97.25, 130.75, 90);
        public static Pose blue3TipSideToShootSegment1Heading = tipPoseFactory.of(97.25, 130.75, 270);
        public static Pose blue3TipSideToShootSegment2Start = tipPoseFactory.of(97.25, 130.75, 270);
        public static Pose blue3TipSideToShootSegment2End = tipPoseFactory.of(97.25, 130.75, 90);
        public static Pose blue3TipShootToFlower = tipPoseFactory.of(129.25, 119.75, 90);
    }
}
