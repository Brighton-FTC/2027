package org.firstinspires.ftc.teamcode.OpenCV;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;

import java.util.List;

/**
 * Collision-checked straight paths to vision targets.
 *
 * <p>Shared by the TeleOp pollen command ({@code GenericTeleop#pathToPollen})
 * and the pollen-chase autonomous routines, so both drive the same geometry:
 * a Pedro line from the current pose to the target with the heading
 * interpolated along the way, rejected when any obstacle sits inside the
 * keep-out radius ({@code RobotConfig.Field.POLLEN_PATH_CLEARANCE_IN}).
 */
public final class PollenPaths {

    private PollenPaths() {}

    /**
     * Straight pollen path from {@code start} to {@code target}, or null when
     * no safe path exists: the travel is under {@code minTravelIn}, the
     * coordinates are not solvable (NaN), or any obstacle lies within
     * {@code clearanceIn} of the line.
     *
     * <p>{@code obstacles} must NOT contain the target itself — it sits on
     * the path's endpoint by definition ({@code
     * PoseCreatorComponent#getObstaclePositions} already excludes it).
     */
    public static Path plan(Pose start, Pose target, List<Vector2D> obstacles,
                            double clearanceIn, double minTravelIn) {
        if (Double.isNaN(target.x()) || Double.isNaN(target.y())) {
            return null;
        }
        if (start.distance(target) < minTravelIn) {
            return null;
        }
        Path path = line(start.toVector2D(), target.toVector2D())
                .linear(start.heading(), target.heading());
        if (collides(start, target, obstacles, clearanceIn)) {
            return null;
        }
        return path;
    }

    /**
     * True when any obstacle lies within {@code clearanceIn} of the straight
     * segment start -> end. Point-to-segment distance — exact for a Line path.
     */
    public static boolean collides(Pose start, Pose end,
                                   List<Vector2D> obstacles, double clearanceIn) {
        Vector2D a = start.toVector2D();
        Vector2D seg = end.toVector2D().minus(a);
        double lenSq = seg.magnitudeSquared();
        if (obstacles == null) {
            return false;
        }
        for (Vector2D obstacle : obstacles) {
            double t = lenSq == 0.0
                    ? 0.0 : obstacle.minus(a).dot(seg) / lenSq;
            t = Math.max(0.0, Math.min(1.0, t));
            if (obstacle.minus(a.plus(seg.times(t))).magnitude() < clearanceIn) {
                return true;
            }
        }
        return false;
    }
}
