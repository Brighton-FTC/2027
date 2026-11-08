package org.firstinspires.ftc.teamcode.Autonomous.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;

/**
 * Ivy commands for autoaim (turret tracking) and the distance-based shooter.
 *
 * <p>Both {@link #aim} and {@link #spinUp} are <i>sustained</i> commands: they
 * re-run their control loop every scheduler tick and only finish when they are
 * cancelled. That is deliberate — the turret has to keep tracking and the
 * wheel has to keep re-computing its speed while the robot drives, so the
 * normal pattern is to schedule them once for the whole match:
 *
 * <pre>
 * schedule(AutoAimCommands.aim(follower, turret));
 * schedule(AutoAimCommands.spinUp(follower, launcher));
 * </pre>
 *
 * and then compose the rest of the routine around them.
 */
public final class AutoAimCommands {

    private AutoAimCommands() {}

    /**
     * Tracks the goal from the follower's live pose every loop; stops the
     * turret motor when cancelled.
     */
    public static Command aim(Follower follower, TurretPIDComponent turret) {
        return Command.build()
                .setExecute(() -> turret.aimToObject(
                        follower.pose().x(),
                        follower.pose().y(),
                        follower.pose().heading()))
                .setEnd(endCondition -> turret.stop())
                .requiring(turret);
    }

    /**
     * Re-runs the ballistic shooter model from the follower's live pose every
     * loop; stops the wheel when cancelled.
     */
    public static Command spinUp(Follower follower, DynamicAngleComponent launcher) {
        return Command.build()
                .setExecute(() -> launcher.dynamicMotorPower(
                        follower.pose().x(),
                        follower.pose().y()))
                .setEnd(endCondition -> launcher.stop())
                .requiring(launcher);
    }

    /**
     * Finishes once the turret is within
     * {@link org.firstinspires.ftc.teamcode.config.RobotConfig.Turret#AIM_TOLERANCE_DEG}
     * of the goal, or after {@code timeoutMs} — whichever comes first.
     */
    public static Command waitForAim(Follower follower, TurretPIDComponent turret, double timeoutMs) {
        return race(
                waitUntil(() -> turret.isAimed(
                        follower.pose().x(),
                        follower.pose().y(),
                        follower.pose().heading())),
                waitMs(timeoutMs));
    }

    /** One-shot: cuts turret power now. */
    public static Command stopTurret(TurretPIDComponent turret) {
        return instant(turret::stop);
    }

    /** One-shot: cuts shooter power now. */
    public static Command stopShooter(DynamicAngleComponent launcher) {
        return instant(launcher::stop);
    }
}
