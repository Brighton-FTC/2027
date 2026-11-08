package org.firstinspires.ftc.teamcode.Autonomous.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.IntakeMotorComponent;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;

/**
 * Ivy commands for the intake roller.
 *
 * <p><b>Important:</b> {@link #collect} / {@link #eject} are <i>sustained</i>
 * commands — like every "hold this motor on" command they never finish on
 * their own. Compose them with a group that ends them, e.g.
 * {@code deadline(follow(follower, path), collect(intake))} or
 * {@code race(collect(intake), waitMs(500))}. The roller is stopped from the
 * command's {@code end()}, so cancellation is safe.
 *
 * <p>For a one-liner that runs for a fixed time use {@link #collectFor} /
 * {@link #ejectFor}.
 */
public final class IntakeCommands {

    private IntakeCommands() {}

    /** Runs the intake until the command is cancelled; stops the roller on end. */
    public static Command collect(IntakeMotorComponent intake) {
        return Command.build()
                .setStart(intake::startMotor)
                .setEnd(endCondition -> intake.stopMotor())
                .requiring(intake);
    }

    /** Reverses the intake (unclog) until cancelled; stops the roller on end. */
    public static Command eject(IntakeMotorComponent intake) {
        return Command.build()
                .setStart(intake::reverseMotor)
                .setEnd(endCondition -> intake.stopMotor())
                .requiring(intake);
    }

    /** Runs the intake for {@code millis} milliseconds, then stops it. */
    public static Command collectFor(IntakeMotorComponent intake, double millis) {
        return race(collect(intake), waitMs(millis));
    }

    /** Reverses the intake for {@code millis} milliseconds, then stops it. */
    public static Command ejectFor(IntakeMotorComponent intake, double millis) {
        return race(eject(intake), waitMs(millis));
    }

    /** One-shot: cuts intake power now. */
    public static Command stop(IntakeMotorComponent intake) {
        return instant(intake::stopMotor);
    }
}
