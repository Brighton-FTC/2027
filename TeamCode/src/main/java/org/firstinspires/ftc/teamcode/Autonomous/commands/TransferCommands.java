package org.firstinspires.ftc.teamcode.Autonomous.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;

/**
 * Ivy commands for the transfer roller (the path between intake and shooter).
 *
 * <p>Same contract as {@link IntakeCommands}: {@link #feed} and
 * {@link #reverse} are sustained commands that end when cancelled, and the
 * roller is stopped from {@code end()}.
 */
public final class TransferCommands {

    private TransferCommands() {}

    /** Feeds toward the shooter until cancelled; stops the roller on end. */
    public static Command feed(FlyWheelMotorComponent transfer) {
        return Command.build()
                .setStart(() -> transfer.runMotorAt(RobotConfig.Transfer.FORWARD_POWER))
                .setEnd(endCondition -> transfer.stopMotor())
                .requiring(transfer);
    }

    /** Runs the transfer backwards (unclog) until cancelled; stops on end. */
    public static Command reverse(FlyWheelMotorComponent transfer) {
        return Command.build()
                .setStart(() -> transfer.runMotorAt(RobotConfig.Transfer.REVERSE_POWER))
                .setEnd(endCondition -> transfer.stopMotor())
                .requiring(transfer);
    }

    /** Feeds for {@code millis} milliseconds, then stops. */
    public static Command feedFor(FlyWheelMotorComponent transfer, double millis) {
        return race(feed(transfer), waitMs(millis));
    }

    /** Reverses for {@code millis} milliseconds, then stops. */
    public static Command reverseFor(FlyWheelMotorComponent transfer, double millis) {
        return race(reverse(transfer), waitMs(millis));
    }

    /** One-shot: cuts transfer power now. */
    public static Command stop(FlyWheelMotorComponent transfer) {
        return instant(transfer::stopMotor);
    }
}
