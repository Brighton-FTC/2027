package org.firstinspires.ftc.teamcode.Autonomous.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Servo.ServoComponent;

import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Ivy commands for any {@link ServoComponent} (launch cap, ramp, umbrella…).
 *
 * <p>Every command here is an <i>instant</i>: it moves the servo once and
 * finishes immediately, so it can be dropped straight into a
 * {@code sequential(...)} without any waiting.
 */
public final class ServoCommands {

    private ServoCommands() {}

    /** Drives the servo to its configured open position. */
    public static Command open(ServoComponent servo) {
        return instant(servo::open);
    }

    /** Drives the servo to its configured closed position. */
    public static Command close(ServoComponent servo) {
        return instant(servo::close);
    }

    /** Drives the servo to its configured default (init) position. */
    public static Command toDefault(ServoComponent servo) {
        return instant(servo::toDefault);
    }

    /** Flips the servo between open and closed, returning the new state. */
    public static Command toggle(ServoComponent servo) {
        return instant(servo::toggle);
    }
}
