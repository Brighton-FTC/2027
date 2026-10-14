package org.firstinspires.ftc.teamcode.config;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * Single place to configure robot controls.
 *
 * <p>Every driver/operator action is one {@code *_BUTTON} field — nothing is
 * hard-coded in the TeleOp. All fields are dashboard-tunable (Panels), so
 * bindings can be changed without touching logic code. Gamepads are fixed by
 * role: driver actions live on gamepad1, operator actions on gamepad2.
 *
 * <p>Buttons use PlayStation names via {@link PSButtons} — same class as
 * {@code Brighton-FTC/2026}. Face buttons MUST use {@link PSButtons}
 * ({@code SQUARE}, {@code CROSS}, {@code CIRCLE}, {@code TRIANGLE},
 * {@code SHARE}, {@code OPTIONS}); bumpers / sticks / D-pad have no PS alias
 * so they use {@link GamepadKeys.Button} directly (same pattern as the
 * reference repo).
 *
 * <p>Simplified default scheme (intuitive roles):
 * <ul>
 *   <li>Driver (gamepad1): sticks drive, {@link #SLOW_BUTTON} (RIGHT_BUMPER) held for
 *       slow mode, {@link #FIELD_CENTRIC_BUTTON} (SQUARE) toggles centricity.</li>
 *   <li>Operator (gamepad2): {@link #COLLECT_BUTTON} (CROSS) held to collect
 *       (intake + transfer + kicker feed together), {@link #AIM_AND_SPIN_BUTTON}
 *       (CIRCLE) toggles turret auto-aim + flywheel spin-up together (never one
 *       without the other), {@link #FIRE_BUTTON} (TRIANGLE) kicks one shot,
 *       {@link #UNCLOG_BUTTON} (DPAD_DOWN) held to reverse the feed path.</li>
 * </ul>
 *
 * <p>HOLD actions run only while held (release auto-stops — nothing gets stuck
 * on). TOGGLE actions latch on press. FIRE is a momentary pulse that
 * auto-closes after {@link #FIRE_KICK_SECONDS}.
 */
@Configurable
public final class RobotControls {

    private RobotControls() {}

    // ---- driver (gamepad1) ----

    /** Hold for slow driving (see {@link #SLOW_IS_TOGGLE}). */
    public static GamepadKeys.Button SLOW_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;
    /**
     * False (default) = slow while held, release for full speed — can't get
     * stuck in slow mode. True = classic press-to-toggle.
     */
    public static boolean SLOW_IS_TOGGLE = false;

    /** Press to toggle field-centric / robot-centric driving. */
    public static GamepadKeys.Button FIELD_CENTRIC_BUTTON = PSButtons.SQUARE;

    // ---- operator (gamepad2) ----

    /**
     * Hold to collect: intake forward + transfer forward + kicker open together.
     * Release stops everything and closes the kicker. One button replaces the
     * old intake-toggle / transfer-toggle / combo / kicker-open-close cluster.
     */
    public static GamepadKeys.Button COLLECT_BUTTON = PSButtons.CROSS;
    /** Whether COLLECT also opens the kicker to feed the shooter. */
    public static boolean COLLECT_OPENS_KICKER = true;

    /**
     * Press to toggle turret auto-aim + flywheel spin-up together. They are
     * deliberately linked — aiming without spinning (or vice versa) is never
     * useful, so one button owns both.
     */
    public static GamepadKeys.Button AIM_AND_SPIN_BUTTON = PSButtons.CIRCLE;

    /**
     * Press to fire: opens the kicker for {@link #FIRE_KICK_SECONDS}, then it
     * auto-closes. Ignored while {@link #UNCLOG_BUTTON} is held.
     */
    public static GamepadKeys.Button FIRE_BUTTON = PSButtons.TRIANGLE;
    /** How long (seconds) the kicker stays open per FIRE press. */
    public static double FIRE_KICK_SECONDS = 0.35;

    /**
     * Hold to unclog: reverses intake + transfer and forces the kicker closed.
     * Release auto-stops. Takes precedence over COLLECT while both are held.
     */
    public static GamepadKeys.Button UNCLOG_BUTTON = GamepadKeys.Button.DPAD_DOWN;


    public static GamepadKeys.Button RAMP_BUTTON = GamepadKeys.Button.DPAD_UP;
}
