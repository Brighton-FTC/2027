package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Gamepad keys referred to by their PlayStation names.
 *
 * <p>Use these in {@code DriverProfiles} bindings so profiles read exactly
 * like what the drivers say. Note FTCLib treats triggers as a separate
 * analog enum ({@link GamepadKeys.Trigger}), not buttons — hence the
 * separate L2/R2 constants.
 *
 * <p>D-pad and stick axes have no PS-name ambiguity; bind those directly
 * ({@code GamepadKeys.Button.DPAD_*}).
 */
public final class PSButtons {

    private PSButtons() {}

    // ---- face buttons ----
    public static final GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
    public static final GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
    public static final GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
    public static final GamepadKeys.Button CROSS = GamepadKeys.Button.A;

    // ---- shoulders (bumper-style digital) ----
    public static final GamepadKeys.Button L1 = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button R1 = GamepadKeys.Button.RIGHT_BUMPER;
    public static final GamepadKeys.Button L3 = GamepadKeys.Button.LEFT_STICK_BUTTON;
    public static final GamepadKeys.Button R3 = GamepadKeys.Button.RIGHT_STICK_BUTTON;

    // ---- analog triggers (FTCLib reads these off a separate enum) ----
    public static final GamepadKeys.Trigger L2 = GamepadKeys.Trigger.LEFT_TRIGGER;
    public static final GamepadKeys.Trigger R2 = GamepadKeys.Trigger.RIGHT_TRIGGER;

    // ---- misc keys ----
    public static final GamepadKeys.Button SHARE = GamepadKeys.Button.BACK;
    public static final GamepadKeys.Button OPTIONS = GamepadKeys.Button.START;
}
