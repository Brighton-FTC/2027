package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Gamepad buttons referred to by their PlayStation names.
 * For sticks, bumpers, D-pad, etc. see {@link GamepadKeys.Button}.
 */
public final class PSButtons {

    private PSButtons() {}

    public static final GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
    public static final GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
    public static final GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;
    public static final GamepadKeys.Button CROSS = GamepadKeys.Button.A;

    public static final GamepadKeys.Button SHARE = GamepadKeys.Button.BACK;
    public static final GamepadKeys.Button OPTIONS = GamepadKeys.Button.START;
}
