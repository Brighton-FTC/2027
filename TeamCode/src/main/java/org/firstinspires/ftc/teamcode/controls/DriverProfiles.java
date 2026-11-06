package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.PSButtons;

/**
 * The registry of driver control mappings. Each profile is one driver's
 * answer sheet, transcribed verbatim from their submissions — expressed
 * through {@link PSButtons} PlayStation aliases (plus the raw
 * {@code GamepadKeys} D-pad enum) so the telemetry cheat sheet reads
 * exactly like what the drivers say. All three drivers are assumed to use
 * PlayStation-style controllers; face-button letters from Driver J map to
 * their PS shapes (Y→triangle, B→circle, X→square, A→cross).
 *
 * <p>To swap drivers at a competition means editing
 * {@link #ACTIVE} on the dashboard, or (permanently) ONE line in ONE
 * constructor below — no TeleOp logic ever changes.
 *
 * <p>Naming conventions used for transcription (label → key):
 * L1/R1 = shoulders, L2/R2 = analog triggers, L3/R3 = stick clicks,
 * triangle/circle/X(square?)/square as per PS face buttons.
 */
@Configurable
public final class DriverProfiles {

    private DriverProfiles() {}

    /**
     * Name of the active profile ("A", "Y", "J" — case-insensitive).
     * Editable live on the Panels dashboard; reloads at next loop tick.
     */
    public static String ACTIVE = "A";

    /**
     * Universal drive-mode key, live on EVERY profile no matter what
     * {@link #ACTIVE} says — press to flip field-centric / robot-centric.
     *
     * <p>START (OPTIONS) is free: between them A, Y and J already claim every
     * face button, both bumpers, both triggers, both stick clicks and (for Y)
     * the whole D-pad, so this is the one deliberate key nobody loses.
     *
     * <p>It lives outside the profile maps on purpose — a {@link DriverProfile}
     * holds one key per action, so binding it inside a profile would overwrite
     * driver A's CIRCLE / driver Y's D-pad-right, and driver J (who submitted
     * no drive-mode key at all) would still have nothing. {@code GenericTeleop}
     * polls this on top of the active profile instead.
     */
    public static final Binding UNIVERSAL_DRIVE_MODE = Binding.button(PSButtons.OPTIONS);

    /**
     * Driver A's layout, exactly as submitted:
     * shoot L2, auto-aim L1, shoot-any-distance R1, ramp L3,
     * intake R2, flush triangle, umbrella R3, drive mode circle,
     * switch goal (hive) square.
     *
     * <p>SLOW_MODE intentionally unbound: their SFA key (R1) is the usual
     * slow-mode key, so binding it would double-fuse.
     */
    public static final DriverProfile A = DriverProfile.builder("A")
            .bind(ControlAction.SHOOT, Binding.trigger(PSButtons.L2))
            .bind(ControlAction.TOGGLE_AUTOAIM, Binding.button(PSButtons.L1))
            .bind(ControlAction.TOGGLE_SHOOT_ANY_DISTANCE, Binding.button(PSButtons.R1))
            .bind(ControlAction.TOGGLE_DRIVE_MODE, Binding.button(PSButtons.CIRCLE))
            .bind(ControlAction.TOGGLE_RAMP, Binding.button(PSButtons.L3))
            .bind(ControlAction.TOGGLE_INTAKE, Binding.trigger(PSButtons.R2))
            .bind(ControlAction.FLUSH_STORAGE, Binding.button(PSButtons.TRIANGLE))
            .bind(ControlAction.TOGGLE_UMBRELLA, Binding.button(PSButtons.R3))
            .bind(ControlAction.SWITCH_GOAL, Binding.button(PSButtons.SQUARE))
            .build();

    /**
     * Driver Y's layout, exactly as submitted:
     * shoot RT(R2), autoaim Dpad Left, SFA Dpad Up, ramp THE LEFT "SHAPE"
     * (interpreted as the left shoulder key = L1), intake LT(L2),
     * flush Dpad Down, umbrella O (Circle), drive mode Dpad Right,
     * switch goal X (Cross — need confirmation, could be Xbox-X/Square).
     */
    public static final DriverProfile Y = DriverProfile.builder("Y")
            .bind(ControlAction.SHOOT, Binding.trigger(PSButtons.R2))
            .bind(ControlAction.TOGGLE_AUTOAIM,
                    Binding.button(GamepadKeys.Button.DPAD_LEFT))
            .bind(ControlAction.TOGGLE_SHOOT_ANY_DISTANCE,
                    Binding.button(GamepadKeys.Button.DPAD_UP))
            // "left shape idk which it is" — need confirmation, assumed left bumper.
            .bind(ControlAction.TOGGLE_RAMP, Binding.button(PSButtons.L1))
            .bind(ControlAction.TOGGLE_INTAKE, Binding.trigger(PSButtons.L2))
            .bind(ControlAction.FLUSH_STORAGE,
                    Binding.button(GamepadKeys.Button.DPAD_DOWN))
            .bind(ControlAction.TOGGLE_UMBRELLA, Binding.button(PSButtons.CIRCLE)) // O
            .bind(ControlAction.TOGGLE_DRIVE_MODE,
                    Binding.button(GamepadKeys.Button.DPAD_RIGHT))
            .bind(ControlAction.SWITCH_GOAL, Binding.button(PSButtons.CROSS))
            .bind(ControlAction.SLOW_MODE,
                    Binding.button(PSButtons.R1, ControlAction.InputStyle.HOLD))
            .build();

    /**
     * Driver J's layout, exactly as submitted:
     * shooting R2, autoaim Y(triangle), SFA L2, ramp X(square), intake B(circle),
     * flush A(cross), umbrella R1, switch goal L1.
     *
     * <p>Their submission omitted drive mode (#8) — TOGGLE_DRIVE_MODE is
     * intentionally left unbound until they pick a key.
     */
    public static final DriverProfile J = DriverProfile.builder("J")
            .bind(ControlAction.SHOOT, Binding.trigger(PSButtons.R2))
            .bind(ControlAction.TOGGLE_AUTOAIM, Binding.button(PSButtons.TRIANGLE))
            .bind(ControlAction.TOGGLE_SHOOT_ANY_DISTANCE, Binding.trigger(PSButtons.L2))
            .bind(ControlAction.TOGGLE_RAMP, Binding.button(PSButtons.SQUARE))
            .bind(ControlAction.TOGGLE_INTAKE, Binding.button(PSButtons.CIRCLE))
            .bind(ControlAction.FLUSH_STORAGE, Binding.button(PSButtons.CROSS))
            .bind(ControlAction.TOGGLE_UMBRELLA, Binding.button(PSButtons.R1))
            .bind(ControlAction.SWITCH_GOAL, Binding.button(PSButtons.L1))
            .bind(ControlAction.SLOW_MODE,
                    Binding.button(PSButtons.R3, ControlAction.InputStyle.HOLD))
            .build();

    /**
     * The resolved active profile. Call once per init (or per loop if you want
     * live swapping while the dashboard changes {@link #ACTIVE}); unknown
     * names fall back to {@link #A} so a typo can never strand the robot.
     */
    public static DriverProfile active() {
        switch (ACTIVE.trim().toUpperCase()) {
            case "Y":
                return Y;
            case "J":
                return J;
            case "A":
            default:
                return A;
        }
    }
}
