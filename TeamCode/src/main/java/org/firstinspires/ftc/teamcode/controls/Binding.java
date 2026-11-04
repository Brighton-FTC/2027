package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.Optional;

import org.firstinspires.ftc.teamcode.controls.ControlAction.InputStyle;

/**
 * One key that one {@link ControlAction} is bound to in a
 * {@link DriverProfile}: which pad, a button or a trigger, and the input
 * style (defaults to the action's own style unless overridden).
 *
 * <p>Triggers are a separate FTCLib enum ({@link GamepadKeys.Trigger},
 * analog values read through {@code TriggerReader}); buttons are
 * {@link GamepadKeys.Button} read through {@code GamepadEx}.
 */
public final class Binding {

    /** Which physical gamepad the key lives on. */
    public enum Pad {
        /** gamepad1 — the primary driver controller. */
        DRIVER,
        /** gamepad2 — the operator controller. */
        OPERATOR
    }

    /** The key. Exactly one of {@link #button} / {@link #trigger} is set. */
    public final Optional<GamepadKeys.Button> button;
    public final Optional<GamepadKeys.Trigger> trigger;

    /** Which pad this key is on. */
    public final Pad pad;

    /** Effective input style (action default unless overridden). */
    public final InputStyle style;

    private Binding(GamepadKeys.Button button, GamepadKeys.Trigger trigger,
                    Pad pad, InputStyle style) {
        this.button = Optional.ofNullable(button);
        this.trigger = Optional.ofNullable(trigger);
        this.pad = pad;
        this.style = style;
    }

    /** Bind a digital button on the driver pad, using the action's default style. */
    public static Binding button(GamepadKeys.Button button) {
        return button(button, Pad.DRIVER, null);
    }

    /** Bind a digital button with an explicit style override. */
    public static Binding button(GamepadKeys.Button button, InputStyle style) {
        return button(button, Pad.DRIVER, style);
    }

    /** Bind a digital button on an explicit pad, with optional style override. */
    public static Binding button(GamepadKeys.Button button, Pad pad, InputStyle style) {
        return new Binding(button, null, pad, style);
    }

    /** Bind an (analog) trigger on the driver pad, using the action's default style. */
    public static Binding trigger(GamepadKeys.Trigger trigger) {
        return trigger(trigger, Pad.DRIVER, null);
    }

    /** Bind a trigger with an explicit style override. */
    public static Binding trigger(GamepadKeys.Trigger trigger, InputStyle style) {
        return trigger(trigger, Pad.DRIVER, style);
    }

    /** Bind a trigger on an explicit pad, with optional style override. */
    public static Binding trigger(GamepadKeys.Trigger trigger, Pad pad, InputStyle style) {
        return new Binding(null, trigger, pad, style);
    }

    @Override
    public String toString() {
        String key;
        if (button.isPresent()) {
            key = button.get().name();
        } else if (trigger.isPresent()) {
            key = trigger.get().name();
        } else {
            key = "UNBOUND";
        }
        return key + " [" + style + " @ " + pad + "]";
    }
}
