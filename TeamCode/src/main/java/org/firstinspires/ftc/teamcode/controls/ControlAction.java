package org.firstinspires.ftc.teamcode.controls;

/**
 * Every driver-facing action the robot supports, as an enum. A
 * {@link DriverProfile} maps each of these to a physical key on one of the
 * two gamepads, and {@code GenericTeleop} dispatches them — so adding a new
 * driver's layout means adding an entry to {@link DriverProfiles}, and
 * changing a binding means changing one line there.
 *
 * <p>The default input style of each action is declared here; a profile can
 * override it per binding if a driver prefers hold-to-toggle, etc.
 */
public enum ControlAction {
    // ---- shooting ----

    /** Kick one ball through (momentary — auto-closes after FIRE_KICK_SECONDS). */
    SHOOT("Shoot", InputStyle.PULSE),

    /** Toggle turret auto-aim (turret tracks the goal). */
    TOGGLE_AUTOAIM("Auto-aim", InputStyle.TOGGLE),

    /** Toggle shoot-from-any-distance (flywheel auto-power from pose). */
    TOGGLE_SHOOT_ANY_DISTANCE("Shoot any distance", InputStyle.TOGGLE),

    // ---- feed path ----

    /** Toggle intake (intake + transfer forward together). */
    TOGGLE_INTAKE("Intake", InputStyle.TOGGLE),

    /** Hold to flush the storage path backwards (unclog). */
    FLUSH_STORAGE("Flush storage", InputStyle.HOLD),

    // ---- mechanisms ----

    /** Toggle the ramp servo. NOT YET WIRED — no ramp component exists. */
    TOGGLE_RAMP("Ramp", InputStyle.TOGGLE),

    /** Toggle the umbrella servo. NOT YET WIRED — no umbrella component exists. */
    TOGGLE_UMBRELLA("Umbrella", InputStyle.TOGGLE),

    // ---- driving ----

    /** Toggle robot-centric / field-centric drive. */
    TOGGLE_DRIVE_MODE("Drive mode", InputStyle.TOGGLE),

    /**
     * Handle a tipped goal (manual override instead of auto goal-pick).
     * NOT YET WIRED — TurretPIDComponent/DynamicAngleComponent need goal
     * setters before this can re-target at runtime.
     */
    SWITCH_GOAL("Switch goal", InputStyle.TOGGLE),

    /** Hold (or press-to-toggle) to drive at reduced speed. May be unbound. */
    SLOW_MODE("Slow mode", InputStyle.HOLD);

    /** Human-readable label for telemetry. */
    public final String label;

    /** What pressing this action's key means by default. */
    public final InputStyle defaultStyle;

    ControlAction(String label, InputStyle defaultStyle) {
        this.label = label;
        this.defaultStyle = defaultStyle;
    }

    /** How a bound key is interpreted. */
    public enum InputStyle {
        /** Runs while the key is down; released = off. */
        HOLD,
        /** Press flips a latched boolean. */
        TOGGLE,
        /** Each press fires the action once (interval handled by the robot). */
        PULSE
    }
}
