package org.firstinspires.ftc.teamcode.controls;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

/**
 * A named, complete-or-partial control layout for one driver: a map from
 * {@link ControlAction} to {@link Binding}. Instances are immutable — build
 * with the builder, then treat as a frozen lookup table.
 *
 * <p>Actions left out of the map are simply unbound (no input triggers them,
 * nothing crashes). GenericTeleop iterates this map every loop and dispatches
 * each active action's logical handler, so swapping drivers at a competition
 * is one config change: see {@link DriverProfiles}.
 */
public final class DriverProfile {

    /** Display name shown in telemetry (and matched against DriverProfiles.ACTIVE). */
    public final String name;

    private final Map<ControlAction, Binding> bindings;

    private DriverProfile(String name, Map<ControlAction, Binding> bindings) {
        this.name = name;
        this.bindings = Collections.unmodifiableMap(new EnumMap<>(bindings));
    }

    /** The binding for an action, or {@code null} when the action is unbound. */
    public Binding get(ControlAction action) {
        return bindings.get(action);
    }

    /** Immutable view of all bindings. */
    public Map<ControlAction, Binding> all() {
        return bindings;
    }

    @Override
    public String toString() {
        return name;
    }

    // ---- builder ----

    public static Builder builder(String name) {
        return new Builder(name);
    }

    public static final class Builder {
        private final String name;
        private final Map<ControlAction, Binding> map = new EnumMap<>(ControlAction.class);

        private Builder(String name) {
            this.name = name;
        }

        /** Add a binding (styles may still be overridden with {@link #style}). */
        public Builder bind(ControlAction action, Binding binding) {
            map.put(action, binding);
            return this;
        }

        /** Override the input style of an already-bound action (e.g. HOLD instead of TOGGLE). */
        public Builder style(ControlAction action, ControlAction.InputStyle style) {
            Binding b = map.get(action);
            if (b == null) {
                throw new IllegalArgumentException(
                        "Cannot set style on unbound action " + action + " in profile " + name);
            }
            if (b.button.isPresent()) {
                map.put(action, Binding.button(b.button.get(), b.pad, style));
            } else {
                map.put(action, Binding.trigger(b.trigger.get(), b.pad, style));
            }
            return this;
        }

        public DriverProfile build() {
            return new DriverProfile(name, map);
        }
    }
}
