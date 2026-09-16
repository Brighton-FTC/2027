package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Single place to tune the whole robot.
 *
 * <p>Hardware names must match the RC configuration file. Physics / PID /
 * power values are all public statics so they are editable from Dashboard
 * (Panels) without touching logic code.
 */
@Configurable
public final class RobotConfig {

    private RobotConfig() {}

    /** HardwareMap device names. Must match the robot configuration file. */
    @Configurable
    public static final class Hardware {
        private Hardware() {}

        public static String TURRET_MOTOR = "turretMotor";
        public static String FLYWHEEL_MOTOR = "flyWheelMotor";
        public static String TRANSFER_MOTOR = "transferMotor";
        public static String INTAKE_MOTOR = "intakeMotor";
        public static String LAUNCH_CAP_SERVO = "launchCap";
        public static String WEBCAM_NAME = "Webcam 1";
    }

    /** Driver / drivetrain behavior. */
    @Configurable
    public static final class Drive {
        private Drive() {}

        public static double SLOW_MODE_MULTIPLIER = 0.25;
        public static boolean FIELD_CENTRIC_DEFAULT = false;
    }

    /** Field / goal geometry, in inches. */
    @Configurable
    public static final class Field {
        private Field() {}

        public static double GOAL_X = 0.0;
        public static double GOAL_Y = 144.0;
        public static double GOAL_HEIGHT = 42.0;
    }

    /** Intake roller. Negative power = intaking. */
    @Configurable
    public static final class Intake {
        private Intake() {}

        public static double INTAKE_POWER = -0.9;
        public static double REVERSE_POWER = 0.9;
    }

    /** Transfer roller between intake and shooter. */
    @Configurable
    public static final class Transfer {
        private Transfer() {}

        public static double FORWARD_POWER = 0.5;
        public static double REVERSE_POWER = -0.5;
    }

    /** Velocity-PID flywheel (shooter wheel). */
    @Configurable
    public static final class Flywheel {
        private Flywheel() {}

        public static double KP = 0.03;
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static double KF = 0.00065;

        /** Clamp on PIDF output power, to protect hardware. */
        public static double MAX_POWER = 1.0;
        /** Velocity below this (ticks/s) counts as stopped. */
        public static double STOPPED_VELOCITY = 50.0;
    }

    /** Ballistic launcher model that feeds the flywheel. */
    @Configurable
    public static final class Launcher {
        private Launcher() {}

        public static double FIXED_LAUNCH_ANGLE_DEG = 65.0;
        public static double FLYWHEEL_RADIUS_IN = 1.9;
        /** Fudge factor for hood/friction losses. Tune experimentally. */
        public static double EFFICIENCY = 1.0;
        /** Gravity in in/s^2. */
        public static double GRAVITY_IN_PER_S2 = 386.09;
        /** Sentinel meaning "pose unknown, do not shoot". */
        public static double UNKNOWN_POSE = 1000.0;
        /** Never command more than this wheel speed (RPM). */
        public static double MAX_RPM = 6000.0;
    }

    /** Turret yaw. */
    @Configurable
    public static final class Turret {
        private Turret() {}

        public static double KP = 0.008;
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static double KF = 0.0;

        /** Degrees of turret rotation per encoder tick. */
        public static double DEGREES_PER_TICK = 0.167;
        /** Software end-stop, symmetric +/- degrees. */
        public static double MAX_ANGLE_DEG = 90.0;
        /** Clamp on PIDF output power. */
        public static double MAX_POWER = 1.0;
        /** Considered aimed when within this many degrees of target. */
        public static double AIM_TOLERANCE_DEG = 1.5;
    }

    /** Kicker / cap servo that pushes a ball into the flywheel. */
    @Configurable
    public static final class Kicker {
        private Kicker() {}

        public static double OPEN_POSITION = 1.0;
        public static double CLOSED_POSITION = 0.0;
    }

    /** Vision / AprilTag camera. */
    @Configurable
    public static final class Vision {
        private Vision() {}

        public static int CAMERA_WIDTH = 640;
        public static int CAMERA_HEIGHT = 480;
        public static boolean ENABLE_LIVE_VIEW = true;
    }
}
