package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drivetrain-only test TeleOp, runnable on the Driver Hub.
 *
 * <p>Extends {@link GenericTeleop} so drive control, slow mode, and
 * telemetry come from the shared implementation. No extra mechanism
 * inputs are used here — just drive with gamepad1:
 * left stick = drive/strafe, right stick X = turn,
 * RIGHT_BUMPER = slow-mode toggle.
 */
@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTestTeleop extends GenericTeleop {

    @Override
    protected double getObjectXPosition() {
        // Unused for drivetrain testing; turret/launcher aiming stays idle.
        return 0;
    }

    @Override
    protected Pose getStartingPose() {
        // Start at origin; localization is seeded from here in GenericTeleop.init().
        return Pose.zero();
    }
}
