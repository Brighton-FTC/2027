package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drivetrain-only test TeleOp.
 *
 * <p>Drive with gamepad1: left stick = drive/strafe, right stick X = turn,
 * RIGHT_BUMPER = slow-mode toggle, SQUARE = field/robot-centric toggle.
 */
@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTestTeleop extends GenericTeleop {

    @Override
    protected double getGoalX() {
        // Unused for drivetrain testing; turret/launcher aiming stays idle.
        return 0;
    }

    @Override
    protected Pose getStartingPose() {
        return Pose.zero();
    }
}
