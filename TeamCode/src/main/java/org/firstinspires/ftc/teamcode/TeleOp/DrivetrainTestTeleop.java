package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig.Vision.HiveCell;

/**
 * Drivetrain-only test TeleOp.
 *
 * <p>Drive with gamepad1 (see {@code RobotControls} for bindings):
 * left stick = drive/strafe, right stick X = turn.
 * Inherits the full control scheme from {@link GenericTeleop}; shooter and
 * feed mechanisms stay idle until their operator buttons are used.
 */
@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTestTeleop extends GenericTeleop {

    @Override
    protected double getGoalX() {
        // Unused for drivetrain testing; turret/launcher aiming stays idle.
        return RobotConfig.Field.RED_GOAL_X;
    }

    @Override
    protected double getGoalY() {
        return RobotConfig.Field.RED_GOAL_Y;
    }

    @Override
    protected double getGoalHeight() {
        return RobotConfig.Field.GOAL_HEIGHT;
    }

    @Override
    protected HiveCell getTargetCell() {
        return RobotConfig.Vision.RED_TARGET_CELL;
    }

    @Override
    protected Pose getStartingPose() {
        return Pose.zero();
    }
}
