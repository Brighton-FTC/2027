package org.firstinspires.ftc.teamcode.Turret;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Servo.ServoComponent;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.Objects;

/**
 * Turret yaw controlled by a PIDF loop on encoder ticks.
 *
 * <p>Angles are in degrees, relative to the robot forward (0 = straight
 * ahead, + = left). Commands are software-clamped to
 * {@code +/- MAX_ANGLE_DEG} so the turret can never wrap cables.
 *
 * <p>Gains and limits live in {@link RobotConfig.Turret} and are re-applied
 * on every update, so dashboard edits take effect immediately.
 *
 * <p>This class only writes telemetry keys — it never calls
 * {@code telemetry.update()}; the OpMode owns the update loop.
 */
@Configurable
public class TurretPIDComponent {

    private final ServoComponent turretServo;
    private final Telemetry telemetry;

    private final double goalX;
    private final double goalY;
    private final double degreesPerTick; // 1/1800 Servo pos : [0, 1]

    /** Uses hardware/field constants from {@link RobotConfig}. */
    public TurretPIDComponent(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap,
                RobotConfig.Hardware.TURRET_MOTOR,
                RobotConfig.Turret.DEGREES_PER_TICK,
                RobotConfig.Field.GOAL_X,
                RobotConfig.Field.GOAL_Y,
                telemetry);
    }

    public TurretPIDComponent(HardwareMap hardwareMap, String servoID,
                              double degreesPerTick,
                              double goalX, double goalY,
                              Telemetry telemetry) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(servoID, "servoID");
        Objects.requireNonNull(telemetry, "telemetry");
        if (!(degreesPerTick > 0)) {
            throw new IllegalArgumentException("degreesPerTick must be > 0");
        }
        this.turretServo = new ServoComponent(hardwareMap, servoID);
        this.degreesPerTick = degreesPerTick;
        this.goalX = goalX;
        this.goalY = goalY;
        this.telemetry = telemetry;
    }


    /** Current turret angle in degrees, relative to robot forward. */
    public double getCurrentAngle() {
        return turretServo.getPos();
    }


    // GoBuilda Speed Servo 1800 degrees is 0.0005556 ticks per degree 1/1800.
    public double encoderTicksToAngle(double ticks) {
        return ticks * degreesPerTick;
    }


    // GoBuilda Speed Servo 1800 degrees is 0.0005556 ticks per degree
    public double angleToEncoderTicks(double degrees) {
        return degrees / degreesPerTick;
    }

    /**
     * Rotates the turret by a relative offset in degrees.
     * The resulting absolute target is clamped to the software end-stops.
     */
    public void turnTurretBy(double deltaDegrees) {
        double targetAngle = clampAngle(getCurrentAngle() + deltaDegrees);
        driveToAngle(targetAngle);
    }

    /**
     * Aims at the goal from the given robot pose (inches + heading radians,
     * field frame). If the goal is behind the robot the turret holds at the
     * nearest end-stop instead of spinning around.
     */
    public void aimToObject(double robotX, double robotY, double robotHeadingRad) {
        if (isUnknownPose(robotX, robotY)) {
            return;
        }
        double robotDeg = Math.toDegrees(robotHeadingRad);
        double goalBearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        double relative = normalize180(goalBearing - robotDeg);
        double clampedTarget = clampAngle(relative);
        double toTurn = clampedTarget - getCurrentAngle();

        telemetry.addData("Turret/toTurnDeg", toTurn);
        telemetry.addData("Turret/relativeDeg", relative);
        telemetry.addData("Turret/targetDeg", clampedTarget);
        telemetry.addData("Turret/currentDeg", getCurrentAngle());
        turnTurretBy(toTurn);
    }

    /** True when the turret is within tolerance of the goal bearing. */
    public boolean isAimed(double robotX, double robotY, double robotHeadingRad) {
        if (isUnknownPose(robotX, robotY)) {
            return false;
        }
        double robotDeg = Math.toDegrees(robotHeadingRad);
        double goalBearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        double relative = normalize180(goalBearing - robotDeg);
        double clampedTarget = clampAngle(relative);
        return Math.abs(clampedTarget - getCurrentAngle()) <= RobotConfig.Turret.AIM_TOLERANCE_DEG;
    }

    public void reset(){
        turretServo.setDefaultPos();}

    // ---- internals ----

    private void driveToAngle(double targetAngleDeg) {
        double targetTicks = angleToEncoderTicks(targetAngleDeg);
        turretServo.setPos(targetTicks);
    }

    private static double clampAngle(double angleDeg) {
        double limit = Math.abs(RobotConfig.Turret.MAX_ANGLE_DEG);
        return Math.max(-limit, Math.min(limit, angleDeg));
    }

    private static double clampPower(double power) {
        double limit = Math.abs(RobotConfig.Turret.MAX_POWER);
        return Math.max(-limit, Math.min(limit, power));
    }

    private static double normalize180(double angleDeg) {
        while (angleDeg > 180) {
            angleDeg -= 360;
        }
        while (angleDeg <= -180) {
            angleDeg += 360;
        }
        return angleDeg;
    }



    private static boolean isUnknownPose(double x, double y) {
        double sentinel = RobotConfig.Launcher.UNKNOWN_POSE;
        return x == sentinel || y == sentinel;
    }
}
