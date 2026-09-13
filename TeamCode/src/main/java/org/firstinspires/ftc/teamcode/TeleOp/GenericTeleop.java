package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.teamcode.ServoKick.ServoKickComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


/*  FOLLOWING THE PRINCIPLES OF DRY (DON'T REPEAT YOURSELF),
    Code is modified to have an abstract generic teleop class.
    Any instance-specific variables MUST go into abstract functions
 */

@Configurable
@TeleOp
public abstract class GenericTeleop extends OpMode {
    public Follower follower;
    private boolean shooting = false;

    private boolean driveFieldCentric = false;

    private boolean intaking = false;

    private boolean transfering = false;

    private boolean aim = false;

    private boolean opened = false;


    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    public final Pose startingPose = getStartingPose();
    private boolean automatedDrive = false;
    private TurretPIDComponent turret;

    private ServoKickComponent cap;

    private FlyWheelMotorComponent transfer;
    private DynamicAngleComponent launcher;
//    private FlyWheelMotorComponent launcher;

    private IntakeMotorComponent intake;
    private TelemetryManager telemetryManager;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.25;

    protected abstract double getObjectXPosition();

    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//
        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, getObjectXPosition(), 144, telemetry);
        launcher = new DynamicAngleComponent(hardwareMap, "servo", getObjectXPosition(), 144, 42,1.9, 1, startingPose, telemetry);
//        launcher = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");
        transfer = new FlyWheelMotorComponent(hardwareMap, "transferMotor");

        cap = new ServoKickComponent(hardwareMap, "launchCap");
//
        intake = new IntakeMotorComponent(hardwareMap, "intakeMotor");

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        turret.resetTurretEncoder();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
        telemetryManager.update();

        if (!automatedDrive) {

            if(driveFieldCentric) {
                if (!slowMode) follower.setTeleOpDrive(
                        gamepadEx1.getLeftY(),
                        -gamepadEx1.getLeftX(),
                        -gamepadEx1.getRightX() * slowModeMultiplier,
                        false
                );

                else follower.setTeleOpDrive(
                        gamepadEx1.getLeftY(),
                        -gamepadEx1.getLeftX(),
                        -gamepadEx1.getRightX() * slowModeMultiplier,
                        false
                );
            }
            else {
                if (!slowMode) follower.setTeleOpDrive(
                        gamepadEx1.getLeftY(),
                        -gamepadEx1.getLeftX(),
                        -gamepadEx1.getRightX(),
                        true
                );

                else follower.setTeleOpDrive(
                        gamepadEx1.getLeftY() * slowModeMultiplier,

                        -gamepadEx1.getLeftX() * slowModeMultiplier,
                        -gamepadEx1.getRightX() * slowModeMultiplier,
                        true
                );
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)&&!aim) {
                aim = true;
            }else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)&&aim){
                aim = false;
            }
            if (aim){
                turret.aimToObject(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slowMode = !slowMode;
            }
//            if(gamepadEx1.wasJustPressed(PSButtons.SQUARE)){
//                driveFieldCentric = !driveFieldCentric;
//            }
//
            if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE) && !shooting) {
//                launcher.runMotorAt(1);
                shooting = true;
            }
            else if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE)&& shooting){
//                launcher.stopMotor();
                launcher.stop();
                shooting = !shooting;
            }
            if (shooting){
                launcher.dynamicMotorPower(follower.getPose().getX(), follower.getPose().getY());
            }
//
//
//            if (gamepadEx1.wasJustPressed(PSButtons.CROSS)&&!intaking){
//                intake.startMotor();
//                transfer.runMotorAt(1);
//                intaking = !intaking;
//            }
//            else if (gamepadEx1.wasJustPressed(PSButtons.CROSS)&&intaking){
//                intake.stopMotor();
//                transfer.stopMotor();
//                intaking = !intaking;
//            }

            if (gamepadEx2.wasJustPressed(PSButtons.TRIANGLE)&&!intaking){
                intake.reverseMotor();
                transfer.runMotorAt(-0.5);
                intaking = !intaking;
            }
            else if(gamepadEx2.wasJustPressed(PSButtons.TRIANGLE)&&intaking){
                intake.stopMotor();
                transfer.stopMotor();
                intaking = !intaking;
            }

            if(gamepadEx2.wasJustPressed(PSButtons.CROSS)&&!transfering){
                transfer.runMotorAt(0.5);
                transfering = !transfering;
            }else if(gamepadEx2.wasJustPressed(PSButtons.CROSS)&&transfering){
                transfer.stopMotor();
                transfering = !transfering;
            }
            if(gamepadEx2.wasJustPressed(PSButtons.CIRCLE)&&!intaking){
                intake.startMotor();
                intaking = !intaking;
            }else if(intaking&&gamepadEx2.wasJustPressed(PSButtons.CIRCLE)){
                intaking = !intaking;
                intake.stopMotor();
            }


            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                cap.open();
            }
            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                cap.close();
            }
            if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)&&!intaking){
                intake.startMotor();
                transfer.runMotorAt(0.5);
                cap.open();
                intaking = !intaking;
            }else if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)&&intaking){
                intake.stopMotor();
                transfer.stopMotor();
                cap.close();
                intaking = !intaking;
            }
//
        }

        telemetry.addData("rpm", launcher.getRPM());
        telemetryManager.debug("position", follower.getPose());
        telemetryManager.debug("velocity", follower.getVelocity());
        telemetryManager.debug("automatedDrive", automatedDrive);
    }
}