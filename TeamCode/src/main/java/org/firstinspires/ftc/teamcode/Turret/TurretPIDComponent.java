package org.firstinspires.ftc.teamcode.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AprilTag.AprilTagLocalization;


@Config
@Configurable
public class TurretPIDComponent {

    private Follower follower;
    public static double kP = 0.008;

    public static double n = 360;
    public static double kI = 0.1;
    public static double kD = 0.0;

    public static double kF = 0.0;

    private double lastTurretAngle = 0;

    private double turretGlobalAngle = 0;

    private double scalingFactor;

    private double turretAngle;

    private double objectXPosition;

    private double objectYPosition;
    private Motor turretMotor;
    private final MultipleTelemetry telemetry;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private final PIDFController controller = new PIDFController(0, 0, 0, 0);

    public TurretPIDComponent(HardwareMap hardwareMap, String motorID, double scalingFactor, double objectXPosition, double objectYPosition, Telemetry telemetry) {
        turretMotor = new Motor(hardwareMap, motorID);
        turretMotor.stopAndResetEncoder();
        //remove if
        turretMotor.setDistancePerPulse(4*scalingFactor); // 360/537.7 = 4*0.167
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setRunMode(Motor.RunMode.RawPower);

//        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.scalingFactor = scalingFactor;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();

    }

    public void resetTurretEncoder(){
        turretMotor.stopAndResetEncoder();
    }

    public double encoderTicksToAngle(int ticks) {
        return (ticks * scalingFactor);
        //return (((double) ticks /4)*scalingFactor);
    }

    //Gear Ratio is 0.25
    //Turret degree per motor revolution is 360*0.25 = 90
    //PPR for GoBuilda Yellow Jacket = 537.7
    public int angleToEncoderTicks(double degrees) {
        return (int) (degrees / scalingFactor);
        //return (int) ((degrees*4)/scalingFactor);
    }

    public void turnTurretBy(double degrees) {
        controller.setPIDF(kP, kI, kD, kF);

        double currentPosition = turretMotor.getCurrentPosition();
        double TARGET_TICK_VALUE = angleToEncoderTicks(degrees) + currentPosition;
        controller.setSetPoint(TARGET_TICK_VALUE);
        double power = controller.calculate(currentPosition);

        telemetry.addData("Motor Power: ", power);
        telemetry.update();

        turretMotor.set(power);
    }



    public void aimToObject(double robotX, double robotY, double robotHeading) {
        if (robotX != 1000 && robotY != 1000) {
            double robotAngle = Math.toDegrees(robotHeading);
            double destinationAngle = Math.toDegrees(Math.atan2(objectYPosition - robotY,
                    objectXPosition - robotX));



            turretAngle = encoderTicksToAngle(turretMotor.getCurrentPosition());

            double delta = turretAngle - lastTurretAngle;

            turretGlobalAngle += delta;
            lastTurretAngle = turretAngle;

            double toTurn = destinationAngle - (turretAngle + robotAngle);

            if (Math.abs(turretGlobalAngle+toTurn) >= 180 && toTurn > 0){
                toTurn-=360;
            }else if (Math.abs(turretGlobalAngle+toTurn) >= 180 && toTurn < 0){
                toTurn+=360;
            }

            telemetry.addData("To turn :", toTurn);
            telemetry.addData("error", encoderTicksToAngle((int) controller.getPositionError()));
            telemetry.addData("destination", destinationAngle);
            telemetry.addData("current angle", turretAngle);
            telemetry.update();


            turnTurretBy(toTurn);
        }
    }

    public double getPIDSetPoint(){
        return controller.getSetPoint();
    }

}