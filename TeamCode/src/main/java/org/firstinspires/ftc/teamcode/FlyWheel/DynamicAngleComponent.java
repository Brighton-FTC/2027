package org.firstinspires.ftc.teamcode.FlyWheel;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.Math;


@Config
@Configurable
public class DynamicAngleComponent {


    private double objectXPosition;

    private double rpm;

    private double objectYPosition;

    private double objectHeight;

    private double flyWheelRadius;

    public static double efficiency;

    //private FlyWheelMotorComponent flyWheel;
    private FlyWheelMotorPIDComponent flyWheel;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private Follower follower;

    public DynamicAngleComponent(HardwareMap hardwareMap, String servoID, double objectXPosition, double objectYPosition, double objectHeight, double flyWheelRadius, double efficiency, Pose startingPose, Telemetry telemetry) {
//        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        //flyWheel = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");
        flyWheel = new FlyWheelMotorPIDComponent(hardwareMap, "flyWheelMotor");
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.objectHeight = objectHeight;
        this.flyWheelRadius = flyWheelRadius;
        this.efficiency = efficiency;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
    }

    //Gear ratio for servo gear vs launcher gear
    //setPos (Gr*degrees/ppd)/180

    public void dynamicMotorPower(double robotX, double robotY) {

        /*
        Assuming turret faces the goal at all time, therefore 3D kinematics may be neglected.
        2 Modes of launching artifacts - adjusting speed or adjusting angle (adjusting speed comes in priority)

        Launch mode is switched to dynamic angle if / when it is unreachable at 65 degrees


         */
//        double robotYPosition = camera.returnYPosition();
//        double robotXPosition = camera.returnXPosition();

        if (robotX != 1000 && robotY != 1000){

            //Efficiency of the hood must not be neglected.
            double fixV = (2.0 * Math.PI * flyWheelRadius / 65.0) * 6000  * efficiency;


            double distance = Math.sqrt(Math.pow(objectXPosition - robotX, 2) + Math.pow(objectYPosition - robotY, 2));

            //We let y = objectHeight and x = distance from robot
            double denom = distance * Math.tan(Math.toRadians(65)) - objectHeight;

            double v;

            if(denom<=0) {
                //Linear velocity required for artifact to pass through x = distance from robot and y = object height
                v = 0;
            }
            else {
                v = Math.sqrt((386.09 * Math.pow(distance, 2)) / (2.0 * Math.cos(Math.toRadians(65)) * Math.cos(Math.toRadians(65)) * denom));
            }

//            double launchEnergy = 0.5*0.00512835678 *Math.pow(v, 2);
//            double requiredEnergy = launchEnergy / efficiency; //we tune efficiency
//            double inertia = 0.5*0.0056187848* Math.pow(3.78, 2); //disk inertia calculated by 1/2 mr^2

//            //Denominator less than or equal 0 will yield undefined / imaginary solution. Meaning no velocity will allow artifact to reach target at 65 degrees.
//            if (denom <= 0) {
//                double inside = Math.pow(fixV, 4) - 386.09 * (386.09 * Math.pow(distance, 2) + 2 * objectHeight * Math.pow(fixV, 2));
//
//                //Solving the quadratic yields 2 roots of trajectory in different shapes.
//                double destinationAngleFlat = Math.atan((Math.pow(fixV, 2) - Math.sqrt(inside)) / (386.09 * distance));
//                double destinationAngleArc = Math.atan((Math.pow(fixV, 2) + Math.sqrt(inside)) / (386.09 * distance));
//
//                double chosen = Math.min(destinationAngleFlat, destinationAngleArc);
//                turnServoTo(Math.toDegrees(chosen) % 360);
//            } else {
//
//                //This ensures launch angle is reset to 65 when the launcher is running in dynamic velocity mode.
//                resetServo();
//
//                //just tune the efficiency. experimental measurement of efficiency is too much hassle.
//                double v_real = v/efficiency;
//
//                //Linear velocity is converted to angular velocity.
//                double rpm = (65.0 / (2.0 * Math.PI * flyWheelRadius)) * v_real;
//
//
//                double motorPower = rpm / 6000;
//
//                flyWheel.runMotorAt(motorPower);
//            }
            //just tune the efficiency. experimental measurement of efficiency is too much hassle.
//            double v_real = Math.sqrt((2*requiredEnergy)/inertia); //angular velocity calculated from rotational energy 1/2 IΩ (omega)
            double v_real = v/efficiency;
            //Linear velocity is converted to angular velocity.
            rpm = (60.0 / (2.0 * Math.PI*flyWheelRadius)) * v_real;



            flyWheel.runMotorAt(rpm);
        }

    }

    public double getRPM(){
        return rpm;
    }

    public void stop(){
        flyWheel.stopMotor();
    }

}
