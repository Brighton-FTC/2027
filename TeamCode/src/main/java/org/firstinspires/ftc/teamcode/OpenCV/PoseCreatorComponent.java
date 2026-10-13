package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class PoseCreatorComponent {

    private OpenCVComponent camera;
    private final double theta;

    private final double robotX;

    private final double robotY;

    private SamplePipeline.Detection best;

    public PoseCreatorComponent(String target, Pose robotPose){
        OpenCVComponent camera = new OpenCVComponent(hardwareMap, RobotConfig.Hardware.WEBCAM_NAME, telemetry);
        this.best = camera.getBestDetection();
        this.theta = robotPose.heading();
        this.robotX = robotPose.x();
        this.robotY = robotPose.y();
    }

    public double getX(){
        return robotX + best.forwardIn*Math.cos(theta) - best.lateralIn*Math.sin(theta);
    }

    public double getY(){
        return robotY + best.forwardIn*Math.sin(theta) - best.lateralIn*Math.cos(theta);
    }

    public double getHeading(){
        double dx = robotX - getX();
        double dy = robotY - getX();
        return Math.atan2(dx, dy);
    }

    public Pose createPose(){
        return new Pose(getX(), getY(), getHeading());
    }

}
