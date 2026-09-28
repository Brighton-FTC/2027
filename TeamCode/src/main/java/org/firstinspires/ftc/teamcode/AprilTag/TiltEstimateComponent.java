package org.firstinspires.ftc.teamcode.AprilTag;


public class TiltEstimateComponent {

    private double normalPitch;
    public TiltEstimateComponent(double normalPitch){
        this.normalPitch = normalPitch;
    }



    public  boolean isDown(double Degrees){
        return normalPitch > Degrees;
    }
}
