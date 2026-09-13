package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    /**
     * Alias kept for callers that reference {@code Constants.createFollower}.
     * Fill in {@link #create(HardwareMap)} with your robot's drivetrain,
     * localizer, and algorithm configs (copy them from the Pedro Pathing
     * Quickstart's {@code pedro} package:
     * https://github.com/Pedro-Pathing/Quickstart/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro).
     */
    public static Follower createFollower(HardwareMap h) {
        return create(h);
    }
}