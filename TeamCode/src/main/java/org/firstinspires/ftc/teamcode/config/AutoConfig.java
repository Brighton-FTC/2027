package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public final class AutoConfig {

    private AutoConfig(){}

    public static final class AutoPath{
        private AutoPath(){}


    }

    public static final class AutoPoses{
        private AutoPoses(){}

        public static final PoseFactory poseFactory = PoseFactory.radians();


        public static Pose flowerAudience =
                poseFactory.of(72, 12, Math.PI / 2);

        public static Pose flowerBlueAlliance =
                poseFactory.of(132, 72, Math.PI);

        public static Pose flowerScoring =
                poseFactory.of(72, 132, 3 * Math.PI / 2);

        public static Pose flowerRedAlliance =
                poseFactory.of(12, 72, 0);
        
        public static Pose redGardenPollen =
                poseFactory.of(12, 12, 5 * Math.PI / 4);

        public static Pose blueGardenPollen =
                poseFactory.of(132, 132, Math.PI / 4);

        public static Pose red1Start  = poseFactory.of(132, 24, Math.PI);
        public static Pose red2Start  = poseFactory.of(132, 48, Math.PI);

        public static Pose blue1Start = poseFactory.of(9, 96, 0);
        public static Pose blue2Start = poseFactory.of(9, 120, 0);


        // TODO: ADD IN THE SHOOTING POSES!!!!!!!!!!!!!!!!!!!!!!
        public static Pose BlueShoot;

        public static Pose RedShoot;
        
    }
}
