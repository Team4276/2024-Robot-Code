package frc.team4276.frc2024.Limelight;

import java.util.HashMap;

import frc.team254.lib.geometry.Pose2d;

public class Field {
    public static class Red{
        public static final HashMap<Integer, Apriltag> kAprilTagMap = new HashMap<>();

        private static final Apriltag kAprilTag1 = new Apriltag(1, new Pose2d());

        static {
            kAprilTagMap.put(1, kAprilTag1);
        }
        
    }

    public static class Blue{
        public static final HashMap<Integer, Apriltag> kAprilTagMap = new HashMap<>();
    }
    
}