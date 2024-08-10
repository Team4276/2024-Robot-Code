package frc.team4276.frc2024.field;

import java.util.HashMap;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class Field {
    private static final Apriltag kAprilTag1 = new Apriltag(1, new Pose2d(15.079472, 0.245872, new Rotation2d(120)),
            1.355852);
    private static final Apriltag kAprilTag2 = new Apriltag(2, new Pose2d(16.1852, 0.883668, new Rotation2d(120)),
            1.355852);
    private static final Apriltag kAprilTag3 = new Apriltag(3, new Pose2d(16.5794, 4.98273, new Rotation2d(180)),
            1.451102);
    private static final Apriltag kAprilTag4 = new Apriltag(4, new Pose2d(16.5794, 5.54788, new Rotation2d(180)),
            1.451102);
    private static final Apriltag kAprilTag5 = new Apriltag(5, new Pose2d(14.7008, 8.20422, new Rotation2d(270)),
            1.355852);
    private static final Apriltag kAprilTag6 = new Apriltag(6, new Pose2d(1.8415, 8.20422, new Rotation2d(270)),
            1.355852);
    private static final Apriltag kAprilTag7 = new Apriltag(7, new Pose2d(-0.0381001, 5.54788, new Rotation2d(0)),
            1.451102);
    private static final Apriltag kAprilTag8 = new Apriltag(8, new Pose2d(-0.0381001, 4.98273, new Rotation2d(0)),
            1.451102);
    private static final Apriltag kAprilTag9 = new Apriltag(9, new Pose2d(0.356109, 0.883668, new Rotation2d(60)),
            1.355852);
    private static final Apriltag kAprilTag10 = new Apriltag(10, new Pose2d(1.46152, 0.245872, new Rotation2d(60)),
            1.355852);
    private static final Apriltag kAprilTag11 = new Apriltag(11, new Pose2d(11.9047, 3.71323, new Rotation2d(300)),
            1.3208);
    private static final Apriltag kAprilTag12 = new Apriltag(12, new Pose2d(11.9047, 4.49835, new Rotation2d(60)),
            1.3208);
    private static final Apriltag kAprilTag13 = new Apriltag(13, new Pose2d(11.2202, 4.10516, new Rotation2d(180)),
            1.3208);
    private static final Apriltag kAprilTag14 = new Apriltag(14, new Pose2d(5.3208, 4.10516, new Rotation2d(0)),
            1.3208);
    private static final Apriltag kAprilTag15 = new Apriltag(15, new Pose2d(4.64135, 4.49835, new Rotation2d(120)),
            1.3208);
    private static final Apriltag kAprilTag16 = new Apriltag(16, new Pose2d(4.64135, 3.71323, new Rotation2d(240)),
            1.3208);

    public static final HashMap<Integer, Apriltag> kAprilTagMap = new HashMap<>();

    static {
        kAprilTagMap.put(1, kAprilTag1);
        kAprilTagMap.put(2, kAprilTag2);
        kAprilTagMap.put(3, kAprilTag3);
        kAprilTagMap.put(4, kAprilTag4);
        kAprilTagMap.put(5, kAprilTag5);
        kAprilTagMap.put(6, kAprilTag6);
        kAprilTagMap.put(7, kAprilTag7);
        kAprilTagMap.put(8, kAprilTag8);
        kAprilTagMap.put(9, kAprilTag9);
        kAprilTagMap.put(10, kAprilTag10);
        kAprilTagMap.put(11, kAprilTag11);
        kAprilTagMap.put(12, kAprilTag12);
        kAprilTagMap.put(13, kAprilTag13);
        kAprilTagMap.put(14, kAprilTag14);
        kAprilTagMap.put(15, kAprilTag15);
        kAprilTagMap.put(16, kAprilTag16);
    }

    public static class POIs {
        public Translation2d kSpeakerCenter;
        public Translation2d kSpeakerSS;
        public Translation2d kSpeakerAS;
        public Translation2d kAmp;
        public Translation2d kStageMid;
        public Translation2d kStageSS;
        public Translation2d kStageAS;
        public Translation2d kBank;
    }

    public static class Blue {
        public static final HashMap<Integer, Apriltag> kAprilTagMap = new HashMap<>();

        static {
            kAprilTagMap.put(1, kAprilTag1);
            kAprilTagMap.put(2, kAprilTag2);
            kAprilTagMap.put(3, kAprilTag3);
            kAprilTagMap.put(4, kAprilTag4);
            kAprilTagMap.put(5, kAprilTag5);
            kAprilTagMap.put(6, kAprilTag6);
            kAprilTagMap.put(7, kAprilTag7);
            kAprilTagMap.put(8, kAprilTag8);
            kAprilTagMap.put(9, kAprilTag9);
            kAprilTagMap.put(10, kAprilTag10);
            kAprilTagMap.put(11, kAprilTag11);
            kAprilTagMap.put(12, kAprilTag12);
            kAprilTagMap.put(13, kAprilTag13);
            kAprilTagMap.put(14, kAprilTag14);
            kAprilTagMap.put(15, kAprilTag15);
            kAprilTagMap.put(16, kAprilTag16);
        }

        public static final POIs kPOIs = new POIs();

        static {
            kPOIs.kSpeakerCenter = new Translation2d(0.225, 5.55);
            kPOIs.kSpeakerSS = new Translation2d();
            kPOIs.kSpeakerAS = new Translation2d();
            kPOIs.kAmp = new Translation2d();
            kPOIs.kStageMid = new Translation2d();
            kPOIs.kStageSS = new Translation2d();
            kPOIs.kStageAS = new Translation2d();
            kPOIs.kBank = new Translation2d();
        }

    }

    public static class Red {
        public static final HashMap<Integer, Apriltag> kAprilTagMap = new HashMap<>();

        static {
            kAprilTagMap.put(1, kAprilTag1);
            kAprilTagMap.put(2, kAprilTag2);
            kAprilTagMap.put(3, kAprilTag3);
            kAprilTagMap.put(4, kAprilTag4);
            kAprilTagMap.put(5, kAprilTag5);
            kAprilTagMap.put(6, kAprilTag6);
            kAprilTagMap.put(7, kAprilTag7);
            kAprilTagMap.put(8, kAprilTag8);
            kAprilTagMap.put(9, kAprilTag9);
            kAprilTagMap.put(10, kAprilTag10);
            kAprilTagMap.put(11, kAprilTag11);
            kAprilTagMap.put(12, kAprilTag12);
            kAprilTagMap.put(13, kAprilTag13);
            kAprilTagMap.put(14, kAprilTag14);
            kAprilTagMap.put(15, kAprilTag15);
            kAprilTagMap.put(16, kAprilTag16);
        }
        public static final POIs kPOIs = new POIs();

        static {
            kPOIs.kSpeakerCenter = new Translation2d(16.317, 5.55);
            kPOIs.kSpeakerSS = new Translation2d();
            kPOIs.kSpeakerAS = new Translation2d();
            kPOIs.kAmp = new Translation2d();
            kPOIs.kStageMid = new Translation2d();
            kPOIs.kStageSS = new Translation2d();
            kPOIs.kStageAS = new Translation2d();
            kPOIs.kBank = new Translation2d();
        }
    }

}
