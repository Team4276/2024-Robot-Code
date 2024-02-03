package frc.team4276.frc2024.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.vision.TargetInfo;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.LimelightConstants;
import frc.team4276.frc2024.field.Apriltag;
import frc.team4276.frc2024.field.Field;

public class LimeLight extends Subsystem {
    private final NetworkTable mNetworkTable;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mListenerId = -1;

    private static HashMap<Integer, Apriltag> mTagMap = Field.Red.kAprilTagMap;

    private boolean mDisableProcessing = false;

    private static LimeLight mInstance;

    public static LimeLight getInstance() {
        if (mInstance == null) {
            mInstance = new LimeLight();
        }
        return mInstance;
    }

    private LimeLight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Represents a Class to Store a Vision Update
     * Allows us to maintain the timestamp of a vision capture along with the
     * corresponding 2d Translation from Camera to Goal
     */
    public static class VisionUpdate {
        private double timestamp;
        private Translation2d cameraToTarget;
        private int tagId;
        private Pose2d fieldToTag;

        public VisionUpdate(double timestamp, Translation2d cameraToTarget, int tagId) {
            this.timestamp = timestamp;
            this.cameraToTarget = cameraToTarget;
            this.fieldToTag = mTagMap.get(tagId).getTagInField();
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Translation2d getCameraToTag() {
            return cameraToTarget;
        }

        public Pose2d getTagInField() {
            return fieldToTag;
        }

        public int getTagId() {
            return tagId;
        }
    }

    public void setRedTagMap() {
        mTagMap = Field.Red.kAprilTagMap;
    }

    public void setBlueTagMap() {
        mTagMap = Field.Blue.kAprilTagMap;
    }

    public synchronized void setDisableProcessing(boolean disableLimelight) {
        mDisableProcessing = disableLimelight;
    }

    public synchronized boolean getIsDisabled() {
        return mDisableProcessing;
    }

    private class PeriodicIO {
        // Inputs
        public double latency;
        public boolean seesTarget;
        public int tagId;
        public double imageCaptureLatency;
        public double[] targetDistanceToRobot;
        public Number[] corners;
    }

    /**
     * Returns the Tag Id
     * @return
     */
    public int getTagId() {
        return mPeriodicIO.tagId;
    }

    private class Listener implements TableEventListener {
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                if (mDisableProcessing) {
                    readInputsAndAddVisionUpdate();
                }
            }
        }
    }

    public void readInputsAndAddVisionUpdate() {
        final double timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.imageCaptureLatency = mNetworkTable.getEntry("cl")
                .getDouble(LimelightConstants.kImageCaptureLatency);
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0
                + mPeriodicIO.imageCaptureLatency / 1000.0
                + LimelightConstants.kLimelightTransmissionTimeLatency;
        mPeriodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.tagId = (int) mNetworkTable.getEntry("tid").getNumber(-1).doubleValue();
        mPeriodicIO.targetDistanceToRobot = mNetworkTable.getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
        mPeriodicIO.corners = mNetworkTable.getEntry("tcornxy").getNumberArray(new Number[] { 0, 0, 0, 0, 0 });

        for (int i = 0; i < mPeriodicIO.corners.length; i++) {
            SmartDashboard.putNumber("Corner " + i, mPeriodicIO.corners[i].doubleValue());
        }

        if (mPeriodicIO.seesTarget) {
            if (mTagMap.keySet().contains(mPeriodicIO.tagId) && mPeriodicIO.targetDistanceToRobot != null) {
                RobotState.getInstance().visionUpdate(
                        new VisionUpdate(timestamp - mPeriodicIO.latency,
                                new Translation2d(
                                        mPeriodicIO.targetDistanceToRobot[0],
                                        mPeriodicIO.targetDistanceToRobot[1]),
                                mPeriodicIO.tagId));

                SmartDashboard.putNumber("Distance to camera X", mPeriodicIO.targetDistanceToRobot[0]);
                SmartDashboard.putNumber("Distance to camera Y", mPeriodicIO.targetDistanceToRobot[1]);
            } else {
                RobotState.getInstance().visionUpdate(null);
            }
        }

    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putNumber("imageCaptureLatency", mPeriodicIO.imageCaptureLatency);
        SmartDashboard.putNumber("latency", mPeriodicIO.latency);


    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                start();
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    // private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    // /**
    //  * Get the Normalized Corners
    //  * @return
    //  */
    // public List<TargetInfo> getTarget() {
    //     // Get corners
    //     List<Translation2d> corners = getCorners(mPeriodicIO.corners);

    //     if (corners.size() < 4 || !mTagMap.containsKey(mPeriodicIO.tagId)) {
    //         return null;
    //     }

    //     // Sort by y, list will have "highest in image" corner first
    //     corners.sort(ySort);
    //     ArrayList<TargetInfo> targetInfos = new ArrayList<>();

    //     for (Translation2d corner : corners) {
    //         targetInfos.add(getRawTargetInfo(new Translation2d(corner.x(), corner.y()), getTagId()));

    //     }


    //     return targetInfos;
    // }

    // /**
    //  * Returns Normalized Undistorted View Plane Coordinate
    //  * @param desiredTargetPixel Raw Pixel Value
    //  * @param tagId Tag ID
    //  * @return Normalized Target Info ready for Pinhole Calculations
    //  */
    // public synchronized TargetInfo getRawTargetInfo(Translation2d desiredTargetPixel, int tagId) {
    //     if (desiredTargetPixel == null) {
    //         return null;
    //     } else {
    //         double[] undistortedNormalizedPixelValues;
    //         UndistortMap undistortMap = Constants.kLimelightConstants.getUndistortMap();
    //         if (undistortMap == null) {
    //             try {
    //                 undistortedNormalizedPixelValues = undistortFromOpenCV(new double[]{desiredTargetPixel.x() / Constants.kResolutionWidth, desiredTargetPixel.y() / Constants.kResolutionHeight});
    //             } catch (Exception e) {
    //                 DriverStation.reportError("Undistorting Point Throwing Error!", false);
    //                 return null;
    //             }
    //         } else {
    //             undistortedNormalizedPixelValues = undistortMap.pixelToUndistortedNormalized((int) desiredTargetPixel.x(), (int) desiredTargetPixel.y());
    //         }

    //         double y_pixels = undistortedNormalizedPixelValues[0];
    //         double z_pixels = undistortedNormalizedPixelValues[1];


    //         //Negate OpenCV Undistorted Pixel Values to Match Robot Frame of Reference
    //         //OpenCV: Positive Downward and Right
    //         //Robot: Positive Upward and Left
    //         double nY = -(y_pixels - mCameraMatrix.get(0, 2)[0]);// -(y_pixels * 2.0 - 1.0);
    //         double nZ = -(z_pixels - mCameraMatrix.get(1, 2)[0]);// -(z_pixels * 2.0 - 1.0);

    //         double y = nY / mCameraMatrix.get(0, 0)[0];
    //         double z = nZ / mCameraMatrix.get(1, 1)[0];

    //         return new TargetInfo(y, z, tagId);
    //     }
    // }

    // /**
    //  * Stores each Corner received by LL as a Translation2d for further processing
    //  * @param tcornxy array from LL with Pixel Coordinate
    //  * @return List of Corners
    //  */
    // private static List<Translation2d> getCorners(Number[] tcornxy) {
    //     // Check if there is a non even number of corners
    //     if (tcornxy.length % 2 != 0) {
    //         return List.of();
    //     }

    //     ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
    //     for (int i = 0; i < tcornxy.length; i += 2) {
    //         corners.add(new Translation2d(tcornxy[i].doubleValue(), tcornxy[i + 1].doubleValue()));
    //     }

    //     return corners;
    // }

    /**
     * Starts the Listener
     */
    public synchronized void start() {
        if (mListenerId < 0) {
            mListenerId = mNetworkTable.addListener("json", EnumSet.of(Kind.kValueAll), new Listener());
        }
    }

    @Override
    public void stop() {
    }

}
