package frc.team4276.frc2024.subsystems;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.LimelightConstants;

public class LimeLight extends Subsystem {
    private final NetworkTable mNetworkTable;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private int mListenerId = -1;

    private boolean mDisableProcessing = false;

    private static LimeLight mInstance;

    public static LimeLight getInstance(){
        if (mInstance == null){
            mInstance = new LimeLight();
        }
        return mInstance;
    }

    /**
     * Represents a Class to Store a Vision Update
     * Allows us to maintain the timestamp of a vision capture along with the corresponding 2d Translation from Camera to Goal
     */
    public static class VisionUpdate {
        private double timestamp;
        private Translation2d cameraToTarget;
        private int tagId;
        private Pose2d fieldToTag;

        public VisionUpdate(double timestamp, Translation2d cameraToTarget, int tagId) {
            this.timestamp = timestamp;
            this.cameraToTarget = cameraToTarget;
            //this.fieldToTag = mTagMap.get(tagId).getFieldToTag();
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Translation2d getCameraToTag()  {
            return cameraToTarget;
        }
        // public Pose2d getFieldToTag() {
        //     return fieldToTag;
        // }

        public int getTagId() {
            return tagId;
        }
    }

    public synchronized void setDisableProcessing(boolean disableLimelight) {
        mDisableProcessing = disableLimelight;
    }

    public synchronized boolean getIsDisabled() {
        return mDisableProcessing;
    }

    private LimeLight(){
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    private class PeriodicIO{
        // Inputs
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public boolean seesTarget;
        public int tagId;
        public double imageCaptureLatency;
        public double[] targetDistanceToRobot;

        // Outputs
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
    }

    private class Listener implements TableEventListener {
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if (key.equals("json")) {
                if (!mDisableProcessing) {
                    readInputsAndAddVisionUpdate();
                }
            }
        }
    }

    public void readInputsAndAddVisionUpdate() {
        final double timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.imageCaptureLatency = mNetworkTable.getEntry("cl").getDouble(LimelightConstants.kImageCaptureLatency);
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + mPeriodicIO.imageCaptureLatency / 1000.0
                + LimelightConstants.kLimelightTransmissionTimeLatency;
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("getpipe").getDouble(0);
        mPeriodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.tagId = (int) mNetworkTable.getEntry("tid").getNumber(-1).doubleValue();
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.targetDistanceToRobot = mNetworkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
        Translation2d cameraToTarget = new Translation2d(
            mPeriodicIO.targetDistanceToRobot[0], 
            mPeriodicIO.targetDistanceToRobot[1]);

        if (mPeriodicIO.seesTarget) {
             /*
             if (mTagMap.keySet().contains(tagId) && cameraToTarget != null) {
                 DriveSubsystem.getInstance().visionUpdate(
                        new VisionUpdate(timestamp - mPeriodicIO.latency, cameraToTarget, tagId));
            } else {
                DriveSubsystem.getInstance().visionUpdate(null);
            }
            */
            RobotState.getInstance().visionUpdate(
                new VisionUpdate(timestamp - mPeriodicIO.latency, cameraToTarget, mPeriodicIO.tagId));
        }


    }

    @Override
    public void readPeriodicInputs() {}

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putNumber("imageCaptureLatency", mPeriodicIO.imageCaptureLatency);
        SmartDashboard.putNumber("latency", mPeriodicIO.latency);

        SmartDashboard.putNumber("Limelight X", mPeriodicIO.targetDistanceToRobot[0]);
        SmartDashboard.putNumber("Limelight Y", mPeriodicIO.targetDistanceToRobot[1]);

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

    /**
     * Starts the Listener
     */
    public synchronized void start() {
        if (mListenerId < 0) {
            mListenerId = mNetworkTable.addListener("json", EnumSet.of(Kind.kValueAll), new Listener());
        }
    }


    @Override
    public void stop() { }

    




}
