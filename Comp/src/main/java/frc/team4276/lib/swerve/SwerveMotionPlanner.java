package frc.team4276.lib.swerve;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4276.lib.path.DriveToTrajectoryState;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class SwerveMotionPlanner {
    private final DriveToTrajectoryState mDriveToTrajectoryState;
    private final AdaptiveTrajectoryTimeSampler mAdaptiveTrajectoryTimeSampler;

    private Trajectory<SwerveSample> mTrajectory;

    private boolean mIsFinished = true;

    private Translation2d mTranslationError = Translation2d.identity(); 
    private Rotation2d mRotationError = Rotation2d.identity();

    private Pose2d mTargetPose = Pose2d.identity();
    
    public SwerveMotionPlanner(){
        mDriveToTrajectoryState = new DriveToTrajectoryState(DriveConstants.kAutoTranslationPIDConstants,
                DriveConstants.kAutoRotationPIDConstants, Constants.kLooperDt, DriveConstants.kAutoTransAccelFF, DriveConstants.kAutoRotAccelFF);
        
        mAdaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(DriveConstants.kAutoMaxError);
    }

    public synchronized void setTrajectory(Trajectory<SwerveSample> traj, Pose2d currentPose, ChassisSpeeds currentSpeeds, double timestamp){
        mTrajectory = traj;
        mIsFinished = false;
        mDriveToTrajectoryState.reset();
        mAdaptiveTrajectoryTimeSampler.setStartTime(timestamp);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp) {
        return update(currentPose, timestamp, false);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp, boolean isVirtual) {
        if(mTrajectory == null) return ChassisSpeeds.identity();

        SwerveSample targetState = mAdaptiveTrajectoryTimeSampler.getTargetSwerveTrajectoryState(mTrajectory, currentPose.toWPI(), timestamp);

        if (mAdaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp) > mTrajectory.getTotalTime()) {
            mIsFinished = true;
        }

        if(targetState != null){
            mTargetPose = Pose2d.fromWPI(targetState.getPose());
        }

        mTranslationError = Translation2d.fromWPI(mDriveToTrajectoryState.getTranslationError());
        mRotationError = Rotation2d.fromWPI(mDriveToTrajectoryState.getRotationError());

        return ChassisSpeeds.fromWPI(mDriveToTrajectoryState.getTargetSpeeds(
            isVirtual ? targetState.getPose() : currentPose.toWPI(), targetState));
    }

    public synchronized boolean isFinished(){
        return mIsFinished;
    }

    public Translation2d getTranslationError(){
        return mTranslationError;
    }

    public Rotation2d getRotationError(){
        return mRotationError;
    }

    public Pose2d getTargetPose(){
        return mTargetPose;
    }
}