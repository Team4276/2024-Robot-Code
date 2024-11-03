package frc.team4276.lib.swerve;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4276.lib.path.DriveToTrajectoryState;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class MotionPlanner {
    private final DriveToTrajectoryState mDriveToTrajectoryState;
    private final AdaptiveTrajectoryTimeSampler mAdaptiveTrajectoryTimeSampler;

    private Trajectory<?> mTrajectory;

    private boolean mIsFinished = true;

    private Translation2d mTranslationError = Translation2d.identity(); 
    private Rotation2d mRotationError = Rotation2d.identity();
    
    public MotionPlanner(){
        mDriveToTrajectoryState = new DriveToTrajectoryState(DriveConstants.kAutoTranslationPIDConstants,
                DriveConstants.kAutoRotationPIDConstants, Constants.kLooperDt, DriveConstants.kAutoTransAccelFF, DriveConstants.kAutoRotAccelFF);
        
        mAdaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(DriveConstants.kAutoMaxError);
    }

    public synchronized void setTrajectory(Trajectory<?> traj, Pose2d currentPose, ChassisSpeeds currentSpeeds, double timestamp){
        mTrajectory = traj;
        mIsFinished = false;
        mDriveToTrajectoryState.reset();
        mAdaptiveTrajectoryTimeSampler.setStartTime(timestamp);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp) {
        if(mTrajectory == null) return ChassisSpeeds.identity();

        TrajectorySample<?> targetState = mAdaptiveTrajectoryTimeSampler.getTargetTrajectoryState(mTrajectory, currentPose.toWPI(), timestamp);

        if (mAdaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp) > mTrajectory.getTotalTime()) {
            mIsFinished = true;
        }

        mTranslationError = Translation2d.fromWPI(mDriveToTrajectoryState.getTranslationError());
        mRotationError = Rotation2d.fromWPI(mDriveToTrajectoryState.getRotationError());

        return ChassisSpeeds.fromWPI(mDriveToTrajectoryState.getTargetSpeeds(currentPose.toWPI(), targetState));
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
}