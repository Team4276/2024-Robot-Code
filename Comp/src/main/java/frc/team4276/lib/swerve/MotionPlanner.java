package frc.team4276.lib.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4276.lib.path.DriveToTrajectoryState;

import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class MotionPlanner {
    private DriveToTrajectoryState mDriveToTrajectoryState;
    private AdaptiveTrajectoryTimeSampler mAdaptiveTrajectoryTimeSampler;

    private PathPlannerTrajectory mTrajectory;

    private boolean mIsFinished = true;

    private Translation2d mTranslationError = Translation2d.identity(); 
    private Rotation2d mRotationError = Rotation2d.identity();
    
    public MotionPlanner(){
        mDriveToTrajectoryState = new DriveToTrajectoryState(DriveConstants.kAutoTranslationPIDConstants,
                DriveConstants.kAutoRotationPIDConstants, Constants.kLooperDt, DriveConstants.kMaxVel,
                DriveConstants.kTrackWidth * Math.sqrt(2), DriveConstants.kAutoAccelFF);

        mDriveToTrajectoryState = new DriveToTrajectoryState(DriveConstants.kAutoTranslationPIDConstants, 
            DriveConstants.kAutoRotationPIDConstants, Constants.kLooperDt, 0, 0, DriveConstants.kAutoAccelFF);
        
        mAdaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(DriveConstants.kAutoMaxError);
    }

    public synchronized void setTrajectory(PathPlannerPath path, Pose2d currentPose, ChassisSpeeds currentSpeeds, double timestamp){
        // mTrajectory = new PathPlannerTrajectory(path, currentSpeeds.toWPI(), currentPose.getRotation().toWPI());
        mTrajectory = new PathPlannerTrajectory(path, null, null, null);
        mIsFinished = false;
        mDriveToTrajectoryState.reset(currentPose.toWPI(), currentSpeeds.toWPI());
        mAdaptiveTrajectoryTimeSampler.setStartTime(timestamp);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp) {
        if(mTrajectory == null) return ChassisSpeeds.identity();

        PathPlannerTrajectoryState targetState = mAdaptiveTrajectoryTimeSampler.getTargetTrajectoryState(mTrajectory, currentPose.toWPI(), timestamp);

        if (mAdaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp) > mTrajectory.getTotalTimeSeconds()) {
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
