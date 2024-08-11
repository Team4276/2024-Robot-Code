package frc.team4276.lib.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4276.lib.path.DriveToTrajectoryState;

//TODO: add debugs for testing
public class MotionPlanner {
    private DriveToTrajectoryState mDriveToTrajectoryState;
    private AdaptiveTrajectoryTimeSampler mAdaptiveTrajectoryTimeSampler;

    private PathPlannerTrajectory mTrajectory;

    private boolean mIsFinished = true;
    
    public MotionPlanner(){
        mDriveToTrajectoryState = new DriveToTrajectoryState(DriveConstants.kAutoTranslationPIDConstants,
                DriveConstants.kAutoRotationPIDConstants, Constants.kLooperDt, DriveConstants.kMaxVel,
                DriveConstants.kTrackWidth * Math.sqrt(2), DriveConstants.kAutoAccelFF);
        
        mAdaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(DriveConstants.kAutoMaxError);
    }

    public synchronized void setTrajectory(PathPlannerPath path, Pose2d currentPose, ChassisSpeeds currentSpeeds, double timestamp){
        mTrajectory = new PathPlannerTrajectory(path, currentSpeeds.toWPI(), currentPose.getRotation().toWPI());
        mIsFinished = false;
        mDriveToTrajectoryState.reset(currentPose.toWPI(), currentSpeeds.toWPI());
        mAdaptiveTrajectoryTimeSampler.setStartTime(timestamp);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp) {
        if(mTrajectory == null) return ChassisSpeeds.identity();

        PathPlannerTrajectory.State state = mAdaptiveTrajectoryTimeSampler.getTargetTrajectoryState(mTrajectory, null, timestamp);

        if (mAdaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp) > mTrajectory.getTotalTimeSeconds()) {
            mIsFinished = true;
        }

        return ChassisSpeeds.fromWPI(mDriveToTrajectoryState.getTargetSpeeds(currentPose.toWPI(), state));
    }

    public synchronized boolean isFinished(){
        return mIsFinished;
    }
}
