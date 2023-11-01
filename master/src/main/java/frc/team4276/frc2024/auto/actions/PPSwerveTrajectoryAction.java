package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Trajectory mTrajectory;
    private final Rotation2d mHeading;

    public PPSwerveTrajectoryAction(PathPlannerTrajectory trajectory, Rotation2d heading){
        mTrajectory = trajectory;
        mHeading = heading;
    }

    @Override
    public void start() { //TODO: check if position is reset
        if (true) {
            System.out.println("Starting trajectory! (length=" + mTrajectory.getTotalTimeSeconds() + " seconds)");
            mDriveSubsystem.followPathCommandEvents(null);
        } else {
            System.out.println("Odometry reset failed!!! Not starting trajectory!!!");
        }

    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDriveSubsystem.getCurrentCommand().isFinished()) {
            mDriveSubsystem.stop();
            return true;
        } 
        return false;
    }

    @Override
    public void done() {}
}
