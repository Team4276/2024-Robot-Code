package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final PathPlannerTrajectory mTrajectory;
    private final Command mCommand;

    public PPSwerveTrajectoryAction(PathPlannerTrajectory trajectory){
        mTrajectory = trajectory;

        mCommand = mDriveSubsystem.followPathCommand(trajectory);
    }

    @Override
    public void start() {
        if (mDriveSubsystem.readyForAuto()) {
            System.out.println("Starting trajectory! (length=" + mTrajectory.getTotalTimeSeconds() + " seconds)");
            mCommand.schedule();
        } else {
            System.out.println("Odometry reset failed!!! Not starting trajectory!!!");
        }

    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mCommand.isFinished()) {
            mDriveSubsystem.stop();
            return true;
        } 
        return false;
    }

    @Override
    public void done() {}
}
