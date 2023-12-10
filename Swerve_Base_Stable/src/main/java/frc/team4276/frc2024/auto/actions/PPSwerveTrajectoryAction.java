package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    private boolean hasRun = false;

    public PPSwerveTrajectoryAction(PathPlannerTrajectory trajectory){
        mCommand = mDriveSubsystem.followPathCommand(trajectory);

        hasRun = false;
    }

    @Override
    public void start() {
        System.out.println("Start");
        mCommand.initialize();

    }

    @Override
    public void update() {
        System.out.println("Update");
        mCommand.execute();
    }

    @Override
    public boolean isFinished() {
        if (mCommand.isFinished() && hasRun) {
            mDriveSubsystem.stopModules();
            return true;
        } 
        return false;
    }

    @Override
    public void done() {}
}
