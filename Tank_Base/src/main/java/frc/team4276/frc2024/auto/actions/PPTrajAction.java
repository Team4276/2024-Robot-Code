package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;

public class PPTrajAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    public PPTrajAction(PathPlannerTrajectory trajectory){
        mCommand = mDriveSubsystem.followPathCommand(trajectory);
    }

    @Override
    public void start() {
        mCommand.initialize();

    }

    @Override
    public void update() {
        mCommand.execute();
    }

    @Override
    public boolean isFinished() {
        if (mCommand.isFinished()) {

            return true;
        } 
        return false;
    }

    @Override
    public void done() {}
}
