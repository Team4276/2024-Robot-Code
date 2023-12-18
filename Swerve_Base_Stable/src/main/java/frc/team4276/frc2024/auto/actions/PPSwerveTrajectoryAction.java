package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    public PPSwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
        mCommand = mDriveSubsystem.followPathCommand(
                PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, Robot.alliance));

        SmartDashboard.putString("Loaded path with alliance", Robot.alliance.name());

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
            mDriveSubsystem.stopModules();
            return true;
        }
        return false;
    }

    @Override
    public void done() {
    }
}
