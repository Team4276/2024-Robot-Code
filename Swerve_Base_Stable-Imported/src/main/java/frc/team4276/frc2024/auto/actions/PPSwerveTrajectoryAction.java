package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    private PathPlannerTrajectory traj;

    public PPSwerveTrajectoryAction(PathPlannerTrajectory trajectory) {
        this.traj = trajectory;
        mCommand = mDriveSubsystem.followPathCommand(traj);

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

    public Pose2d getInitialPose(){
        return traj.getInitialTargetHolonomicPose();
    }
}
