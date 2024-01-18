package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.EmptySubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    private PathPlannerTrajectory traj;
    
    private FollowPathCommand mCommand2;

    private FollowPathHolonomic mCommand3;

    public PPSwerveTrajectoryAction(String name, ChassisSpeeds speeds, Rotation2d rot) {
        this.traj = PathPlannerPath.fromPathFile(name).getTrajectory(speeds, rot);

        mCommand = mDriveSubsystem.followPathCommand(traj);
        
        mCommand2 = new FollowPathCommand(PathPlannerPath.fromPathFile(name), null, null, null, null, null, null, new EmptySubsystem())

        mCommand3 = new FollowPathHolonomic(
            PathPlannerPath.fromPathFile(name), 
            mDriveSubsystem::getOdometry, 
            mDriveSubsystem.get, null, null, null, 0, 0, null, null, null)

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
