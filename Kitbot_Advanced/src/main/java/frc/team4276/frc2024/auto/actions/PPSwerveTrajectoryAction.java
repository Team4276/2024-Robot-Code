package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.Constants.AutoConstants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.EmptySubsystem;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {

    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final Command mCommand;

    private PathPlannerPath path;

    private Alliance alliance = Alliance.Blue;

    public PPSwerveTrajectoryAction(String name, Rotation2d rot) {
        path = PathPlannerPath.fromPathFile(name);

        alliance = Robot.alliance;

        mCommand = new FollowPathHolonomic(
            path, 
            mDriveSubsystem::getOdometry, 
            mDriveSubsystem::getWPIMeasSpeeds, 
            mDriveSubsystem::setWPISpeeds, 
            AutoConstants.kTranslationPIDConstants, 
            AutoConstants.kRotationPIDConstants, 
            DriveConstants.kMaxVel, 
            DriveConstants.kTrackWidth * Math.sqrt(2), 
            new ReplanningConfig(), 
            () -> alliance == Alliance.Red, 
            new EmptySubsystem());

        SmartDashboard.putString("Loaded path with alliance", alliance.name());
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
        return path.getPreviewStartingHolonomicPose();
    }
}
