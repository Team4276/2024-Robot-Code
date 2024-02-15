package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.Constants.AutoConstants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;
import frc.team4276.frc2024.RobotState;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class PPSwerveTrajectoryAction implements Action {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Command mCommand;

    private PathPlannerPath path;

    public PPSwerveTrajectoryAction(String name, Rotation2d rot) {
        path = PathPlannerPath.fromPathFile(name);

        Alliance alliance = AllianceChooser.getInstance().getAlliance();

        mCommand = new FollowPathHolonomic(
            path, 
            mRobotState::getWPICurrentFieldToVehicle, 
            mDriveSubsystem::getWPIMeasSpeeds, 
            mDriveSubsystem::setWPISpeeds, 
            AutoConstants.kTranslationPIDConstants, 
            AutoConstants.kRotationPIDConstants, 
            DriveConstants.kMaxVel, 
            DriveConstants.kTrackWidth * Math.sqrt(2) / 2, 
            new ReplanningConfig(), 
            () -> alliance == Alliance.Red, 
            new EmptySubsystem());

        System.out.println("Loaded path with alliance" + alliance.name());
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
