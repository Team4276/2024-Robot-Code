package frc.team4276.frc2024.auto.actions;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import frc.team4276.frc2024.Constants.AutoConstants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;
import frc.team4276.frc2024.RobotState;



public class PPSwerveTrajectoryAction implements Action {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Command mCommand;

    private PathPlannerPath path;
    private Pose2d initialPose;

    /**
     * Alliance Test
     */
    public PPSwerveTrajectoryAction(String name) {
        path = PathPlannerPath.fromPathFile(name);

        Alliance alliance = AllianceChooser.getInstance().getAlliance();

        mCommand = new FollowPathHolonomic(
            path, 
            mRobotState::getWPICurrentFieldToVehicle, 
            mDriveSubsystem::getWPIMeasSpeeds, 
            mDriveSubsystem::updatePPPathFollowingSetpoint, 
            AutoConstants.kTranslationPIDConstants, 
            AutoConstants.kRotationPIDConstants, 
            DriveConstants.kMaxVel, 
            DriveConstants.kTrackWidth * Math.sqrt(2) / 2, 
            new ReplanningConfig(), 
            () -> alliance == Alliance.Red, 
            new EmptySubsystem());

        initialPose = alliance == Alliance.Red ? path.flipPath().getPreviewStartingHolonomicPose() 
            : path.getPreviewStartingHolonomicPose();        

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
        return initialPose;
    }
}
