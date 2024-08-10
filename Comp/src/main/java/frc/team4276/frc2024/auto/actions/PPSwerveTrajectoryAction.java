package frc.team4276.frc2024.auto.actions;

import java.lang.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;
import frc.team4276.frc2024.RobotState;

import frc.team254.lib.geometry.Pose2d;

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

        boolean isRed = AllianceChooser.getInstance().isAllianceRed();

        mCommand = new FollowPathHolonomic(
            path, 
            mRobotState::getWPILatestFieldToVehicle, 
            mDriveSubsystem::getWPIMeasSpeeds, 
            mDriveSubsystem::updatePPPathFollowingSetpoint, 
            DriveConstants.kAutoTranslationPIDConstants, 
            DriveConstants.kAutoRotationPIDConstants, 
            DriveConstants.kMaxVel, 
            DriveConstants.kTrackWidth * Math.sqrt(2) / 2, 
            new ReplanningConfig(), 
            () -> isRed, 
            new EmptySubsystem());

        initialPose = Pose2d.fromWPI(isRed ? path.flipPath().getPreviewStartingHolonomicPose() 
            : path.getPreviewStartingHolonomicPose());        

        SmartDashboard.putString("Loaded path with alliance", isRed ? "Red" : "Blue");
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
        return mCommand.isFinished();
    }

    @Override
    public void done() {
    }

    public Pose2d getInitialPose(){
        return initialPose;
    }
}
