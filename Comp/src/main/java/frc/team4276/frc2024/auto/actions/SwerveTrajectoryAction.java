package frc.team4276.frc2024.auto.actions;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;
import frc.team254.lib.geometry.Pose2d;

public class SwerveTrajectoryAction implements Action {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private PathPlannerPath path;
    private Pose2d initialPose;
    private Command command;
    private PathFollowingController controller;
    private RobotConfig config;
    private Subsystem s;

    /**
     * Alliance Test
     */
    public SwerveTrajectoryAction(String name) {
        try {
            path = PathPlannerPath.fromChoreoTrajectory(name);

            boolean isRed = AllianceChooser.getInstance().isAllianceRed();

            controller = new PathFollowingController() {
                @Override
                public double getPositionalError() {
                    return 0;
                }

                @Override
                public boolean isHolonomic() {
                    return true;
                }

                @Override
                public void reset(edu.wpi.first.math.geometry.Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                    
                }

                @Override
                public ChassisSpeeds calculateRobotRelativeSpeeds(edu.wpi.first.math.geometry.Pose2d currentPose,
                        PathPlannerTrajectoryState targetState) {

                    return new ChassisSpeeds();
                }
            };

            config = new RobotConfig(0, 0, null, 0, 0);

            command = new FollowPathCommand(path, RobotState.getInstance()::getWPILatestFieldToVehicle,
                mDriveSubsystem::getWPIMeasSpeeds, mDriveSubsystem::updatePPPathFollowingSetpoint, controller,
                 config, AllianceChooser.getInstance()::isAllianceRed, s);




            initialPose = Pose2d.fromWPI(path.getStartingDifferentialPose());

            SmartDashboard.putString("Comp/Loaded path with alliance", isRed ? "Red" : "Blue");

        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

    }

    @Override
    public void start() {
        mDriveSubsystem.setPathFollowingPath(path);
        
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return mDriveSubsystem.isPathFinished();
    }

    @Override
    public void done() {
    }

    public Pose2d getInitialPose() {
        return initialPose;
    }
}
