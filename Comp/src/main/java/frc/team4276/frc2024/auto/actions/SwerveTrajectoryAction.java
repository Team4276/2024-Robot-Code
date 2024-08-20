package frc.team4276.frc2024.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

import frc.team254.lib.geometry.Pose2d;

public class SwerveTrajectoryAction implements Action {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private PathPlannerPath path;
    private Pose2d initialPose;

    /**
     * Alliance Test
     */
    public SwerveTrajectoryAction(String name) {
        path = PathPlannerPath.fromPathFile(name);

        boolean isRed = AllianceChooser.getInstance().isAllianceRed();

        if (isRed) {
            path = path.flipPath();
        }

        initialPose = Pose2d.fromWPI(path.getPreviewStartingHolonomicPose());

        SmartDashboard.putString("Comp/Loaded path with alliance", isRed ? "Red" : "Blue");

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
