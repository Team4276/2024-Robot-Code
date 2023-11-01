package frc.team4276.frc2024.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

public class Example extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;

    private final String path1 = "";

    public Example(){
        mDriveSubsystem = DriveSubsystem.getInstance();

        PathPlannerTrajectory traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath(path1, 1, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            Robot.alliance);


        addCommands(
            new ParallelCommandGroup(
                mDriveSubsystem.followPathCommandEvents(traj1)
            )

        );
    }   
}
