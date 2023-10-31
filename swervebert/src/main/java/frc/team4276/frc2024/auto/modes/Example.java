package frc.team4276.frc2024.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.auto.AutoEvents;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

public class Example extends SequentialCommandGroup {
    private DriveSubsystem mDriveSubsystem;

    private AutoEvents autoEvents;

    private final String path1 = "";

    private PathPlannerTrajectory traj1;

    private Command eventPath1;

    public Example(){
        mDriveSubsystem = DriveSubsystem.getInstance();
        autoEvents = new AutoEvents();

        traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath(path1, 1, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            Robot.alliance);

        eventPath1 = new FollowPathWithEvents(mDriveSubsystem.followPathCommand(traj1),traj1.getMarkers(), AutoEvents.eventMap);

        addCommands(
            new ParallelCommandGroup(
                eventPath1,
                new WaitUntilCommand(() -> autoEvents.waitForEvent())
            )

        );
    }   
}
