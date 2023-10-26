package frc.team4276.frc2024.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

public class followPathWithEvents{
    private HashMap<String, Command> eventMap = new HashMap<>();

    public followPathWithEvents(){
    
    }

    public Command followPPPEvents(String name, DriveSubsystem driveSubsystem, double maxSpeed){
        PathPlannerTrajectory path = PathPlanner.loadPath(name, 
            maxSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        if (Robot.alliance != Alliance.Invalid){
            path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, Alliance.Blue);
        }

        FollowPathWithEvents command = new FollowPathWithEvents(
        driveSubsystem.followPathCommand(path), 
        path.getMarkers(), eventMap);

        return command;
      }
}