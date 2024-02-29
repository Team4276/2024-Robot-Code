package frc.team4276.frc2024.auto.paths;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Translation2d;
import frc.team4276.frc2024.Constants.DriveConstants;

public class PathPlannerPaths {

    public PathPlannerPath generate(List<Translation2d> positions, List<RotationTarget> rotation_targets, PathConstraints path_constraints){
        List<PathPoint> waypoints = new ArrayList<>();

        for (int i = 0; i < positions.size(); i++){
            waypoints.add(i, new PathPoint(positions.get(i), rotation_targets.get(i), path_constraints));
        }

        return PathPlannerPath.fromPathPoints(waypoints, path_constraints, 
            new GoalEndState(0.0, rotation_targets.get(rotation_targets.size() - 1).getTarget(), true));
    }

    public PathPlannerPath getStageStartToFirstScore() {
        List<Translation2d> positions = new ArrayList<>();
        List<RotationTarget> rotation_targets = new ArrayList<>();

        return generate(positions, rotation_targets, 
            new PathConstraints(DriveConstants.kAutoLimits.kMaxDriveVelocity, DriveConstants.kAutoLimits.kMaxAccel, 
            DriveConstants.kAutoLimits.kMaxAngularVelocity, DriveConstants.kAutoLimits.kMaxAngularAccel));
    }
}
