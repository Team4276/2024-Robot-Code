package frc.team4276.frc2024.auto.modes;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.ParallelAction;
import frc.team4276.frc2024.auto.actions.WaitForEventAction;

public class ActionExample extends AutoModeBase {
    private final String path1 = "";

    private final PathPlannerTrajectory traj1;

    public ActionExample(){
        traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath(path1, 1, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            Robot.alliance);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        new ParallelAction(List.of(
            new PPSwerveTrajectoryAction(traj1)
        ));
        new WaitForEventAction();

        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialHolonomicPose();
    }
}
