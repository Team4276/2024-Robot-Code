package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;

public class ActionExample extends AutoModeBase {
    private final String path1 = "Example";

    private PPSwerveTrajectoryAction traj1;

    public ActionExample(){
        traj1 = new PPSwerveTrajectoryAction(path1);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(traj1);
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
