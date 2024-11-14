package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;

public class BoxTest extends AutoModeBase {
    private final PhoreoTrajectoryAction traj1;

    public BoxTest(){
        traj1 = new PhoreoTrajectoryAction("Test2");
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
