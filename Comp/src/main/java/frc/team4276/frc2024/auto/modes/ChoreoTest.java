package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.SwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;

import frc.team254.lib.geometry.Pose2d;

public class ChoreoTest extends AutoModeBase {
    private SwerveTrajectoryAction traj1;

    public ChoreoTest(String name){
        traj1 = new SwerveTrajectoryAction(name);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(0.0));
        runAction(traj1);
        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
