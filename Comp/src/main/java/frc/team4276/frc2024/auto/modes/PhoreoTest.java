package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;
import frc.team254.lib.geometry.Pose2d;

public class PhoreoTest extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private PhoreoTrajectoryAction traj1;

    public PhoreoTest(String name){
        traj1 = new PhoreoTrajectoryAction(name);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setNominal();
        mSuperstructure.setGoalState(GoalState.STOW);
        mSuperstructure.setForceDisablePrep(true);
        mSuperstructure.setDynamic(false);
        
        runAction(new WaitAction(0.0));
        runAction(traj1);
        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
