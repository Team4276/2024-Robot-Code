package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.ChoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;
import frc.team254.lib.geometry.Pose2d;

public class ChoreoTest extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private ChoreoTrajectoryAction traj1;

    public ChoreoTest(String name){
        traj1 = new ChoreoTrajectoryAction(name);
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
