package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.Superstructure;

public class SubSSCTaxi extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private PPSwerveTrajectoryAction traj1;

    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setGoalState(Superstructure.GoalState.SUB_CLOSE);
        runAction(new WaitAction(2.0));
        mSuperstructure.SHOOT();
        runAction(new WaitAction(0.25));
        mSuperstructure.setGoalState(Superstructure.GoalState.READY_MIDDLE);
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}