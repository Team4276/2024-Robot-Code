package frc.team4276.frc2024.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.ParallelAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;
import frc.team4276.frc2024.subsystems.Superstructure;

public class SubSS2Piece extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;

    public SubSS2Piece() {
        traj1 = new PPSwerveTrajectoryAction("SubSSCtoStagePickup");
        traj2 = new PPSwerveTrajectoryAction("StagePickuptoSubSSC");
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setGoalState(GoalState.SUB_CLOSE);
        runAction(new WaitAction(2.0));
        mSuperstructure.SHOOT();
        runAction(new WaitAction(0.25));
        runAction(new ParallelAction(List.of(traj1,
            new SeriesAction(
                new WaitAction(0.0)),
                new LambdaAction(() -> mSuperstructure.setGoalState(GoalState.SLOWTAKE))
            )));
        runAction(new ParallelAction(List.of(traj2,
            new SeriesAction(
                new WaitAction(0.25)),
                new LambdaAction(() -> mSuperstructure.setGoalState(GoalState.READY_LOW))
            )));
        mSuperstructure.setGoalState(GoalState.SUB_CLOSE);
        runAction(new WaitAction(2.0));
        mSuperstructure.SHOOT();
        runAction(new WaitAction(0.25));
        mSuperstructure.setGoalState(GoalState.READY_MIDDLE);


    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
