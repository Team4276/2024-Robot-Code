package frc.team4276.frc2024.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Shoot extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    public Shoot() {}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(List.of(
            new LambdaAction(() -> mSuperstructure.setIntakeState(IntakeState.IDLE)),
            new LambdaAction(() -> mSuperstructure.setGoalState(GoalState.SUB_CLOSE)),
            new WaitAction(2.0),
            new LambdaAction(() -> mSuperstructure.SHOOT()),
            new WaitAction(0.25),
            new LambdaAction(() -> mSuperstructure.setIntakeState(IntakeState.IDLE)),
            new LambdaAction(() -> mSuperstructure.setGoalState(GoalState.READY_MIDDLE))
        )));
        
    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}
