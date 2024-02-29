package frc.team4276.frc2024.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.LambdaRunnableAction;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.ParallelAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.FlywheelSubsystem.DesiredFlywheelMode;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;

public class SubMiddle2Piece extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;

    public SubMiddle2Piece() {
        traj1 = new PPSwerveTrajectoryAction("SubMiddleToCloseMiddlePickup");
        traj2 = new PPSwerveTrajectoryAction("CloseMiddlePickupToSub");
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // Shoot preload
        mSuperstructure.setFlywheelState(new FlywheelState(DesiredFlywheelMode.RPM, -4500, -4500));
        runAction(new LambdaRunnableAction(() -> mSuperstructure.setStateJankIntake(), 4.0));
        mSuperstructure.setIntakeState(IntakeState.FOOT);
        runAction(new WaitAction(1.0));

        // Drive to note and intake
        mSuperstructure.setIntakeState(IntakeState.IDLE);
        mSuperstructure.setFlywheelState(new FlywheelState());
        runAction(new ParallelAction(List.of(
                new SeriesAction(List.of(
                        new LambdaRunnableAction(() -> mSuperstructure.setStateJankHold(), 1.0),
                        new LambdaAction(() -> mSuperstructure.setIntakeState(IntakeState.FASTAKE)),
                        new LambdaRunnableAction(() -> mSuperstructure.setStateJankIntake(), 3.0))),
                traj1)));

        // Drive to Sub and shoot
        runAction(new ParallelAction(List.of(
                new LambdaRunnableAction(() -> mSuperstructure.setStateJankHold(), 3.0),
                traj2)));
        mSuperstructure.setFlywheelState(new FlywheelState(DesiredFlywheelMode.RPM, -4500, -4500));
        runAction(new LambdaRunnableAction(() -> mSuperstructure.setStateJankIntake(), 3.0));
        mSuperstructure.setIntakeState(IntakeState.FOOT);

    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
