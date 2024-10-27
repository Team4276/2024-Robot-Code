package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.ChoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.ParallelAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close_5Note_MidSteal extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final ChoreoTrajectoryAction traj1;
    private final ChoreoTrajectoryAction traj2;
    private final ChoreoTrajectoryAction traj3;
    private final ChoreoTrajectoryAction traj4;
    private final ChoreoTrajectoryAction traj5;
    
    public Close_5Note_MidSteal(){
        traj1 = new ChoreoTrajectoryAction("Close_5Note_MidSteal", 1);
        traj2 = new ChoreoTrajectoryAction("Close_5Note_MidSteal", 2);
        traj3 = new ChoreoTrajectoryAction("Close_5Note_MidSteal", 3);
        traj4 = new ChoreoTrajectoryAction("Close_5Note_MidSteal", 4);
        traj5 = new ChoreoTrajectoryAction("Close_5Note_MidSteal", 5);

    }

    private double kReadyWaitTime = 1.0;
    private double kShotWaitTime = 0.5;

    //TODO: add repickup
    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(true);
        mSuperstructure.setNominal();
        mSuperstructure.setForceDisablePrep(true);
        
        // Note 1 Poop and Drive to Pickup Note 2
        mSuperstructure.setGoalState(GoalState.POOP);
        runAction(new ParallelAction(
                traj1,
                new SeriesAction(
                    new WaitAction(0.5),
                    new LambdaAction(() -> mSuperstructure.setGoalState(GoalState.STOW))
                )
            )
        );
        mSuperstructure.setGoalState(GoalState.INTAKE);
        mSuperstructure.setForceDisablePrep(false);

        // Note 2 Drive and Score
        runAction(traj2);
        runAction(new SuperstructureAction(GoalState.READY, kReadyWaitTime));
        runAction(new SuperstructureAction(GoalState.SHOOT, kShotWaitTime));

        // Note 3 Drive and Pickup and Score
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(new WaitAction(1.0));
        runAction(new ParallelAction(
                traj3,
                new SeriesAction(
                    new WaitForAction(mSuperstructure::isHoldingNote),
                    new SuperstructureAction(GoalState.READY, kReadyWaitTime),
                    new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
                    new SuperstructureAction(GoalState.SKIM)
                )
            )
        );

        // Note 4 Drive to Pickup and Score
        runAction(new ParallelAction(
            traj4,
            new SeriesAction(
                new WaitAction(1.0),
                new SuperstructureAction(GoalState.INTAKE),
                new WaitForAction(mSuperstructure::isHoldingNote),
                new SuperstructureAction(GoalState.READY, kReadyWaitTime),
                new SuperstructureAction(GoalState.SHOOT, kShotWaitTime)
            )
        ));

        // Note 5 Drive to Pickup and Score
        runAction(new ParallelAction(
            traj5,
            new SeriesAction(
                new WaitAction(1.0),
                new SuperstructureAction(GoalState.INTAKE),
                new WaitForAction(mSuperstructure::isHoldingNote),
                new SuperstructureAction(GoalState.READY, kReadyWaitTime),
                new SuperstructureAction(GoalState.SHOOT, kShotWaitTime)
            )
        ));
        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
