package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.ParallelAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close_5Note_MidSteal extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PhoreoTrajectoryAction traj1;
    private final PhoreoTrajectoryAction traj2;
    private final PhoreoTrajectoryAction traj3;
    private final PhoreoTrajectoryAction traj4;
    
    public Close_5Note_MidSteal(){
        traj1 = new PhoreoTrajectoryAction("Close_5Note_MidSteal", 1);
        traj2 = new PhoreoTrajectoryAction("Close_5Note_MidSteal", 2);
        traj3 = new PhoreoTrajectoryAction("Close_5Note_MidSteal", 3);
        traj4 = new PhoreoTrajectoryAction("Close_5Note_MidSteal", 4);

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
        DriveSubsystem.getInstance().overrideHeading(false);
        
        // Note 1 Poop and Drive to Pickup Note 2 and Score
        mSuperstructure.setGoalState(GoalState.POOP);
        runAction(new ParallelAction(
                traj1,
                new SeriesAction(
                    new WaitAction(1.0),
                    new SuperstructureAction(GoalState.INTAKE),
                    new WaitForAction(mSuperstructure::isHoldingNote),
                    new SuperstructureAction(GoalState.STOW)
                )
            )
        );
        mSuperstructure.setForceDisablePrep(false);
        runAction(new SuperstructureAction(GoalState.READY, kReadyWaitTime));
        runAction(new SuperstructureAction(GoalState.SHOOT, kShotWaitTime));

        // Note 3 Drive and Pickup and Score
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(new WaitAction(1.0));
        runAction(new ParallelAction(
                traj2,
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
            traj3,
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
            traj4,
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
