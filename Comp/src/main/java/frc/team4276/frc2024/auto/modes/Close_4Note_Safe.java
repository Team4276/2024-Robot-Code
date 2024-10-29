package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.ChoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close_4Note_Safe extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final ChoreoTrajectoryAction traj1;
    private final ChoreoTrajectoryAction traj2;
    private final ChoreoTrajectoryAction traj3;
    private final ChoreoTrajectoryAction traj4;
    private final ChoreoTrajectoryAction traj5;
    private final ChoreoTrajectoryAction traj6;
    private final ChoreoTrajectoryAction traj7;

    public Close_4Note_Safe(){
        traj1 = new ChoreoTrajectoryAction("Close_4Note_Safe", 1);
        traj2 = new ChoreoTrajectoryAction("Close_4Note_Safe", 2);
        traj3 = new ChoreoTrajectoryAction("Close_4Note_Safe", 3);
        traj4 = new ChoreoTrajectoryAction("Close_4Note_Safe", 4);
        traj5 = new ChoreoTrajectoryAction("Close_4Note_Safe", 5);
        traj6 = new ChoreoTrajectoryAction("Close_4Note_Safe", 6);
        traj7 = new ChoreoTrajectoryAction("Close_4Note_Safe", 7);
    }

    private double kShotWaitTime = 0.5;
    private double kIntakeWaitTime = 1.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(false);
        mSuperstructure.setNominal();
        mSuperstructure.setPrep(true);

        // Note 1 Shoot
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitForAction(mSuperstructure::isReady));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.STOW);

        // Note 2 Drive and Pickup
        runAction(traj1);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(new WaitAction(kIntakeWaitTime));
        runAction(traj2);

        // Note 2 Drive and Score
        runAction(traj3);
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitForAction(mSuperstructure::isReady));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.STOW);

        // Note 3 Drive and Pickup
        runAction(traj4);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(new WaitAction(kIntakeWaitTime));
        runAction(traj5);

        // Note 3 Drive and Score
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitForAction(mSuperstructure::isReady));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.STOW);

        // Note 4 Drive and Pickup
        runAction(traj6);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(new WaitAction(kIntakeWaitTime));
        runAction(traj7);

        // Note 4 Drive and Shoot
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitForAction(mSuperstructure::isReady));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.STOW);
        
        mSuperstructure.setDynamic(true);
        mSuperstructure.setPrep(false);

    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}