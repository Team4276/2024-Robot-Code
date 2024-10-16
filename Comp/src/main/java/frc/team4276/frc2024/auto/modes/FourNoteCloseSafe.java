package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.SwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class FourNoteCloseSafe extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final SwerveTrajectoryAction traj1;
    private final SwerveTrajectoryAction traj2;
    private final SwerveTrajectoryAction traj3;
    private final SwerveTrajectoryAction traj4;
    private final SwerveTrajectoryAction traj5;
    private final SwerveTrajectoryAction traj6;
    private final SwerveTrajectoryAction traj7;
    private final SwerveTrajectoryAction traj8;

    public FourNoteCloseSafe(){
        traj1 = new SwerveTrajectoryAction("Close_4Note_Safe", 1);
        traj2 = new SwerveTrajectoryAction("Close_4Note_Safe", 2);
        traj3 = new SwerveTrajectoryAction("Close_4Note_Safe", 3);
        traj4 = new SwerveTrajectoryAction("Close_4Note_Safe", 4);
        traj5 = new SwerveTrajectoryAction("Close_4Note_Safe", 5);
        traj6 = new SwerveTrajectoryAction("Close_4Note_Safe", 6);
        traj7 = new SwerveTrajectoryAction("Close_4Note_Safe", 7);
        traj8 = new SwerveTrajectoryAction("Close_4Note_Safe", 8);
    }

    private double kShotWaitTime = 0.5;
    private double kIntakeWaitTime = 1.0;

    //TODO: add intake ready checks to subsystem
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
