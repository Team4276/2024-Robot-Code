package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.ChoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

import frc.team254.lib.geometry.Pose2d;

public class Close_4Note_Fast extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final ChoreoTrajectoryAction traj1;
    private final ChoreoTrajectoryAction traj2;
    private final ChoreoTrajectoryAction traj3;
    private final ChoreoTrajectoryAction traj4;
    private final ChoreoTrajectoryAction traj5;
    private final ChoreoTrajectoryAction traj6;

    public Close_4Note_Fast(){
        traj1 = new ChoreoTrajectoryAction("Close_4Note_Fast", 1);
        traj2 = new ChoreoTrajectoryAction("Close_4Note_Fast", 2);
        traj3 = new ChoreoTrajectoryAction("Close_4Note_Fast", 3);
        traj4 = new ChoreoTrajectoryAction("Close_4Note_Fast", 4);
        traj5 = new ChoreoTrajectoryAction("Close_4Note_Fast", 5);
        traj6 = new ChoreoTrajectoryAction("Close_4Note_Fast", 6);
    }

    private double kShotWaitTime = 0.5;
    private double kReadyWaitTime = 1.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(true);
        mSuperstructure.setNominal();
        mSuperstructure.setPrep(true);

        // Note 1 Shoot
        runAction(traj1);
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.SKIM);

        // Note 2 Drive and Pickup and Drive to Score
        runAction(traj2);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(traj3);
        
        // Note 2 Score
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.INTAKE);

        // Note 3 Drive and Pickup and Score
        runAction(traj4);
        runAction(new WaitForAction(mSuperstructure::isHoldingNote));
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.SKIM);

        // Note 4 Drive and Pickup and Score
        runAction(traj5);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(traj6);
        runAction(new WaitForAction(mSuperstructure::isHoldingNote));
        mSuperstructure.setGoalState(GoalState.READY);
        mDriveSubsystem.overrideHeading(true);
        runAction(new WaitAction(2.0));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.STOW);
        
        // Nominal Control States
        mDriveSubsystem.overrideHeading(false);
        mSuperstructure.setPrep(false);

    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}
