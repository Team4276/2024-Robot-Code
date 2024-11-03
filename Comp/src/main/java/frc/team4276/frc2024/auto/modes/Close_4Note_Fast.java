package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

import frc.team254.lib.geometry.Pose2d;

public class Close_4Note_Fast extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final PhoreoTrajectoryAction traj1;
    private final PhoreoTrajectoryAction traj2;
    private final PhoreoTrajectoryAction traj3;
    private final PhoreoTrajectoryAction traj4;
    private final PhoreoTrajectoryAction traj5;
    private final PhoreoTrajectoryAction traj6;

    public Close_4Note_Fast(){
        traj1 = new PhoreoTrajectoryAction("Close_4Note_Fast", 1);
        traj2 = new PhoreoTrajectoryAction("Close_4Note_Fast", 2);
        traj3 = new PhoreoTrajectoryAction("Close_4Note_Fast", 3);
        traj4 = new PhoreoTrajectoryAction("Close_4Note_Fast", 4);
        traj5 = new PhoreoTrajectoryAction("Close_4Note_Fast", 5);
        traj6 = new PhoreoTrajectoryAction("Close_4Note_Fast", 6);
    }

    private double kShotWaitTime = 0.5;
    private double kReadyWaitTime = 1.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(true);
        mSuperstructure.setNominal();
        mSuperstructure.setPrep(true);
        mDriveSubsystem.overrideHeading(false);

        runAction(new SeriesAction(
        // Note 1 Shoot
            traj1,
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.SKIM),

            
        // Note 2 Drive and Pickup and Drive to Score
            traj2,
            new SuperstructureAction(GoalState.INTAKE),
            traj3,

        // Note 2 Score
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.INTAKE),

        // Note 3 Drive and Pickup and Score
            traj4,
            new WaitForAction(mSuperstructure::isHoldingNote),
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.SKIM),

        // Note 4 Drive and Pickup and Score
            traj5,
            new SuperstructureAction(GoalState.INTAKE),
            traj6,
            new WaitForAction(mSuperstructure::isHoldingNote),
            new SuperstructureAction(GoalState.READY),
            new LambdaAction(() -> mDriveSubsystem.overrideHeading(true)),
            new WaitAction(2.0),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW)

        ));
        
        // Nominal Control States
        mDriveSubsystem.overrideHeading(false);
        mSuperstructure.setPrep(false);

    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}
