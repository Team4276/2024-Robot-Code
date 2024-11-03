package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close_4Note_Safe extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PhoreoTrajectoryAction traj1;
    private final PhoreoTrajectoryAction traj2;
    private final PhoreoTrajectoryAction traj3;
    private final PhoreoTrajectoryAction traj4;
    private final PhoreoTrajectoryAction traj5;
    private final PhoreoTrajectoryAction traj6;
    private final PhoreoTrajectoryAction traj7;

    public Close_4Note_Safe(){
        traj1 = new PhoreoTrajectoryAction("Close_4Note_Safe", 1);
        traj2 = new PhoreoTrajectoryAction("Close_4Note_Safe", 2);
        traj3 = new PhoreoTrajectoryAction("Close_4Note_Safe", 3);
        traj4 = new PhoreoTrajectoryAction("Close_4Note_Safe", 4);
        traj5 = new PhoreoTrajectoryAction("Close_4Note_Safe", 5);
        traj6 = new PhoreoTrajectoryAction("Close_4Note_Safe", 6);
        traj7 = new PhoreoTrajectoryAction("Close_4Note_Safe", 7);
    }

    private double kShotWaitTime = 0.5;
    private double kIntakeWaitTime = 1.0;
    private double kReadyWaitTime = 1.5;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(false);
        mSuperstructure.setNominal();
        DriveSubsystem.getInstance().overrideHeading(false);

        runAction(new SeriesAction(
        // Note 1 Shoot
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW),

        // Note 2 Drive and Pickup
            traj1,
            new SuperstructureAction(GoalState.INTAKE, kIntakeWaitTime),
            traj2,
            
        // Note 2 Drive and Score
            traj3,
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW),


        // Note 3 Drive and Pickup
            traj4,
            new SuperstructureAction(GoalState.INTAKE, kIntakeWaitTime),

        
        // Note 3 Drive and Score
            traj5,
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW),

        // Note 4 Drive and Pickup
            traj6,
            new SuperstructureAction(GoalState.INTAKE, kIntakeWaitTime),


        
        // Note 4 Drive and Shoot
            traj7,
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW)
        ));
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}
