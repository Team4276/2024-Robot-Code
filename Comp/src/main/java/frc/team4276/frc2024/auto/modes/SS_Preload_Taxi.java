package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.ChoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class SS_Preload_Taxi extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private ChoreoTrajectoryAction traj1;

    public SS_Preload_Taxi(){
        traj1 = new ChoreoTrajectoryAction("SS_Preload_Taxi");

    }
    
    private double kShotWaitTime = 0.5;
    private double kReadyWaitTime = 2.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(false);
        mSuperstructure.setNominal();
        mSuperstructure.setPrep(true);
        DriveSubsystem.getInstance().overrideHeading(false);

        
        runAction(new SeriesAction(
        // Note 1 Shoot and Taxi
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.STOW),

            new WaitAction(10.0)

            // traj1
            
        ));

        mSuperstructure.setDynamic(true);
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}
