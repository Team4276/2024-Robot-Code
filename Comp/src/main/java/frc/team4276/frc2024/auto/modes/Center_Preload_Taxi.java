package frc.team4276.frc2024.auto.modes;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.SwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Center_Preload_Taxi extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private SwerveTrajectoryAction traj1;

    public Center_Preload_Taxi(){
        traj1 = new SwerveTrajectoryAction("Center_Preload_Taxi");

    }

    private double kShotWaitTime = 0.5;
    private double kReadyWaitTime = 1.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Set Control States
        mSuperstructure.setDynamic(true);
        mSuperstructure.setNominal();
        DriveSubsystem.getInstance().overrideHeading(false);

        runAction(new SeriesAction(
        // Note 1 Score
            new SuperstructureAction(GoalState.READY, kReadyWaitTime),
            new SuperstructureAction(GoalState.SHOOT, kShotWaitTime),
            new SuperstructureAction(GoalState.INTAKE),

        // Note 2 Pickup and Score
            traj1,
            new WaitForAction(mSuperstructure::isHoldingNote),
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
