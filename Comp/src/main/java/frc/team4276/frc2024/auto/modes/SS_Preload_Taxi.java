package frc.team4276.frc2024.auto.modes;

import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.LambdaAction;
import frc.team4276.frc2024.auto.actions.PhoreoTrajectoryAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class SS_Preload_Taxi extends AutoModeBase {
    private Superstructure mSuperstructure = Superstructure.getInstance();

    private PhoreoTrajectoryAction traj1;

    public SS_Preload_Taxi(){
        traj1 = new PhoreoTrajectoryAction("SS_Preload_Taxi");

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

            new WaitAction(9.0)
            
            // ,

            // new LambdaAction(() -> DriveSubsystem.getInstance().updatePathFollowingSetpoint(
            //     new ChassisSpeeds(-2.0, 0.0, 0.0))),
            // new WaitAction(2.0),
            // new LambdaAction(() -> DriveSubsystem.getInstance().updatePathFollowingSetpoint(
            //     new ChassisSpeeds()))

        ));
    }

    @Override
    public Pose2d getStartingPose() {
        // return traj1.getInitialPose();
        return Pose2d.identity();
    }
    
}