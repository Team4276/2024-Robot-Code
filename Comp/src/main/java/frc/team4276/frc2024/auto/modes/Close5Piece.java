package frc.team4276.frc2024.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.SeriesAction;
import frc.team4276.frc2024.auto.actions.SuperstructureAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close5Piece extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;
    private final PPSwerveTrajectoryAction traj3;
    private final PPSwerveTrajectoryAction traj4;
    private final PPSwerveTrajectoryAction traj5;


    public Close5Piece(){
        traj1 = new PPSwerveTrajectoryAction("SSStarttoStage1stShot");
        traj2 = new PPSwerveTrajectoryAction("Stage1stShottoSubMid");
        traj3 = new PPSwerveTrajectoryAction("SubMidtoCloseMiddlePickup");
        traj4 = new PPSwerveTrajectoryAction("CloseMiddlePickuptoCloseAmpPickup2");
        traj5 = new PPSwerveTrajectoryAction("CloseAmpPickup2to5thShot");
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setGoalState(Superstructure.GoalState.READY_MIDDLE);
        runAction(traj1);
        mSuperstructure.setAutoShoot(true);
        runAction(new SeriesAction(List.of(
            new SuperstructureAction(GoalState.DYNAMIC),
            new SuperstructureAction(GoalState.SLOWTAKE),
            traj2,
            new SuperstructureAction(GoalState.DYNAMIC),
            new SuperstructureAction(GoalState.SLOWTAKE),
            traj3,
            new SuperstructureAction(GoalState.DYNAMIC),
            new SuperstructureAction(GoalState.SLOWTAKE),
            traj4,
            new SuperstructureAction(GoalState.DYNAMIC)
        )));
        mSuperstructure.setGoalState(GoalState.SLOWTAKE);
        runAction(traj5);
        runAction(new SuperstructureAction(GoalState.DYNAMIC));
        mSuperstructure.setAutoShoot(false);
        mSuperstructure.setGoalState(GoalState.READY_MIDDLE);
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
