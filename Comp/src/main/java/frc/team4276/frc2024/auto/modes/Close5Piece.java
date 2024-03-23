package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.RobotState;

public class Close5Piece extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;
    private final PPSwerveTrajectoryAction traj3;
    private final PPSwerveTrajectoryAction traj4;
    private final PPSwerveTrajectoryAction traj5;


    public Close5Piece(){
        traj1 = new PPSwerveTrajectoryAction("CloseMiddlePickuptoCloseAmpPickup");
        traj2 = new PPSwerveTrajectoryAction("Stage1stShottoSubMid");
        traj3 = new PPSwerveTrajectoryAction("SubMidtoCloseMiddlePickup");
        traj4 = new PPSwerveTrajectoryAction("CloseMiddlePickuptoCloseAmpPickup2");
        traj5 = new PPSwerveTrajectoryAction("CloseAmpPickup2to5thShot");
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setGoalState(Superstructure.GoalState.READY_MIDDLE);
        runAction(traj1);
        mSuperstructure.setDynamicFourbarPosition(mRobotState.calcDynamicFourbarAngle());
        mSuperstructure.SHOOT();
        runAction(new WaitAction(0.25));
        runAction(traj2);
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
