package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;
import frc.team4276.frc2024.subsystems.Superstructure;

public class SubSS2Piece extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;

    public SubSS2Piece(){
        traj1 = new PPSwerveTrajectoryAction("SubMiddleToCloseMiddlePickup");
        traj2 = new PPSwerveTrajectoryAction("CloseMiddlePickupToSub");
    }

    @Override
    protected void routine() throws AutoModeEndedException {

    }

    @Override
    public Pose2d getStartingPose() {
        return null;
    }
}
