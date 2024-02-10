package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;

public class PPTest extends AutoModeBase {
    private PPSwerveTrajectoryAction traj1;

    public PPTest(String name, Rotation2d start_rot){
        traj1 = new PPSwerveTrajectoryAction(name, start_rot);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(traj1);
        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}