package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;

public class ActionExample extends AutoModeBase {
    private final String path1 = "Test1";

    private PPSwerveTrajectoryAction traj1;

    public ActionExample(){
        traj1 = new PPSwerveTrajectoryAction(path1, new ChassisSpeeds(), new Rotation2d());

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        SmartDashboard.putString("Auto Status", "Start");
        runAction(traj1);
        SmartDashboard.putString("Auto Status", "Finish");

        
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
