package frc.team4276.frc2024.auto.modes;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.PPSwerveTrajectoryAction;

public class NOPE extends AutoModeBase {
    private final PPSwerveTrajectoryAction traj1;
    private final PPSwerveTrajectoryAction traj2;
    private final PPSwerveTrajectoryAction traj3;
    private final PPSwerveTrajectoryAction traj4;

    private final int nopes;

    /**
     * @param nopes 1-4 nope notes
     */
    public NOPE(int nopes){
        this.nopes = nopes;

        traj1 = new PPSwerveTrajectoryAction("SubSSCtoMid1");
        traj2 = new PPSwerveTrajectoryAction("Mid1toMid2");
        traj3 = new PPSwerveTrajectoryAction("Mid2toMid3");
        traj4 = new PPSwerveTrajectoryAction("Mid3toMid4");
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(traj1);

        if(nopes > 1){
            runAction(traj2);
        }
        
        if(nopes > 2){
            runAction(traj3);
        }

        if(nopes > 3){
            runAction(traj4);
        }
    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
}
