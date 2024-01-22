package frc.team4276.frc2024.subsystems;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.RobotState;

public class RobotStateEstimator extends Subsystem {

    private RobotStateEstimator() {

    }

    private static RobotStateEstimator mInstance;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new RobotStateEstimator();
        }
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                //TODO: quick fix the odometry object; seperate it from the drive subsystem class
                RobotState.getInstance().addOdomToVehicleObservation(
                    timestamp, Pose2d.fromWPIPose2d(DriveSubsystem.getInstance().getOdometry()));
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }
}
