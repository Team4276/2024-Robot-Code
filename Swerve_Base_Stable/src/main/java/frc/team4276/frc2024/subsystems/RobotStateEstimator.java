package frc.team4276.frc2024.subsystems;

import frc.team4276.frc2024.RobotState;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

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

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d initialPose) {
        synchronized(RobotStateEstimator.this) {
            DriveSubsystem.getInstance().resetOdometry(initialPose);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.fromWPIPose2d(initialPose));
        }
    }
}
