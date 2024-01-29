package frc.team4276.frc2024.subsystems;

import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.RobotState;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.swerve.SwerveDriveOdometry;

import frc.team254.lib.geometry.Pose2d;

import edu.wpi.first.wpilibj.Timer;

public class RobotStateEstimator extends Subsystem {
  // Odometry class for tracking robot pose
  private SwerveDriveOdometry mOdometry;

    private RobotStateEstimator() {
        mOdometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                DriveSubsystem.getInstance().getModuleStates());
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
                mOdometry.update(
                    DriveSubsystem.getInstance().getHeading(),
                    DriveSubsystem.getInstance().getModuleStates()
                );
                RobotState.getInstance().addOdomToVehicleObservation(
                    timestamp, Pose2d.fromWPI(mOdometry.getPoseMeters()));
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d initialPose) {
        synchronized(RobotStateEstimator.this) {
            mOdometry.resetPosition(
                DriveSubsystem.getInstance().getModuleStates(),
                initialPose);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.fromWPI(initialPose));
        }
    }
}
