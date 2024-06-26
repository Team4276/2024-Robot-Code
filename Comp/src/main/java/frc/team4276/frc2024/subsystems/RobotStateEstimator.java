package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.RobotState;

import frc.team254.lib.geometry.Pose2d;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.swerve.SwerveDriveOdometry;

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
                try {       
                mOdometry.update(
                    DriveSubsystem.getInstance().getHeading().toWPI(),
                    DriveSubsystem.getInstance().getModuleStates()
                );
                RobotState.getInstance().addOdomObservations(
                    timestamp, Pose2d.fromWPI(mOdometry.getPoseMeters()));

                } catch (Exception e) {
                    System.out.println(e.getMessage());
                  }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public void outputTelemetry(){
        SmartDashboard.putNumber("Robot X", RobotState.getInstance().getCurrentFieldToVehicle().getTranslation().x());
        SmartDashboard.putNumber("Robot Y", RobotState.getInstance().getCurrentFieldToVehicle().getTranslation().y());
        SmartDashboard.putNumber("Distance from Speaker", RobotState.getInstance()
            .getCurrentFieldToVehicle().getTranslation().distance(RobotState.getInstance().getPOIs().kSpeakerCenter));
    }

    

    public void resetOdometry(edu.wpi.first.math.geometry.Pose2d initialPose) {
        synchronized(RobotStateEstimator.this) {
            DriveSubsystem.getInstance().zeroHeading(initialPose.getRotation().getDegrees());
            mOdometry.resetPosition(
                DriveSubsystem.getInstance().getModuleStates(),
                initialPose);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.fromWPI(initialPose));
        }
    }
}
