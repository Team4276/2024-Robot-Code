package frc.team1678.lib.swerve;

import frc.team4276.frc2024.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner {
    private final PIDController forwardController;
    private final PIDController strafeController;
    private final ProfiledPIDController rotationController;

    private final PIDController snapController;
    
    private final HolonomicDriveController mDriveController;

    private Trajectory mCurrentTrajectory;
    private Rotation2d mTargetRotation;
    private Double mStartTime = Double.NaN;

    private boolean isFinished = false;

    public DriveMotionPlanner() {
        forwardController = new PIDController(AutoConstants.kTranslationP, 0.0, AutoConstants.kTranslationD);
        strafeController = new PIDController(AutoConstants.kTranslationP, 0.0, AutoConstants.kTranslationD);
        rotationController = new ProfiledPIDController(AutoConstants.kRotationP, 0.0, AutoConstants.kRotationD, AutoConstants.kThetaControllerConstraints);
        snapController = new PIDController(SnapConstants.kP, SnapConstants.kI, SnapConstants.kD);
        
        rotationController.enableContinuousInput(0, 2 * Math.PI);
        snapController.enableContinuousInput(0, 2 * Math.PI);

        mDriveController = new HolonomicDriveController(forwardController, strafeController, rotationController);

        SmartDashboard.putNumber("Desired traj speed", 0.0);
    }

    public double calculateRotationalAdjustment(double target_heading, double current_heading) {
        if (snapController.getSetpoint() != target_heading) {
            snapController.reset();
            snapController.setSetpoint(target_heading);
        }
        return snapController.calculate(current_heading, target_heading);
    }

    public void setTrajectory(Trajectory trajectory, Rotation2d heading, Pose2d current_pose) {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset(current_pose.getRotation().getRadians());
        mStartTime = Double.NaN;
        mCurrentTrajectory = trajectory;
        isFinished = false;
        setTargetHeading(heading);
    }

    public void setTargetHeading(Rotation2d newHeading) {
        mTargetRotation = newHeading;
    }

    public ChassisSpeeds update(Pose2d current_state, double timestamp) {
        if (mStartTime.isNaN()) {
            mStartTime = Timer.getFPGATimestamp();
        }

        if (timestamp > mStartTime + mCurrentTrajectory.getTotalTimeSeconds()) {
            isFinished = true;    
        }

        if (mCurrentTrajectory == null) {
            return new ChassisSpeeds();
        }
        
        Trajectory.State desired_state = mCurrentTrajectory.sample(timestamp - mStartTime);

        SmartDashboard.putNumber("Desired traj speed", desired_state.velocityMetersPerSecond);

        return mDriveController.calculate(current_state, desired_state, mTargetRotation);
    }

    public Double getXError(double currentX, double timestamp) {
        if (mCurrentTrajectory == null) {
            return Double.NaN;
        }
        return mCurrentTrajectory.sample(timestamp - mStartTime).poseMeters.getX() - currentX;
    }

    public Double getYError(double getY, double timestamp) {
        if (mCurrentTrajectory == null) {
            return Double.NaN;
        }
        return mCurrentTrajectory.sample(timestamp - mStartTime).poseMeters.getY() - getY;
    }

    public Double getRotationalError(double currentRotation) {
        if (mTargetRotation == null) {
            return Double.NaN;
        }
        return mTargetRotation.getDegrees() - currentRotation;
    }

    public Double getRotationalTarget() {
        if (mTargetRotation == null) {
            return Double.NaN;
        }
        return mTargetRotation.getDegrees();
    }

    public boolean isFinished() {
        return mCurrentTrajectory != null && isFinished;
    }

}
