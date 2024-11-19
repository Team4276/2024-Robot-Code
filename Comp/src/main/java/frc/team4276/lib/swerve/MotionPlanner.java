package frc.team4276.lib.swerve;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class MotionPlanner {
    private Trajectory<SwerveSample> mTrajectory;

    private double startTime = 0.0;
    private double timeOffset = 0.0;
    private double prevTime = 0.0;

    private boolean mIsFinished = true;

    private Translation2d mTranslationError = Translation2d.identity();
    private Rotation2d mRotationError = Rotation2d.identity();

    private Pose2d mTargetPose = Pose2d.identity();

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public MotionPlanner() {
        this.xController = new PIDController(DriveConstants.kAutoTranslationKp, 0.0, DriveConstants.kAutoTranslationKd,
                Constants.kLooperDt);
        this.yController = new PIDController(DriveConstants.kAutoTranslationKp, 0.0, DriveConstants.kAutoTranslationKd,
                Constants.kLooperDt);
        this.rotationController = new PIDController(DriveConstants.kAutoRotationKp, 0.0, DriveConstants.kAutoRotationKd,
                Constants.kLooperDt);
        this.rotationController.enableContinuousInput(-3.141592653589793, Math.PI);
    }

    public synchronized void setTrajectory(Trajectory<SwerveSample> traj, Pose2d currentPose,
            ChassisSpeeds currentSpeeds, double timestamp) {
        mTrajectory = traj;
        mIsFinished = false;
        this.startTime = timestamp;
        timeOffset = 0.0;
        prevTime = timestamp;
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp) {
        return update(currentPose, timestamp, false);
    }

    public synchronized ChassisSpeeds update(Pose2d currentPose, double timestamp, boolean isVirtual) {
        if (mTrajectory == null)
            return ChassisSpeeds.identity();
        
        SwerveSample targetState = mTrajectory.sampleAt(timestamp - startTime - timeOffset, false);

        if (targetState == null) {
            return ChassisSpeeds.identity();
        }

        if(targetState.getPose().getTranslation().getDistance(currentPose.getTranslation().toWPI()) > DriveConstants.kAutoMaxError && !isVirtual){
            double dt = timestamp - prevTime;
            timeOffset += dt;

            double[] dummyForces = { 0.0, 0.0, 0.0, 0.0 };

            targetState = mTrajectory.sampleAt(timestamp - startTime - timeOffset, false);
            targetState = new SwerveSample(targetState.t,
                    targetState.x, targetState.y, targetState.heading,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    dummyForces, dummyForces);
        }

        prevTime = timestamp;

        if (timestamp - startTime - timeOffset > mTrajectory.getTotalTime()) {
            mIsFinished = true;
        }

        mTargetPose = Pose2d.fromWPI(targetState.getPose());
        
        double xFF = targetState.vx;
        double yFF = targetState.vy;
        double xFFAccel = targetState.ax * DriveConstants.kAutoTransAccelFF;
        double yFFAccel = targetState.ay * DriveConstants.kAutoTransAccelFF;
        double xError = targetState.x - currentPose.getTranslation().x();
        double yError = targetState.y - currentPose.getTranslation().y();
        double xFeedback = xController.calculate(0.0, isVirtual ? 0.0 : xError);
        double yFeedback = yController.calculate(0.0, isVirtual ? 0.0 : yError);
        double thetaFF = targetState.omega;
        double thetaFFAccel = targetState.alpha * DriveConstants.kAutoRotAccelFF;
        double thetaError = targetState.heading - currentPose.getRotation().getRadians();
        double thetaFeedback = rotationController.calculate(0.0, isVirtual ? 0.0 : thetaError);

        mTranslationError = new Translation2d(xError, yError);
        mRotationError = Rotation2d.fromRadians(thetaError);

        return new ChassisSpeeds(xFFAccel + xFF + xFeedback, yFFAccel + yFF + yFeedback,
                thetaFFAccel + thetaFF + thetaFeedback);
    }

    public synchronized boolean isFinished() {
        return mIsFinished;
    }

    public synchronized Translation2d getTranslationError() {
        return mTranslationError;
    }

    public synchronized Rotation2d getRotationError() {
        return mRotationError;
    }

    public synchronized Pose2d getTargetPose() {
        return mTargetPose;
    }
}