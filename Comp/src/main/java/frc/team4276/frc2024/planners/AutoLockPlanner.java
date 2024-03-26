package frc.team4276.frc2024.planners;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.AutoAlignConstants;
import frc.team4276.frc2024.Constants.AutoLockConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;

import frc.team1678.lib.swerve.ChassisSpeeds;

public class AutoLockPlanner {
    private TrapezoidProfile mTrapezoidProfile;
    private PIDController mPidController;
    private Optional<TrapezoidProfile.State> mStartState;
    private Optional<TrapezoidProfile.State> mGoalState;
    private double mGoalStartTime;

    /** key: metres; values: radians */
    private static InterpolatingDoubleTreeMap fourbarAngleMap = new InterpolatingDoubleTreeMap();

    static {
        fourbarAngleMap.put(1.05, Math.toRadians(45.0));
    }

    private static AutoLockPlanner mInstance;

    public static AutoLockPlanner getInstance() {
        if (mInstance == null) {
            mInstance = new AutoLockPlanner();
        }
        return mInstance;
    }

    // TODO: move stuff to constants and tune
    private AutoLockPlanner() {
        mTrapezoidProfile = new TrapezoidProfile(new Constraints(
                AutoLockConstants.kMaxAngularVelocity, AutoLockConstants.kMaxAngularAccel));

        mPidController = new PIDController(0.0, 0.0, 0.0);
        mPidController.enableContinuousInput(0.0, 2 * Math.PI);
        mPidController.setTolerance(Math.PI / 180.0, Math.PI / 180.0);

        fourbarAngleMap = new InterpolatingDoubleTreeMap();
    }

    public void update(double t, Pose2d currentPose, ChassisSpeeds currentVel) {
        double distance = getSpeakerDistance(currentPose);
        double fourbar_angle = fourbarAngleMap.get(distance);

        Superstructure.getInstance().updateDynamicFourbarAngle(fourbar_angle, distance);

        // if (mStartState == null) {
        //     mStartState = Optional.of(new TrapezoidProfile.State(currentPose.getRotation().getRadians(),
        //             currentVel.omegaRadiansPerSecond));

        //     mGoalState = Optional
        //             .of(new TrapezoidProfile.State(new Rotation2d(RobotState.getInstance().getPOIs().kSpeakerCenter
        //                     .translateBy((currentPose.getTranslation().inverse())), true).getRadians(), 0.0));

        //     mGoalStartTime = t;

        //     mPidController.setSetpoint(mGoalState.get().position);
        // }

        double dtheta = 0.0;

        // if (mTrapezoidProfile.isFinished(t - mGoalStartTime)) {
        //     dtheta = mPidController.calculate(currentPose.getRotation().getRadians());

        // } else {
        //     double adjustment = 0.0;
        //     double error = mGoalState.get().position - mStartState.get().position;
        //     if (error > Math.PI) {
        //         adjustment = (2 * Math.PI);
        //     } else if (error < -Math.PI) {
        //         adjustment = -(2 * Math.PI);
        //     }

        //     dtheta = mTrapezoidProfile.calculate(t - mGoalStartTime,
        //             new TrapezoidProfile.State(mStartState.get().position + adjustment, mStartState.get().velocity),
        //             mGoalState.get()).velocity;
        // }

        DriveSubsystem.getInstance().updateAutoLockRotationSpeed(dtheta);
    }

    public void reset() {
        mStartState = null;
        mGoalState = null;
        mGoalStartTime = -1.0;
        mPidController.reset();

    }

    public double getSpeakerDistance(Pose2d currentPose) {
        return currentPose.getTranslation().distance(RobotState.getInstance().getPOIs().kSpeakerCenter);
    }

    public boolean isValidSpeakerDistance(double distance) {
        return distance <= AutoAlignConstants.kValidSpeakerDistance;
    }
}
