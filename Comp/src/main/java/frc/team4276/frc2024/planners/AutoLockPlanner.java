package frc.team4276.frc2024.planners;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.AutoAlignConstants;
import frc.team4276.frc2024.Constants.AutoLockConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team1678.lib.Util;
import frc.team1678.lib.swerve.ChassisSpeeds;

public class AutoLockPlanner {
    private ProfiledPIDController mProfiledPIDController;
    private double mGoalRotation = 0.0;
    private Pose2d mInitialPose;

    /** key: metres; values: radians */
    private static InterpolatingDoubleTreeMap fourbarAngleMap;

    private static AutoLockPlanner mInstance;

    public static AutoLockPlanner getInstance() {
        if (mInstance == null) {
            mInstance = new AutoLockPlanner();
        }
        return mInstance;
    }

    // TODO: move stuff to constants and tune
    private AutoLockPlanner() {
        mProfiledPIDController = new ProfiledPIDController(
            AutoLockConstants.kP, 0.0, AutoLockConstants.kD, 
            new Constraints(AutoLockConstants.kMaxAngularVelocity, AutoLockConstants.kMaxAngularAccel), Constants.kLooperDt);
        mProfiledPIDController.enableContinuousInput(0, 2 * Math.PI);
        mProfiledPIDController.setTolerance(AutoLockConstants.kTolerance);

        fourbarAngleMap = new InterpolatingDoubleTreeMap();
        fourbarAngleMap.put(1.05, Math.toRadians(45.0));
    }

    public boolean atSetpoint(){
        return Util.epsilonEquals(mProfiledPIDController.getSetpoint().position, mProfiledPIDController.getGoal().position, AutoLockConstants.kTolerance);
    }

    public void update(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        double distance = getSpeakerDistance(currentPose);
        double fourbar_angle = fourbarAngleMap.get(distance);

        Superstructure.getInstance().updateDynamicFourbarAngle(fourbar_angle, distance);

        if(mInitialPose == null){
            mInitialPose = currentPose;            
            mProfiledPIDController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
        }

        mGoalRotation = new Rotation2d(mInitialPose.getTranslation().inverse().
            translateBy(RobotState.getInstance().getPOIs().kSpeakerCenter), true).getRadians();

        DriveSubsystem.getInstance().updateAutoLockAngularVel(mProfiledPIDController.calculate(currentPose.getRotation().getRadians(), mGoalRotation));
    }

    public void reset() {
        mInitialPose = null;
    }

    public double getSpeakerDistance(Pose2d currentPose) {
        return currentPose.getTranslation().distance(RobotState.getInstance().getPOIs().kSpeakerCenter);
    }

    public boolean isValidSpeakerDistance(double distance) {
        return distance <= AutoAlignConstants.kValidSpeakerDistance;
    }


}
