package frc.team4276.lib.path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

// Credit: 4481 (2024)

/**
 * Can be used to aim a robot to a selected point. The translation will not be affected.
 */
public class DriveToTrajectoryState{

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;
    private final double maxModuleSpeed;
    private final double mpsToRps;
    private final double accelFF;

    public DriveToTrajectoryState(PIDConstants translationConstants, PIDConstants rotationConstants, double period, double maxModuleSpeed, double driveBaseRadius, double accelFF) {
        this.xController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.yController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.rotationController = new ProfiledPIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, new TrapezoidProfile.Constraints(0.0, 0.0), period);
        this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-3.141592653589793, Math.PI);
        this.maxModuleSpeed = maxModuleSpeed;
        this.mpsToRps = 1.0 / driveBaseRadius;
        this.accelFF = accelFF;
    }


    /**
     * @param currentPose           the current pose of the robot
     * @param targetState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        // double xFF = targetState.linearVelocity * targetState.pose.getRotation().getCos();
        // double yFF = targetState.linearVelocity * targetState.pose.getRotation().getSin();
        // double xFFAccel = targetState.accelerationMpsSq * targetState.pose.getRotation().getCos() * accelFF;
        // double yFFAccel = targetState.accelerationMpsSq * targetState.pose.getRotation().getSin() * accelFF;
        // double xFeedback = this.xController.calculate(currentPose.getX(), targetState.positionMeters.getX());
        // double yFeedback = this.yController.calculate(currentPose.getY(), targetState.positionMeters.getY());
        // double angVelConstraint = targetState.constraints.getMaxAngularVelocityRps();
        // double maxAngVelModule = Math.max(0.0, this.maxModuleSpeed - targetState.velocityMps) * this.mpsToRps;
        // double maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
        // TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngVel, targetState.constraints.getMaxAngularAccelerationRpsSq());
        // double targetRotationVel = this.rotationController.calculate(currentPose.getRotation().getRadians(), new TrapezoidProfile.State(targetState.targetHolonomicRotation.getRadians(), 0.0), rotationConstraints);

        // return new ChassisSpeeds(xFFAccel + xFF + xFeedback, yFFAccel + yFF + yFeedback, targetRotationVel);
        return new ChassisSpeeds();
    }

    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        this.rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    public Translation2d getTranslationError(){
        return new Translation2d(this.xController.getPositionError(), this.yController.getPositionError()); 
    }

    public Rotation2d getRotationError(){
        return Rotation2d.fromRadians(this.rotationController.getPositionError());
    }
}