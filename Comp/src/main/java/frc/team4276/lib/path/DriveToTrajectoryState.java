package frc.team4276.lib.path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.config.PIDConstants;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.TrajectorySample;



// Credit: 4481 (2024)

/**
 * Can be used to aim a robot to a selected point. The translation will not be affected.
 */
public class DriveToTrajectoryState{

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final double translationAccelFF;
    private final double rotationAccelFF;

    private TrajectorySample<SwerveSample> prevTargetState;

    public DriveToTrajectoryState(PIDConstants translationConstants, PIDConstants rotationConstants, double period, double translationAccelFF, double rotationAccelFF) {
        this.xController = new PIDController(translationConstants.kP, 0.0, translationConstants.kD, period);
        this.yController = new PIDController(translationConstants.kP, 0.0, translationConstants.kD, period);
        this.rotationController = new PIDController(rotationConstants.kP, 0.0, rotationConstants.kD, period);
        this.rotationController.enableContinuousInput(-3.141592653589793, Math.PI);
        this.translationAccelFF = translationAccelFF;
        this.rotationAccelFF = rotationAccelFF;
    }


    /**
     * @param currentPose           the current pose of the robot
     * @param targetState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, SwerveSample targetState) {
        if(prevTargetState == null) {
            prevTargetState = targetState;
        }

        double xFF = targetState.vx;
        double yFF = targetState.vy;
        double xFFAccel = targetState.ax * translationAccelFF;
        double yFFAccel = targetState.ay * translationAccelFF;
        double xFeedback = this.xController.calculate(currentPose.getX(), targetState.x);
        double yFeedback = this.yController.calculate(currentPose.getY(), targetState.y);
        double thetaFF = targetState.omega;
        double thetaFFAccel = targetState.alpha * rotationAccelFF;
        double thetaFeedback = this.rotationController.calculate(currentPose.getRotation().getRadians(), targetState.heading);

        return new ChassisSpeeds(xFFAccel + xFF + xFeedback, yFFAccel + yFF + yFeedback, thetaFFAccel + thetaFF + thetaFeedback);
    }

    public void reset() {
        prevTargetState = null;
    }

    public Translation2d getTranslationError(){
        return new Translation2d(this.xController.getPositionError(), this.yController.getPositionError()); 
    }

    public Rotation2d getRotationError(){
        return Rotation2d.fromRadians(this.rotationController.getPositionError());
    }
}