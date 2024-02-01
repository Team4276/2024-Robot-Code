package frc.team4276.frc2024.planners;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Twist2d;

import frc.team4276.frc2024.Constants.AutoAlignConstants;

public class AutoAlignPlanner {
    private TrapezoidProfile mXProfile;
    private TrapezoidProfile mYProfile;
    private TrapezoidProfile mThetaProfile;
    private Constraints mXConstraints;
    private Constraints mYConstraints;
    private Constraints mThetaConstraints;

    private PIDController mXController;
    private PIDController mYController;
    private PIDController mThetaController;

    private State goalX;
    private State goalY;
    private State goalTheta;

    private State prevGoalX;
    private State prevGoalY;
    private State prevGoalTheta;

    private boolean atGoal = false;
    
    public AutoAlignPlanner(){
        mXConstraints = new Constraints(AutoAlignConstants.kMaxTransVel, AutoAlignConstants.kMaxTransAccel);
        mYConstraints = new Constraints(AutoAlignConstants.kMaxTransVel, AutoAlignConstants.kMaxTransAccel);
        mThetaConstraints = new Constraints(AutoAlignConstants.kMaxThetaVel, AutoAlignConstants.kMaxThetaAccel);

        mXProfile = new TrapezoidProfile(mXConstraints);
        mYProfile = new TrapezoidProfile(mYConstraints);
        mThetaProfile = new TrapezoidProfile(mThetaConstraints);

        mXController = new PIDController(AutoAlignConstants.kTranslationP, AutoAlignConstants.kTranslationI, AutoAlignConstants.kTranslationD);
        mYController = new PIDController(AutoAlignConstants.kTranslationP, AutoAlignConstants.kTranslationI, AutoAlignConstants.kTranslationD);
        mThetaController = new PIDController(AutoAlignConstants.kThetaP, AutoAlignConstants.kThetaI, AutoAlignConstants.kThetaD);

    }

    public ChassisSpeeds update(double timeStamp, Pose2d currentPose, Twist2d currentVel){
        if (atGoal){
            return new ChassisSpeeds();
        }

        State XState = mXProfile.calculate(timeStamp, new State(currentPose.getTranslation().x(), currentVel.dx), goalX);
        State YState = mYProfile.calculate(timeStamp, new State(currentPose.getTranslation().y(), currentVel.dy), goalY);
        State ThetaState = mThetaProfile.calculate(timeStamp, new State(currentPose.getRotation().getRadians(), currentVel.dtheta), goalTheta);

        return new ChassisSpeeds();
    }

    public void reset(){
        atGoal = false;
    }

    public void setAlignment(Pose2d goal){
        if (Math.abs(Math.hypot(goalX.position, goalY.position) - goal.getTranslation().norm()) > AutoAlignConstants.kTranslationTolerance ||
                Math.abs(goalTheta.position - goal.getRotation().getRadians()) > AutoAlignConstants.kThetaTolerance){
            goalX = new State(goal.getTranslation().x(), 0);
            goalY = new State(goal.getTranslation().y(), 0);
            goalTheta = new State(goal.getRotation().getRadians(), 0);
        }
    }
}
