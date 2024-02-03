package frc.team4276.frc2024.planners;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.geometry.Twist2d;
import frc.team4276.frc2024.Constants.AutoAlignConstants;
import frc.team4276.lib.motion.ProfileFollower;

public class AutoAlignPlanner {
    private TrapezoidProfile mXProfile;
    private TrapezoidProfile mYProfile;
    private TrapezoidProfile mThetaProfile;
    private Constraints mXConstraints;
    private Constraints mYConstraints;
    private Constraints mThetaConstraints;

    private ProfileFollower mXController;
    private ProfileFollower mYController;
    private ProfileFollower mThetaController;

    private State goalX;
    private State goalY;
    private State goalTheta;

    private State prevGoalStateX;
    private State prevGoalStateY;
    private State prevGoalStateTheta;

    private boolean atGoal = false;
    
    public AutoAlignPlanner(){
        mXConstraints = new Constraints(AutoAlignConstants.kMaxTransVel, AutoAlignConstants.kMaxTransAccel);
        mYConstraints = new Constraints(AutoAlignConstants.kMaxTransVel, AutoAlignConstants.kMaxTransAccel);
        mThetaConstraints = new Constraints(AutoAlignConstants.kMaxThetaVel, AutoAlignConstants.kMaxThetaAccel);

        mXProfile = new TrapezoidProfile(mXConstraints);
        mYProfile = new TrapezoidProfile(mYConstraints);
        mThetaProfile = new TrapezoidProfile(mThetaConstraints);

        mXController = new ProfileFollower(AutoAlignConstants.kTranslationConstants);
        mYController = new ProfileFollower(AutoAlignConstants.kTranslationConstants);
        mThetaController = new ProfileFollower(AutoAlignConstants.kThetaConstants);
        

    }

    //TODO: write another class for the motion profile itself nvm hold off on that idk anymore
    //TODO: fix this thing bc its stupid

    public ChassisSpeeds update(double timeStamp, Pose2d currentPose, Twist2d currentVel){
        if (atGoal){
            return new ChassisSpeeds();
        }

        State curr_state_x = new State(currentPose.getTranslation().x(), currentVel.dx);
        State curr_state_y = new State(currentPose.getTranslation().y(), currentVel.dy);
        State curr_state_theta = new State(currentPose.getRotation().getRadians(), currentVel.dtheta);

        State XState = mXProfile.calculate(timeStamp, curr_state_x, goalX);
        State YState = mYProfile.calculate(timeStamp, curr_state_y, goalY);
        State ThetaState = mThetaProfile.calculate(timeStamp, curr_state_theta, goalTheta);

        Translation2d des_translation = new Translation2d(XState.position, YState.position);

        if (des_translation.distance(currentPose.getTranslation()) > AutoAlignConstants.kTranslationTolerance){
            
        }



        double dx = mXController.calculate(XState, curr_state_x);
        double dy = mYController.calculate(YState, curr_state_y);
        double dtheta = mThetaController.calculate(ThetaState, curr_state_theta);

        return new ChassisSpeeds(dx, dy, dtheta);
    }

    public void reset(){
        atGoal = false;
    }

    public synchronized void setAlignment(Pose2d goal){
        if (Math.abs(Math.hypot(goalX.position, goalY.position) - goal.getTranslation().norm()) > AutoAlignConstants.kTranslationTolerance ||
                Math.abs(goalTheta.position - goal.getRotation().getRadians()) > AutoAlignConstants.kThetaTolerance){
            goalX = new State(goal.getTranslation().x(), 0);
            goalY = new State(goal.getTranslation().y(), 0);
            goalTheta = new State(goal.getRotation().getRadians(), 0);
        }
    }

    public boolean atGoal(){
        return atGoal;
    }


}
