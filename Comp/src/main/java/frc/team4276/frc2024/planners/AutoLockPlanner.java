package frc.team4276.frc2024.planners;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.AutoAlignConstants;

import frc.team254.lib.geometry.Pose2d;

import frc.team1678.lib.swerve.ChassisSpeeds;

public class AutoLockPlanner {
    /** key: metres; values: degrees */
    private static InterpolatingDoubleTreeMap fourbarAngleMap;

    static {
        fourbarAngleMap.put(1.05, 45.0);
    }

    private static AutoLockPlanner mInstance;

    public static AutoLockPlanner getInstance(){
        if(mInstance == null){
            mInstance = new AutoLockPlanner();
        }
        return mInstance;
    }

    private AutoLockPlanner(){
        fourbarAngleMap = new InterpolatingDoubleTreeMap();
    }

    public double[] update(Pose2d currentPose, ChassisSpeeds currentVel){
        double dtheta = 0.0;
        double fourbar_angle = calcDynamicFourbarAngle(getSpeakerDistance(currentPose));

        double[] output = {
            dtheta,
            fourbar_angle
        };

        return output;
    }

    
    public double getSpeakerDistance(Pose2d currentPose){
        return currentPose.getTranslation().distance(RobotState.getInstance().getPOIs().kSpeakerCenter);
    }

    public boolean isValidSpeakerDistance(double distance){
        return distance <= AutoAlignConstants.kValidSpeakerDistance;
    }

    /** 
     * @return fourbar angle in radians
     */
    public double calcDynamicFourbarAngle(double distance){
        return Math.toRadians(fourbarAngleMap.get(distance));
    }
    
}
