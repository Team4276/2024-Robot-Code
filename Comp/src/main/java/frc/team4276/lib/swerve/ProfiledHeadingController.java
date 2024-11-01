package frc.team4276.lib.swerve;

import edu.wpi.first.wpilibj.Timer;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.motion.TrapezoidProfile;

import frc.team254.lib.util.SynchronousPIDF;

public class ProfiledHeadingController {
    private SynchronousPIDF mPIDF;
    private TrapezoidProfile mProfile;

    private double[] mTargetState = {Double.NaN, 0.0};
    private double[] mStateSetpoint = {Double.NaN, Double.NaN};

    private double lastUpdatedTimestamp;

    private static ProfiledHeadingController mInstance;

    public static ProfiledHeadingController getInstance(){
        if(mInstance == null){
            mInstance = new ProfiledHeadingController();
        }

        return mInstance;
    }

    private ProfiledHeadingController(){
        mPIDF = new SynchronousPIDF(
            DriveConstants.kProfiledSnapHeadingKp,
            DriveConstants.kProfiledSnapHeadingKi, 
            DriveConstants.kProfiledSnapHeadingKd);

        mPIDF.setInputRange(-Math.PI, Math.PI);
        mPIDF.setContinuous();
        mPIDF.setDeadband(DriveConstants.kProfiledSnapPositionTolerance);

        mPIDF.setOutputRange(-DriveConstants.kMaxAngularVel, DriveConstants.kMaxAngularVel);

        mProfile = new TrapezoidProfile(DriveConstants.kMaxAngularVel, DriveConstants.kMaxAngularAccel);

        mTargetState[0] = 0.0;
        lastUpdatedTimestamp = Timer.getFPGATimestamp();
    }

    public void reset(double headingRadians, double omegaRadsPerSecond){
        mStateSetpoint[0] = headingRadians;
        mStateSetpoint[1] = omegaRadsPerSecond;
    }

    public void setTarget(double angle_rad, double headingRadians, double omegaRadsPerSecond){
        mTargetState[0] = angle_rad;
        reset(headingRadians, omegaRadsPerSecond); //TODO: fix
    }

    public double getTargetRad(){
        return mTargetState[0];
    }
    
    public double update(double headingRadians, double timestammp) {
        mStateSetpoint = mProfile.calculate(Constants.kLooperDt, mStateSetpoint, mTargetState);

        double output = mPIDF.calculate(headingRadians - mStateSetpoint[0], timestammp - lastUpdatedTimestamp);

        lastUpdatedTimestamp = timestammp;

        return output;

    }

}
