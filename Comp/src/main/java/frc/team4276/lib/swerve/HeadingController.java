package frc.team4276.lib.swerve;

import edu.wpi.first.wpilibj.Timer;

import frc.team4276.frc2024.Constants.DriveConstants;

import frc.team254.lib.util.SynchronousPIDF;

public class HeadingController {
    private SynchronousPIDF mPIDF;

    private double mTargetHeading;

    private double lastUpdatedTimestamp;

    private static HeadingController mInstance;

    public static HeadingController getInstance(){
        if(mInstance == null){
            mInstance = new HeadingController();
        }

        return mInstance;
    }

    private HeadingController(){
        mPIDF = new SynchronousPIDF(
            DriveConstants.kSnapHeadingKp,
            DriveConstants.kSnapHeadingKi, 
            DriveConstants.kSnapHeadingKd);

        mPIDF.setInputRange(-Math.PI, Math.PI);
        mPIDF.setContinuous();
        mPIDF.setDeadband(DriveConstants.kSnapPositionTolerance);

        mPIDF.setOutputRange(-DriveConstants.kMaxAngularVel, DriveConstants.kMaxAngularVel);

        mTargetHeading = 0.0;
        lastUpdatedTimestamp = Timer.getFPGATimestamp();
    }

    public void setTarget(double angle_rad){
        mTargetHeading = angle_rad;
    }

    public double getTargetRad(){
        return mTargetHeading;
    }
    
    public double update(double headingRadians, double timestammp) {
        double output = mPIDF.calculate(headingRadians - mTargetHeading, timestammp - lastUpdatedTimestamp);

        lastUpdatedTimestamp = timestammp;

        return output;

    }

}
