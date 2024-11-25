package frc.team4276.frc2024.subsystems.drive.controllers;

import frc.team4276.frc2024.subsystems.drive.DriveConstants;

import edu.wpi.first.math.controller.PIDController;

public class HeadingController {
    private PIDController mController;

    private double mTargetHeading;

    public HeadingController(){
        mController = new PIDController(
            DriveConstants.kSnapHeadingKp,
            DriveConstants.kSnapHeadingKi, 
            DriveConstants.kSnapHeadingKd);

        mController.setTolerance(DriveConstants.kSnapPositionTolerance);
        mController.enableContinuousInput(-Math.PI, Math.PI);

        mTargetHeading = 0.0;
    }

    public void setTarget(double angle_rad){
        mTargetHeading = angle_rad;
    }

    public double getTargetRad(){
        return mTargetHeading;
    }
    
    public double update(double headingRadians) {
        double output = mController.calculate(headingRadians, mTargetHeading);

        return output;

    }

}
