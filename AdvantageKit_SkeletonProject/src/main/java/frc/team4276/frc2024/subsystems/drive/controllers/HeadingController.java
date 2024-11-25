package frc.team4276.frc2024.subsystems.drive.controllers;

import frc.team4276.frc2024.subsystems.drive.DriveConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;

public class HeadingController {
    private PIDController mController;

    private DoubleSupplier mTargetHeadingSupplier;

    public HeadingController(){
        mController = new PIDController(
            DriveConstants.kSnapHeadingKp,
            DriveConstants.kSnapHeadingKi, 
            DriveConstants.kSnapHeadingKd);

        mController.setTolerance(DriveConstants.kSnapPositionTolerance);
        mController.enableContinuousInput(-Math.PI, Math.PI);

        mTargetHeadingSupplier = () -> 0.0;
    }

    public void setTarget(DoubleSupplier headingSupplier){
        mTargetHeadingSupplier = headingSupplier;
    }
    
    public double update(double headingRadians) {
        double output = mController.calculate(headingRadians, mTargetHeadingSupplier.getAsDouble());

        return output;

    }

}
