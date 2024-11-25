package frc.team4276.frc2024.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.subsystems.drive.DriveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfiledHeadingController {
    private PIDController mController;
    private TrapezoidProfile mProfile;

    private TrapezoidProfile.State mTargetState = new TrapezoidProfile.State(Double.NaN, 0.0);
    private TrapezoidProfile.State mStateSetpoint = new TrapezoidProfile.State(Double.NaN, Double.NaN);

    private static ProfiledHeadingController mInstance;

    public static ProfiledHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new ProfiledHeadingController();
        }

        return mInstance;
    }

    private ProfiledHeadingController() {
        mController = new PIDController(
                DriveConstants.kProfiledSnapHeadingKp,
                DriveConstants.kProfiledSnapHeadingKi,
                DriveConstants.kProfiledSnapHeadingKd,
                Constants.kLooperDt);

        mController.enableContinuousInput(-Math.PI, Math.PI);
        mController.setTolerance(DriveConstants.kProfiledSnapPositionTolerance);
        mController.setIZone(Math.toRadians(1.0));
        mController.setIntegratorRange(0.01, 0.01);

        mProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularVel, DriveConstants.kMaxAngularAccel));

        mTargetState.position = 0.0;
    }

    public void reset(double headingRadians, double omegaRadsPerSecond) {
        mStateSetpoint.position = headingRadians;
        mStateSetpoint.velocity = omegaRadsPerSecond;
    }

    public void setTarget(double angle_rad, double headingRadians, double omegaRadsPerSecond, boolean wasNotHeadingControl) {
        if (wasNotHeadingControl || DriverStation.isDisabled()) {
            reset(headingRadians, omegaRadsPerSecond);
        }

        mTargetState.position = angle_rad;
    }

    public double getTargetRad() {
        return mTargetState.position;
    }

    public double update(double headingRadians, double timestammp) {
        mStateSetpoint = mProfile.calculate(Constants.kLooperDt, mStateSetpoint, mTargetState);

        double output = mController.calculate(headingRadians, mStateSetpoint.position) * DriveConstants.kMaxAngularVel;

        return output;

    }

}
