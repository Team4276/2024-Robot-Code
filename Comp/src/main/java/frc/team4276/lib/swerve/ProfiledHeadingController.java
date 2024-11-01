package frc.team4276.lib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.motion.TrapezoidProfile;

public class ProfiledHeadingController {
    private PIDController mController;
    private TrapezoidProfile mProfile;

    private double[] mTargetState = { Double.NaN, 0.0 };
    private double[] mStateSetpoint = { Double.NaN, Double.NaN };

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

        mProfile = new TrapezoidProfile(DriveConstants.kMaxAngularVel, DriveConstants.kMaxAngularAccel);

        mTargetState[0] = 0.0;
    }

    public void reset(double headingRadians, double omegaRadsPerSecond) {
        mStateSetpoint[0] = headingRadians;
        mStateSetpoint[1] = omegaRadsPerSecond;
    }

    private boolean wasTargeting = false;

    public void setTarget(double angle_rad, double headingRadians, double omegaRadsPerSecond) {
        if ((DriveSubsystem.getInstance().getDriveControlState() == DriveSubsystem.DriveControlState.HEADING_CONTROL
                && !wasTargeting) || DriverStation.isDisabled()) {
            reset(headingRadians, omegaRadsPerSecond);
        }

        mTargetState[0] = angle_rad;
    }

    public double getTargetRad() {
        return mTargetState[0];
    }

    public double update(double headingRadians, double timestammp) {
        mStateSetpoint = mProfile.calculate(Constants.kLooperDt, mStateSetpoint, mTargetState);

        double output = mController.calculate(headingRadians, mStateSetpoint[0]) * DriveConstants.kMaxAngularVel;

        return output;

    }

}
