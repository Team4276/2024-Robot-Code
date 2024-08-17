// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants.MaxSwerveModuleConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;

import frc.team1678.lib.swerve.ModuleState;

import frc.team254.lib.geometry.Rotation2d;

public class MAXSwerveModule extends Subsystem {
    private final VIKCANSparkMax mDrive;
    private final VIKCANSparkMax mTurn;

    private final RelativeEncoder mDriveEncoder;
    private final AbsoluteEncoder mTurnEncoder;

    private final PeriodicIO mPeriodicIO;

    private final MAXSwerveModuleConstants mConstants;

    public static class MAXSwerveModuleConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";
        public int kDriveId = -1;
        public int kTurnId = -1;
        public double kOffset = 0.0;
    }

    public MAXSwerveModule(MAXSwerveModuleConstants constants) {
        mConstants = constants;
        mDrive = CANSparkMaxFactory.createDefault(mConstants.kDriveId);
        mTurn = CANSparkMaxFactory.createDefault(mConstants.kTurnId);

        CANSparkMaxFactory.configAbsoluteEncoder(mTurn);

        mDriveEncoder = mDrive.getEncoder();
        mTurnEncoder = mTurn.getAbsoluteEncoder(Type.kDutyCycle);
        mDrive.getPIDController().setFeedbackDevice(mDriveEncoder);
        mTurn.getPIDController().setFeedbackDevice(mTurnEncoder);

        mDriveEncoder.setPositionConversionFactor(MaxSwerveModuleConstants.kDrivingEncoderPositionFactor);
        mDriveEncoder.setVelocityConversionFactor(MaxSwerveModuleConstants.kDrivingEncoderVelocityFactor);

        mTurnEncoder.setPositionConversionFactor(MaxSwerveModuleConstants.kTurningEncoderPositionFactor);
        mTurnEncoder.setVelocityConversionFactor(MaxSwerveModuleConstants.kTurningEncoderVelocityFactor);

        mTurnEncoder.setInverted(MaxSwerveModuleConstants.kTurningEncoderInverted);
        mTurnEncoder.setZeroOffset(mConstants.kOffset);

        mTurn.getPIDController().setPositionPIDWrappingEnabled(true);
        mTurn.getPIDController()
                .setPositionPIDWrappingMinInput(MaxSwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        mTurn.getPIDController()
                .setPositionPIDWrappingMaxInput(MaxSwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        CANSparkMaxFactory.configPIDF(mDrive, MaxSwerveModuleConstants.kDrivingPIDFConfig);
        CANSparkMaxFactory.configPIDF(mTurn, MaxSwerveModuleConstants.kTurningPIDFConfig);

        mDrive.setIdleMode(MaxSwerveModuleConstants.kDrivingMotorIdleMode);
        mTurn.setIdleMode(MaxSwerveModuleConstants.kTurningMotorIdleMode);
        mDrive.setSmartCurrentLimit(MaxSwerveModuleConstants.kDrivingMotorCurrentLimit);
        mTurn.setSmartCurrentLimit(MaxSwerveModuleConstants.kTurningMotorCurrentLimit);

        mDriveEncoder.setPosition(0);

        mDrive.burnFlash();
        mTurn.burnFlash();

        mPeriodicIO = new PeriodicIO();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && !isOpenLoop) {
            stop();
            return;
        }

        SmartDashboard.putNumber("Debug/Swerve/" + mConstants.kName + " Des Rotation", desiredState.angle.getDegrees());

        mPeriodicIO.driveDemand = desiredState.speedMetersPerSecond;

        final double targetClamped = Rotation2d.fromWPI(desiredState.angle).getDegrees();
        final double angleUnclamped = mPeriodicIO.turnPosition;
        final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
        final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(angleClamped.inverse());
        double relativeDegrees = relativeAngle.getDegrees();
        if (relativeDegrees > 90.0) {
            relativeDegrees -= 180.0;
            mPeriodicIO.driveDemand *= -1.0;

        } else if (relativeDegrees < -90.0) {
            relativeDegrees += 180.0;
            mPeriodicIO.driveDemand *= -1.0;
        }

        mPeriodicIO.rotationDemand = angleUnclamped + relativeDegrees;
            
    }

    public void stop() {
        mDrive.set(0);
        mTurn.set(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public ModuleState getState() {
        return new ModuleState(
                mPeriodicIO.drivePosition,
                Rotation2d.fromRadians(mPeriodicIO.turnPosition).toWPI(),
                mPeriodicIO.driveVelocity);
    }

    /** Zeroes SwerveModule drive encoder. */
    public void resetEncoders() {
        mDriveEncoder.setPosition(0);
    }

    private class PeriodicIO {
        // inputs
        double drivePosition = 0.0;
        double turnPosition = 0.0;

        double driveVelocity = 0.0;

        // outputs
        double driveDemand = 0.0;
        double rotationDemand = 0.0;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drivePosition = mDriveEncoder.getPosition();
        mPeriodicIO.turnPosition = mTurnEncoder.getPosition();

        mPeriodicIO.driveVelocity = mDriveEncoder.getVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        mDrive.setReference(mPeriodicIO.driveDemand, ControlType.kVelocity, 0, 0, ArbFFUnits.kVoltage);
        mTurn.setReference(mPeriodicIO.rotationDemand, ControlType.kPosition, 0, 0, ArbFFUnits.kVoltage);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Debug/Swerve/" + mConstants.kName + " Rotation Demand", mPeriodicIO.rotationDemand);
        SmartDashboard.putNumber("Debug/Swerve/" + mConstants.kName + " Turn Position", mPeriodicIO.turnPosition);
    }
}