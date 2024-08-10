// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;

import frc.team1678.lib.swerve.ModuleState;

public class MAXSwerveModuleV3 extends Subsystem {
    private final VIKCANSparkMax mDrive;
    private final VIKCANSparkMax mTurn;

    private final RelativeEncoder mDriveEncoder;
    private final AbsoluteEncoder mTurnEncoder;

    private PeriodicIO mPeriodicIO;

    public static class MAXSwerveModuleConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";
        public int kDriveId = -1;
        public int kTurnId = -1;
        public double kOffset = 0.0;
    }

    public MAXSwerveModuleV3(MAXSwerveModuleConstants constants) {
        mDrive = CANSparkMaxFactory.createDefault(constants.kDriveId);
        mTurn = CANSparkMaxFactory.createDefault(constants.kTurnId);

        CANSparkMaxFactory.configAbsoluteEncoder(mTurn);

        mDriveEncoder = mDrive.getEncoder();
        mTurnEncoder = mTurn.getAbsoluteEncoder(Type.kDutyCycle);
        mDrive.getPIDController().setFeedbackDevice(mDriveEncoder);
        mTurn.getPIDController().setFeedbackDevice(mTurnEncoder);

        mDriveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor);
        mDriveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

        mTurnEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);
        mTurnEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor);

        mTurnEncoder.setInverted(Constants.ModuleConstants.kTurningEncoderInverted);
        mTurnEncoder.setZeroOffset(constants.kOffset);

        mTurn.getPIDController().setPositionPIDWrappingEnabled(true);
        mTurn.getPIDController()
                .setPositionPIDWrappingMinInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
        mTurn.getPIDController()
                .setPositionPIDWrappingMaxInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        CANSparkMaxFactory.configPIDF(mDrive, Constants.ModuleConstants.kDrivingPIDFConfig);
        CANSparkMaxFactory.configPIDF(mTurn, Constants.ModuleConstants.kTurningPIDFConfig);

        mDrive.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);
        mTurn.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);
        mDrive.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);
        mTurn.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

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

        ModuleState newState = ModuleState.optimize(desiredState.angle, ModuleState
                .fromSpeeds(Rotation2d.fromRadians(mPeriodicIO.turnPosition), desiredState.speedMetersPerSecond));

        mPeriodicIO.driveDemand = newState.speedMetersPerSecond;
        mPeriodicIO.rotationDemand = newState.angle.getRadians();
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
                new Rotation2d(mPeriodicIO.turnPosition),
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
}
