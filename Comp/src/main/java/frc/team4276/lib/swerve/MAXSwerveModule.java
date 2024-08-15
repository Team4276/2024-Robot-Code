// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants.ModuleConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;

import frc.team1678.lib.swerve.ModuleState;
import frc.team1678.lib.Util;

import frc.team254.lib.geometry.Rotation2d;

public class MAXSwerveModule extends Subsystem {
    private final VIKCANSparkMax mDrive;
    private final VIKCANSparkMax mTurn;

    private final RelativeEncoder mDriveEncoder;
    private final AbsoluteEncoder mTurnEncoder;

    private PeriodicIO mPeriodicIO;

    private MAXSwerveModuleConstants mConstants;

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

        mDriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        mDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        mTurnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        mTurnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        mTurnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
        mTurnEncoder.setZeroOffset(mConstants.kOffset);

        mTurn.getPIDController().setPositionPIDWrappingEnabled(true);
        mTurn.getPIDController()
                .setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        mTurn.getPIDController()
                .setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        CANSparkMaxFactory.configPIDF(mDrive, ModuleConstants.kDrivingPIDFConfig);
        CANSparkMaxFactory.configPIDF(mTurn, ModuleConstants.kTurningPIDFConfig);

        mDrive.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        mTurn.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        mDrive.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        mTurn.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

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

        SmartDashboard.putNumber("Debug/" + mConstants.kName + " Des Rotation", desiredState.angle.getDegrees());

        // ModuleState newState = ModuleState.optimize(desiredState,
        //         Rotation2d.fromRadians(mPeriodicIO.turnPosition).toWPI());

        // double targetAngle = desiredState.angle.getDegrees();
        // if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle),
        // Rotation2d.fromRadians(mPeriodicIO.turnPosition))) {
        // mPeriodicIO.driveDemand = -desiredState.speedMetersPerSecond;
        // targetAngle += 180.0;
        // }

        // mPeriodicIO.rotationDemand =
        // Math.toRadians(Util.placeInAppropriate0To360Scope(Math.toDegrees(mPeriodicIO.turnPosition),
        // targetAngle));

        // SwerveModuleState state = SwerveModuleState.optimize(new
        // SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle),
        // Rotation2d.fromRadians(mPeriodicIO.turnPosition).toWPI());

        // Apply chassis angular offset to the desired state.
        // SwerveModuleState correctedDesiredState = new SwerveModuleState();
        // correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        // correctedDesiredState.angle = desiredState.angle;

        // // Optimize the reference state to avoid spinning further than 90 degrees.
        // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        //         Rotation2d.fromRadians(mPeriodicIO.turnPosition).toWPI());

        // mPeriodicIO.driveDemand = optimizedDesiredState.speedMetersPerSecond;
        // mPeriodicIO.turnPosition = optimizedDesiredState.angle.getRadians();

        // mPeriodicIO.driveDemand = newState.speedMetersPerSecond;
        // mPeriodicIO.rotationDemand = newState.angle.getRadians();

        ModuleS

        double targetAngle = desiredState.angle.getDegrees();

        if (Util.shouldReverse(Rotation2d.fromWPI(desiredState.angle), new Rotation2d(mPeriodicIO.turnPosition))) {
            optimizedDesiredState.speedMetersPerSecond *= -1;
            optimizedDesiredState.angle = new Rotation2d(optimizedDesiredState.angle.getRadians() + Math.PI);
        }

        optimizedDesiredState.angle = new Rotation2d(Math.toRadians(Util.placeInAppropriate0To360Scope(
                Math.toDegrees(m_turningEncoder.getPosition()), optimizedDesiredState.angle.getDegrees())));
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
                new Rotation2d(mPeriodicIO.turnPosition).toWPI(),
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
        SmartDashboard.putNumber("Debug/" + mConstants.kName + " Rotation Demand", mPeriodicIO.rotationDemand);
        SmartDashboard.putNumber("Debug/" + mConstants.kName + " Turn Position", mPeriodicIO.turnPosition);
    }
}
