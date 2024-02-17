// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib.drivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.team254.lib.util.Util;

import frc.team1678.lib.swerve.ModuleState;

import frc.team4276.frc2024.Constants.ModuleConstants;
import frc.team4276.frc2024.Constants.DebugConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.REVLibError;

public class MAXSwerveModuleV2 extends Subsystem {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private final int kDrivingCANId;
  private final int kTurningCANId;

  //TODO: look into sampling depth
  //TODO: look into error initiallization
  //TODO: look into peridic frame period
  //TODO: look into control frame period
  //TODO: look into CANTimeout

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModuleV2(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    kDrivingCANId = drivingCANId;
    kTurningCANId = turningCANId;

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setZeroOffset(chassisAngularOffset);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public ModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new ModuleState(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()),
        m_drivingEncoder.getVelocity());
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

    double speed = desiredState.speedMetersPerSecond;
    Rotation2d angle = desiredState.angle;

    if (Util.shouldReverse(frc.team254.lib.geometry.Rotation2d.fromWPI(desiredState.angle),
      frc.team254.lib.geometry.Rotation2d.fromRadians(m_turningEncoder.getPosition()))) {
      speed *= -1;
      angle.rotateBy(new Rotation2d(Math.PI));
    }

    if (DebugConstants.writeSwerveErrors) {
      double timestamp = Timer.getFPGATimestamp();
      REVLibError e1 = m_drivingPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
      REVLibError e2 = m_turningPIDController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);

      checkMotorCommands(e1, e2, timestamp);
      return;
    }

    m_drivingPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  private void checkMotorCommands(REVLibError e1, REVLibError e2, double timestamp) {
    if(e1 == REVLibError.kOk && e2 == REVLibError.kOk){
      return;
    }

    System.out.println("Driving Motor ID:" + kDrivingCANId + 
      " returned " + e1.toString() + " at " + timestamp);
    System.out.println("Turning Motor ID:" + kTurningCANId + 
      " returned " + e2.toString() + " at " + timestamp);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void stop() {
    m_drivingSparkMax.set(0);
    m_turningSparkMax.set(0);
  }
}
