// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team1678.lib.swerve.ModuleState;
import frc.team254.lib.util.Util;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.Constants.ModuleConstants;
import frc.team4276.frc2024.subsystems.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class MAXSwerveModuleV2 extends Subsystem {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private ModuleState m_desiredState;

  public mPeriodicIO mPeriodicIO = new mPeriodicIO();

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModuleV2(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

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

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
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

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState = new ModuleState();
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
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
      new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset),
      m_drivingEncoder.getVelocity());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(ModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
      stop();
      return;
  
    } else {
      // Apply chassis angular offset to the desired state.
      ModuleState optimizedDesiredState = new ModuleState();

      optimizedDesiredState.speedMetersPerSecond = Util.limit(desiredState.speedMetersPerSecond, DriveConstants.kMaxVel);
      optimizedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

      double targetAngle =  optimizedDesiredState.angle.getDegrees();
      
      if (Util.shouldReverse(
        new frc.team254.lib.geometry.Rotation2d(targetAngle), 
        new frc.team254.lib.geometry.Rotation2d(Math.toDegrees(m_turningEncoder.getPosition())))) {
        optimizedDesiredState.speedMetersPerSecond *= -1;
        optimizedDesiredState.angle =  new Rotation2d(optimizedDesiredState.angle.getRadians() + Math.PI);
      }

      optimizedDesiredState.angle = new Rotation2d(Math.toRadians(Util.placeInAppropriate0To360Scope(
        Math.toDegrees(m_turningEncoder.getPosition()), optimizedDesiredState.angle.getDegrees())));

      // Optimize the reference state to avoid spinning further than 90 degrees.
      //ModuleState optimizedDesiredState = ModuleState.optimize(correctedDesiredState.angle, getState());

      driveSetpoint = optimizedDesiredState.speedMetersPerSecond;
      turnSetpoint = optimizedDesiredState.angle.getRadians();

      // Command driving and turning SPARKS MAX towards their respective setpoints.
      m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
      m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

      m_desiredState = desiredState;
    }
  }

  private double driveSetpoint;
  private double turnSetpoint;

  public double getDriveSetpoint(){
    return driveSetpoint;
  }

  public double getTurnSetpoint(){
    return turnSetpoint;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void stop(){
    m_drivingSparkMax.set(0);
    m_turningSparkMax.set(0);

  }

  

  public static class mPeriodicIO {
    // Inputs
    public double timestamp = 0.0;
    public double targetVelocity = 0.0;
    public double rotationPosition = 0.0;
    public double drivePosition = 0.0;
    public double velocity = 0.0;

    // Outputs
    public double rotationDemand;
    public double driveDemand;
  }

  
  // @Override
  // public synchronized void readPeriodicInputs() {

  //     mPeriodicIO.timestamp = Timer.getFPGATimestamp();

  //     mPeriodicIO.velocity = m_drivingEncoder.getVelocity();
      
  //     mPeriodicIO.rotationPosition = Math.toDegrees(m_turningEncoder.getPosition());

  //     mPeriodicIO.drivePosition = m_drivingEncoder.getPosition();
  // }

  // @Override
  // public synchronized void writePeriodicOutputs() {

  //     double targetAngle = m_desiredState.angle.getDegrees();
  //     Rotation2d currentAngle = Rotation2d.fromDegrees(mPeriodicIO.rotationPosition);
  //     if (Util.shouldReverse(Rotation2d.fromDegrees(targetAngle), currentAngle)) {
  //         mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
  //         targetAngle += 180.0;
  //     }
  //     targetAngle = Util.placeInAppropriate0To360Scope(getCurrentUnboundedDegrees(), targetAngle);

  //     mPeriodicIO.rotationDemand = Conversions.degreesToFalcon(targetAngle,
  //             Constants.SwerveConstants.angleGearRatio);

  //     mAngleMotor.set(ControlMode.Position, mPeriodicIO.rotationDemand);
  //     if (mPeriodicIO.driveControlMode == ControlMode.Velocity) {
  //         mDriveMotor.setControl(new VelocityTorqueCurrentFOC(mPeriodicIO.driveDemand));
  //     } else {
  //         mDriveMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand, true, false));
  //     }
  // }


  
}
