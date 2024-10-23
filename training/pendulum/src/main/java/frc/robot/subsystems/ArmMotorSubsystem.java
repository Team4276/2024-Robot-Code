// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmMotorSubsystem extends SubsystemBase {

  private Talon m_775Motor;
  private PIDController m_pidController;

  // Velocity control when more than 45 degrees from goal
  private final double positionControlThresholdRadians = Math.PI / 4.0;

  // The 775 motor can run for a long time at 2.0V without overheating
  private final double velocityControlVoltageForDesiredSpeed = 2.0;
  private final double motor775_Volts = 12.0;


  public Encoder m_encoder;
  public double encoderPositionRadians = 0.0;
  public double encoderVelocityRadiansPerSecond = 0.0;
  public double desiredPositionRadians;

  public ArmMotorSubsystem() {
    m_775Motor = new Talon(0);

    double kp = 0.08;
    double ki = 0.02;  //01;
    double kd = 0.000002;  //01; //001;
    m_pidController = new PIDController(kp, ki, kd, .05);
    
    m_encoder = new Encoder(1, 2);
    m_encoder.setDistancePerPulse((2 * Math.PI) / 2048); // Distance units are radians
  }

  /**
   * Command to set the arm position, in degrees
   *
   * @return a command
   */
  public Command setArmPositionCommand(double posDegrees) {
    return runOnce(
        () -> {
          m_pidController.reset();
          desiredPositionRadians = posDegrees * ((2 * Math.PI) / 360.0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update encoder position and velocity
    // Zero degrees is arm position when code deploys, normally down
    encoderPositionRadians = (m_encoder.getDistance() % (2 * Math.PI));
    encoderVelocityRadiansPerSecond = m_encoder.getRate();

    SmartDashboard.putNumber("Position (degrees)", encoderPositionRadians * (360.0 / (2 * Math.PI)));
    SmartDashboard.putNumber("Desired Position (degrees)", desiredPositionRadians * (360.0 / (2 * Math.PI)));
    SmartDashboard.putNumber("Velocity (degrees/sec)", encoderVelocityRadiansPerSecond * (360.0 / (2 * Math.PI)));

    if (RobotState.isEnabled()) {

      double motorOutputMinusToPlusOne = 0.0;

      // First, find shortest distance to the goal allowing for wrap at zero
      // e.g 359 and zero degrees are not far apart
      double positionError = desiredPositionRadians - encoderPositionRadians;
      if (positionError > Math.PI) {
        positionError -= Math.PI;
        positionError *= -1.0;
      } else if (positionError < -1.0 * Math.PI) {
        positionError += Math.PI;
        positionError *= -1.0;
      }

      enum ControlMode {
        POSITION,
        VELOCITY_FORWARD,
        VELOCITY_REVERSE
      }
      ControlMode mode = ControlMode.POSITION;
      if (positionError > positionControlThresholdRadians) {
        mode = ControlMode.VELOCITY_FORWARD;
      } else if (positionError < (-1.0 * positionControlThresholdRadians)) {
        mode = ControlMode.VELOCITY_REVERSE;
      }

      switch (mode) {
        case POSITION:
          motorOutputMinusToPlusOne = m_pidController.calculate(encoderPositionRadians,
              encoderPositionRadians + positionError);
          break;

        case VELOCITY_FORWARD:
          motorOutputMinusToPlusOne = velocityControlVoltageForDesiredSpeed / motor775_Volts;
          break;

        case VELOCITY_REVERSE:
          motorOutputMinusToPlusOne = -1.0 * velocityControlVoltageForDesiredSpeed / motor775_Volts;
          break;

        default:
          motorOutputMinusToPlusOne = 0.0;
          break;
      }

      // Limit total power to 80%
      if (motorOutputMinusToPlusOne > 0.8) {
        motorOutputMinusToPlusOne = 0.8;
      } else if (motorOutputMinusToPlusOne < -0.8) {
        motorOutputMinusToPlusOne = -0.8;
      }
      SmartDashboard.putNumber("Motor Output (%)", motorOutputMinusToPlusOne);
      m_775Motor.set(motorOutputMinusToPlusOne);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
