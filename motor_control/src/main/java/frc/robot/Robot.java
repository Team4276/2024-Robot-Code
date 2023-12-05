// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public double rpmSetPoint = 0.0;

  private TalonSRX motor775Pro;

  private PIDController myPidController;
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final int kSensorUnitsPerRotation_775Pro = 12;

  private final int DIO_GO_BUTTON = 0;
  private final int DIO_USE_PID_JUMPER = 5;
  private final int DIO_USE_PID_BRAKE = 6; // Normally brake mode, this will use PID to set RPM to zero - much faster
                                           // stop

  private final int MAX_RPM = 120;

  private double rpmMeasured = 0.0;

  private DigitalInput goButton;
  private DigitalInput usePidSwitch;

  private boolean safetySpinDown = false; // Gets set TRUE if max RPM is exceeded, resets to false when 'GO' button is
                                          // released

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor775Pro = new TalonSRX(9);
    TalonSRXConfiguration myConfig = new TalonSRXConfiguration();
    motor775Pro.configAllSettings(myConfig); // set defaults

    motor775Pro.setNeutralMode(NeutralMode.Brake);

    myPidController = new PIDController(kP, kI, kD);
    goButton = new DigitalInput(DIO_GO_BUTTON);
    usePidSwitch = new DigitalInput(DIO_USE_PID_JUMPER);
  }

  @Override
  public void robotPeriodic() {

    double rawSensorUnitsPer100ms = motor775Pro.getSelectedSensorVelocity();
    rpmMeasured = (rawSensorUnitsPer100ms * (10.0/kSensorUnitsPerRotation_775Pro))  * 60.0;  // RPM

    SmartDashboard.putNumber("rpmSetPoint", rpmSetPoint);
    SmartDashboard.putNumber("rpmMeasured", rpmMeasured);

    if (rpmMeasured > MAX_RPM) {
      safetySpinDown = true;
    } else {
      if (goButton.get()) {
        safetySpinDown = false;
      }
    }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (safetySpinDown) {
      motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
    } else {
      if (goButton.get()) {
        motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
      } else {
        if (usePidSwitch.get()) {
          motor775Pro.set(TalonSRXControlMode.PercentOutput, myPidController.calculate(rpmMeasured, rpmSetPoint));
        } else {
          motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.8);
        }
      }
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
    motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
  }
}
