// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

  public final int MAX_RPM = 1000;
  public double rpmSetPoint_flywheel_RPM = 600.0;
  public final double kGoButtonPercentPower = 0.5;

  private TalonSRX motor775Pro;
  private int canID_775Pro = 2;

  private final int kSensorCountsPerRotation_VersaPlanetaryEncoder = 1024;
  private final int kGearReduction_ToOne = 3;

  public double setPoint_rawSensorUnitsPer100ms = 0.0;

  private final int kLoopIndex = 0;
  private final int kTimeoutMs = 30;

  private final double kP = 0.4;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0; // Feed Forward gain - always set to zero for this test

  private final int DIO_GO_BUTTON = 0;
  private final int DIO_USE_PID_JUMPER = 5;
  private final int DIO_USE_PID_BRAKE = 6; // Normally brake mode, this will use PID to set RPM to zero - much faster
                                           // stop
  private DigitalInput goButton;
  private DigitalInput usePidSwitch;
  private DigitalInput pidBrakingSwitch;

  private boolean safetySpinDown = false; // Gets set TRUE if max RPM is exceeded, resets to false when 'GO' button is
                                          // released
  private double rpmFlywheel = 0.0;

  private double rawUnitsPer100ms_to_rpmFlywheel(double rawUnits) {
    double revPer100ms = rawUnits / (4 * kSensorCountsPerRotation_VersaPlanetaryEncoder); // *4 Because quad encoder
                                                                                          // counts all the edges
    double rpmMotor = revPer100ms * (10.0 * 60.0);
    return rpmMotor / kGearReduction_ToOne; // Output RPM
  }

  private double rpmFlywheel_to_rawUnitsPer100ms(double rpm) {
    double rpmMotor = rpm * kGearReduction_ToOne;
    double revPer100ms = rpmMotor / (10.0 * 60.0);
    return revPer100ms * (4 * kSensorCountsPerRotation_VersaPlanetaryEncoder);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    setPoint_rawSensorUnitsPer100ms = rpmFlywheel_to_rawUnitsPer100ms(rpmSetPoint_flywheel_RPM);

    motor775Pro = new TalonSRX(canID_775Pro);

    /* Factory Default all hardware to prevent unexpected behaviour */
    motor775Pro.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    motor775Pro.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
        kLoopIndex,
        kTimeoutMs);
    /**
     * Phase sensor accordingly.
     * Positive Sensor Reading should match Green (blinking) Leds on Talon
     */
    motor775Pro.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    motor775Pro.configNominalOutputForward(0, kTimeoutMs);
    motor775Pro.configNominalOutputReverse(0, kTimeoutMs);
    motor775Pro.configPeakOutputForward(1, kTimeoutMs);
    motor775Pro.configPeakOutputReverse(-1, kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    motor775Pro.config_kF(kLoopIndex, kF, kTimeoutMs);
    motor775Pro.config_kP(kLoopIndex, kP, kTimeoutMs);
    motor775Pro.config_kI(kLoopIndex, kI, kTimeoutMs);
    motor775Pro.config_kD(kLoopIndex, kD, kTimeoutMs);

    motor775Pro.setNeutralMode(NeutralMode.Brake);

    goButton = new DigitalInput(DIO_GO_BUTTON);
    usePidSwitch = new DigitalInput(DIO_USE_PID_JUMPER);
    pidBrakingSwitch = new DigitalInput(DIO_USE_PID_BRAKE);
  }

  @Override
  public void robotPeriodic() {

    double rawSensorUnitsPer100ms = motor775Pro.getSelectedSensorVelocity();
    rpmFlywheel = rawUnitsPer100ms_to_rpmFlywheel(rawSensorUnitsPer100ms);

    SmartDashboard.putNumber("rpmSetPoint", rpmSetPoint_flywheel_RPM);
    SmartDashboard.putNumber("rpmMeasured", rpmFlywheel);

    if (rpmFlywheel > MAX_RPM) {
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

  private boolean previousGoButton = false;

  @Override
  public void teleopPeriodic() {
    if (safetySpinDown) {
      motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
    } else {
      if (goButton.get()) {
        if (pidBrakingSwitch.get()) {
          motor775Pro.set(ControlMode.Velocity, 0.0);
        } else {
          motor775Pro.set(TalonSRXControlMode.PercentOutput, 0.0);
        }
      } else {
        if (usePidSwitch.get()) {
          if (!previousGoButton) {
            motor775Pro.set(ControlMode.Velocity, setPoint_rawSensorUnitsPer100ms);
          }
        } else {
          motor775Pro.set(TalonSRXControlMode.PercentOutput, kGoButtonPercentPower);
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
