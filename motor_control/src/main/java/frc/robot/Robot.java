// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.PIDController;
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

  private TalonSRX motor775Pro;
  private int canID_775Pro = 2;

  private final int kSensorCountsPerRotation_VersaPlanetaryEncoder = 1024;  
  private final int kGearReduction_ToOne = 3;
  private final double kGoButtonPercentPower = 0.1;

  public double rpmSetPoint_outoutShaft_RPM = 100.0;  
  public double rpmSetPoint_motor_RPM = rpmSetPoint_outoutShaft_RPM * kGearReduction_ToOne;  
  public double setPoint_motor_CountsPer100ms = (rpmSetPoint_motor_RPM * kSensorCountsPerRotation_VersaPlanetaryEncoder) / (60.0*10.0);  

  private final int kLoopIndex = 0;
  private final int kTimeoutMs = 30;

  private final double kP = 0.085;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kF = 0.0;  // Feed Forward gain - always set to zero for this test
  
  private final int DIO_GO_BUTTON = 0;
  private final int DIO_USE_PID_JUMPER = 5;
  private final int DIO_USE_PID_BRAKE = 6; // Normally brake mode, this will use PID to set RPM to zero - much faster
                                           // stop
  private final int MAX_RPM = 1000;

  private double rpmMeasured_outputShaft = 0.0;

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
    motor775Pro.setSensorPhase(false);

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

    goButton = new DigitalInput(DIO_GO_BUTTON);
    usePidSwitch = new DigitalInput(DIO_USE_PID_JUMPER);
  }

  @Override
  public void robotPeriodic() {

    double rawSensorUnitsPer100ms = motor775Pro.getSelectedSensorVelocity();
    rpmMeasured_outputShaft = (rawSensorUnitsPer100ms * (10.0/kSensorCountsPerRotation_VersaPlanetaryEncoder))  * 60.0;  // Motor RPM
    rpmMeasured_outputShaft /= kGearReduction_ToOne;  // Output RPM
    rpmMeasured_outputShaft *= -1.0;  // So positive input power makes a positive velocity
    
    SmartDashboard.putNumber("rpmSetPoint", rpmSetPoint_outoutShaft_RPM);
    SmartDashboard.putNumber("rpmMeasured", rpmMeasured_outputShaft);

    if (rpmMeasured_outputShaft > MAX_RPM) {
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
          motor775Pro.set(ControlMode.Velocity, setPoint_motor_CountsPer100ms);
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
