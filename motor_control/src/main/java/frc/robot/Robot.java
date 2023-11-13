// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

public double rpmSetPoint = 0.0;

private VictorSP cimMotor;

private PIDController cimPidController;
private final double kP = 1.0;   
private final double kI = 0.0;   
private final double kD = 0.0;   

private final int DIO_HALL_ENCODER = 0;
private final int DIO_GO_BUTTON = 1;
private final int DIO_USE_PID_JUMPER = 2;

private Counter hallEncoder;
private final double NUMBER_OF_TEETH_ON_HALL_ENCODER_SPROCKET = 13;

private double rpmMeasured = 0.0;

private DigitalInput goButton;
private DigitalInput usePidJumper;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    cimMotor = new VictorSP(9);
    cimPidController = new PIDController(kP, kI, kD); 
    hallEncoder = new Counter(DIO_HALL_ENCODER);
    hallEncoder.setDistancePerPulse(360.0/NUMBER_OF_TEETH_ON_HALL_ENCODER_SPROCKET);  // Degrees
    hallEncoder.reset();
    goButton = new DigitalInput(DIO_GO_BUTTON);
    usePidJumper = new DigitalInput(DIO_USE_PID_JUMPER);
  }

  @Override
  public void robotPeriodic() {

    rpmMeasured = hallEncoder.getRate();  // Degrees/sec
     rpmMeasured *= 60.0;  // Degrees/min
    rpmMeasured /= 360.0;  // RPM

    SmartDashboard.putNumber("rpmSetPoint", rpmSetPoint);
    SmartDashboard.putNumber("rpmMeasured", rpmMeasured);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    cimMotor.set(0.0); 
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
     if(goButton.get()) {
      cimMotor.set(0.0);
    } else {
      if(usePidJumper.get()) {
         cimMotor.set(cimPidController.calculate(rpmMeasured, rpmSetPoint));
      } else {
        cimMotor.set(0.8);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    cimMotor.set(0.0);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    cimMotor.set(0.0);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    cimMotor.set(0.0);
  }
}
