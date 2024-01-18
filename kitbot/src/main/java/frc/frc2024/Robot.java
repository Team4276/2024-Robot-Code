// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.frc2024;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.frc2024.Constants.LauncherConstants;
import frc.frc2024.Constants.OperatorConstants;
import frc.frc2024.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Shooter mShooter = Shooter.getInstance();

  private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController opController = new XboxController(OperatorConstants.kOperatorControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    mShooter.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (opController.getAButton()){
      mShooter.setSpeed(LauncherConstants.kLauncherSpeedLow);
    } else if (opController.getBButton()) {
      mShooter.setSpeed(LauncherConstants.kLauncherSpeedMid);
    } else if (opController.getYButton()) {
      mShooter.setSpeed(LauncherConstants.kLauncherSpeedHigh);
    } else if (opController.getXButton()) {
      mShooter.setSpeed(LauncherConstants.kLaunchFeederSpeed);
    } else {
      mShooter.setSpeed(0);
    }
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
