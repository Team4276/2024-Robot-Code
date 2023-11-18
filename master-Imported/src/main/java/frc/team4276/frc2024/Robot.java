// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeExecutor;
import frc.team4276.frc2024.auto.AutoModeSelector;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.frc2024.subsystems.DriveSubsystem;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  private final AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  public static Alliance alliance;
  private SendableChooser<Alliance> allianceChooser;

  private AutoModeExecutor mAutoModeExecutor;


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    allianceChooser = new SendableChooser<Alliance>();
    allianceChooser.setDefaultOption("Unselected", Alliance.Invalid);
    allianceChooser.addOption("Blue", Alliance.Blue);
    allianceChooser.addOption("Red", Alliance.Red);

    SmartDashboard.putData(allianceChooser);

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

    mAutoModeExecutor = new AutoModeExecutor();
  }

  @Override
  public void disabledPeriodic() {
    try {
      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
        if (autoMode.isPresent()) {
          mAutoModeExecutor.setAutoMode(autoMode.get());
        }

      if (DriverStation.isDSAttached()){
        alliance = DriverStation.getAlliance();
      } else {
        alliance = allianceChooser.getSelected();
      }
    } catch (Throwable t) {
      throw t;
    }

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    mAutoModeExecutor.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    try {
      if(mAutoModeSelector.getAutoMode().get().getStartingPose().getRotation().getDegrees() != 0){
        mDriveSubsystem.zeroHeading(mDriveSubsystem.getHeading().getDegrees() + 180);
      }
    } catch (Throwable t) {
      throw t;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {        
      if(mControlBoard.driver.getController().getAButtonPressed()){
        mDriveSubsystem.zeroHeading();
      }

      if(mControlBoard.driver.getController().getXButton()){
        mDriveSubsystem.setX();
      } else if(mControlBoard.driver.getLT()){
        mDriveSubsystem.snapDrive(
          -mControlBoard.driver.getLeftY(),
          -mControlBoard.driver.getLeftX(),
          0, true);
      } else if(mControlBoard.driver.getRT()){
        mDriveSubsystem.snapDrive(
          -mControlBoard.driver.getLeftY(),
          -mControlBoard.driver.getLeftX(),
          180, true);
      } else {
        mDriveSubsystem.drive(
          -mControlBoard.driver.getLeftY(),
          -mControlBoard.driver.getLeftX(),
          -mControlBoard.driver.getRightX(),
          true);
      }


    } catch (Throwable t) {
      throw t;
  }
  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
