// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team1678.lib.loops.Looper;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeExecutor;
import frc.team4276.frc2024.auto.AutoModeSelector;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

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

  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final ControlBoard mControlBoard = ControlBoard.getInstance();

  private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  private final AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  private AutoModeExecutor mAutoModeExecutor;

  public static boolean is_red_alliance = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
      mSubsystemManager.setSubsystems(
          mDriveSubsystem);

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

    } catch (Throwable t) {
      throw t;
    }

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    mSubsystemManager.outputToSmartDashboard();
    mEnabledLooper.outputToSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      mEnabledLooper.stop();
      mDisabledLooper.start();

    } catch (Throwable t) {
      throw t;

    }

    if (mAutoModeExecutor != null) {
      mAutoModeExecutor.stop();
    }

    mAutoModeSelector.reset();
    mAutoModeSelector.updateModeCreator(false);
    mAutoModeExecutor = new AutoModeExecutor();
  }

  @Override
  public void disabledPeriodic() {
    try {

      boolean alliance_changed = false;
      if (DriverStation.isDSAttached()) {
        if (DriverStation.getAlliance() == Alliance.Red) {
          if (!is_red_alliance) {
            alliance_changed = true;
          } else {
            alliance_changed = false;
          }
          is_red_alliance = true;
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
          if (is_red_alliance) {
            alliance_changed = true;
          } else {
            alliance_changed = false;
          }
          is_red_alliance = false;
        }
      } else {
        alliance_changed = true;
      }

      mAutoModeSelector.updateModeCreator(alliance_changed);
      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent()) {
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

    } catch (Throwable t) {
      throw t;
    }

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    try {
      
			mDisabledLooper.stop();
      
      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent()) {
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

      mEnabledLooper.start();
      mAutoModeExecutor.start();

    } catch (Throwable t) {
      throw t;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    try {
      if (mAutoModeSelector.getAutoMode().isPresent()) {
        if (mAutoModeSelector.getAutoMode().get().getStartingPose().getRotation().getDegrees() != 0) {
          mDriveSubsystem.zeroHeading(mDriveSubsystem.getHeading().getDegrees() + 180);
        }
      }

      mDisabledLooper.stop();
      mEnabledLooper.start();

    } catch (Throwable t) {
      throw t;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      if (mControlBoard.driver.getController().getAButtonPressed()) {
        mDriveSubsystem.zeroHeading();
      }

      if (mControlBoard.driver.getController().getXButton()) {
        mDriveSubsystem.setX();
      } else if (mControlBoard.driver.getLT()) {
        mDriveSubsystem.snapDrive(
            -mControlBoard.driver.getLeftY(),
            -mControlBoard.driver.getLeftX(),
            0, true);
      } else if (mControlBoard.driver.getRT()) {
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
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
