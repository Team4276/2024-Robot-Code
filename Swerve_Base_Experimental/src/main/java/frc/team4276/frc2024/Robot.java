// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team1678.lib.loops.Looper;
import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeExecutor;
import frc.team4276.frc2024.auto.AutoModeSelector;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.logger.CTestMonitor;

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

  public static Alliance alliance = Alliance.Invalid;

  public static CTestMonitor m_testMonitor = new CTestMonitor();

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

      if (Robot.m_testMonitor.isTestMonitorEnabled()) {
        String msg = new String("disabledInit()\n");
        Robot.m_testMonitor.logWrite(msg);
      }
  
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

      if (AutoModeSelector.mAllianceChooser.getSelected() != alliance){
        alliance = AutoModeSelector.mAllianceChooser.getSelected();
        alliance_changed = true;
      }

      if (DriverStation.isDSAttached() && DriverStation.isFMSAttached()) {
        if (DriverStation.getAlliance() != alliance) {
          alliance = DriverStation.getAlliance();
          alliance_changed = true;
        }
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
      if (Robot.m_testMonitor.isTestMonitorEnabled()) {
        String msg = new String("autonomousInit()\n");
        Robot.m_testMonitor.logWrite(msg);
      }
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
          mDriveSubsystem.zeroHeading(mDriveSubsystem.getHeading().plus(new Rotation2d(Math.PI)).getDegrees());
        }
      }

      mDisabledLooper.stop();
      mEnabledLooper.start();
      
      if (Robot.m_testMonitor.isTestMonitorEnabled()) {
        String msg = new String("teleopInit()\n");
        Robot.m_testMonitor.logWrite(msg);
      }
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
            0);
      } else if (mControlBoard.driver.getRT()) {
        mDriveSubsystem.snapDrive(
            -mControlBoard.driver.getLeftY(),
            -mControlBoard.driver.getLeftX(),
            180);
      } else {
        mDriveSubsystem.teleopDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
            mControlBoard.getSwerveTranslation().x(),
            mControlBoard.getSwerveTranslation().y(),
            mControlBoard.getSwerveRotation(),
            mDriveSubsystem.getHeading()));
      }

      if (mControlBoard.driver.getController().getRightBumperPressed()) {
        mDriveSubsystem.setKinematicLimits(DriveConstants.kUncappedLimits);
      }

      if (mControlBoard.driver.getController().getLeftBumperPressed()) {
        mDriveSubsystem.setKinematicLimits(DriveConstants.kDemoLimits);
      }

    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void testInit() {
    if (Robot.m_testMonitor.isTestMonitorEnabled()) {
      String msg = new String("testInit()\n");
      Robot.m_testMonitor.logWrite(msg);
    }
}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
