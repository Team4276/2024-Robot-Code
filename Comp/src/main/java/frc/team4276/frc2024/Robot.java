// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeExecutor;
import frc.team4276.frc2024.auto.AutoModeSelector;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.ClimberSubsystem;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.FlywheelSubsystem;
import frc.team4276.frc2024.subsystems.IntakeSubsystem;
import frc.team4276.frc2024.subsystems.FourbarSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.vision.VisionDeviceManager;

import frc.team1678.lib.loops.Looper;

import frc.team254.lib.geometry.Pose2d;

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

    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private DriveSubsystem mDriveSubsystem;
    private VisionDeviceManager mVisionDeviceManager;
    private IntakeSubsystem mIntakeSubsystem;
    private FlywheelSubsystem mFlywheelSubsystem;
    private FourbarSubsystem mFourbarSubsystem;
    private ClimberSubsystem mClimberSubsystem;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final AutoModeSelector mAutoModeSelector = AutoModeSelector.getInstance();;
    private AutoModeExecutor mAutoModeExecutor;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        try {
            mDriveSubsystem = DriveSubsystem.getInstance();
            // mVisionDeviceManager = VisionDeviceManager.getInstance();
            mIntakeSubsystem = IntakeSubsystem.getInstance();
            mFlywheelSubsystem = FlywheelSubsystem.getInstance();
            mFourbarSubsystem = FourbarSubsystem.getInstance();
            mClimberSubsystem = ClimberSubsystem.getInstance();

            CameraServer.startAutomaticCapture();

            // Set subsystems
            mSubsystemManager.setSubsystems(
                    mDriveSubsystem,
                    mSuperstructure,
                    mIntakeSubsystem,
                    mFlywheelSubsystem,
                    mFourbarSubsystem,
                    // mVisionDeviceManager,
                    mClimberSubsystem);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            RobotState.getInstance().resetKalmanFilters();

        } catch (Throwable t) {
            throw t;
        }
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
        mEnabledLooper.outputToSmartDashboard();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();
            mDisabledLooper.start();
            mSubsystemManager.stop();

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
    
    private boolean mHasFlippedClimberSetting = false;

    @Override
    public void disabledPeriodic() {
        try {
            if (AllianceChooser.getInstance().isAllianceRed()) {
                RobotState.getInstance().setRed();
            } else {
                RobotState.getInstance().setBlue();
            }

            mAutoModeSelector.updateModeCreator(AllianceChooser.getInstance().isAllianceChanged());
            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent()) {
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            // Safety for Climber
            if (mHasFlippedClimberSetting) {
                mClimberSubsystem.setWantBrakeMode(!mControlBoard.wantClimberCoastMode());

            } else if (!mControlBoard.wantClimberCoastMode()) {
                mHasFlippedClimberSetting = true;

            }

            mFourbarSubsystem.setWantBrakeMode(!mControlBoard.wantFourbarCoastMode());

        } catch (Throwable t) {
            throw t;
        }

    }

    @Override
    public void disabledExit() {
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

                mDriveSubsystem.resetGyro(autoMode.get().getStartingPose().getRotation().getDegrees());
                mDriveSubsystem.resetOdometry(autoMode.get().getStartingPose());
            } else {
                mDriveSubsystem.resetGyro(AllianceChooser.getInstance().isAllianceRed() ? 180.0 : 0.0);
                mDriveSubsystem.resetOdometry(Pose2d.identity());

            }

            if (Constants.RobotStateConstants.kVisionResetsHeading) {
                mDriveSubsystem.resetGyro(Math.toDegrees(RobotState.getInstance().getHeadingFromVision()));

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
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        try {
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
            mControlBoard.update();

        } catch (Throwable t) {
            System.out.println(t.getMessage());
            throw t;
        }
    }

    @Override
    public void teleopExit() {
        mHasFlippedClimberSetting = false;
    }

    @Override
    public void testInit() {
        SmartDashboard.putNumber("Debug/Test/Fourbar Des Position", 90.0);
        SmartDashboard.putNumber("Debug/Test/Fourbar Des Voltage", 0.0);
        SmartDashboard.putNumber("Debug/Test/Flywheel Des RPM", 0.0);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        // mFourbarSubsystem.setFuseMotionSetpoint(SmartDashboard.getNumber("Debug/Test/Fourbar Des Position", 90.0));
        mFourbarSubsystem.setVoltage(SmartDashboard.getNumber("Debug/Test/Fourbar Des Voltage", 0.0));
        mFlywheelSubsystem.setTargetRPM(SmartDashboard.getNumber("Debug/Test/Flywheel Des RPM", 0.0));
    }
}