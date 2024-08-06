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
import frc.team4276.frc2024.subsystems.LimeLight;
import frc.team4276.frc2024.subsystems.FourbarSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team1678.lib.loops.Looper;
import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team4276.frc2024.Logging.LoggableRobotFile;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;

//TODO: refactor imports

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
    private LimeLight mLimeLight;
    private IntakeSubsystem mIntakeSubsystem;
    private FlywheelSubsystem mFlywheelSubsystem;
    private FourbarSubsystem mFourbarSubsystem;
    private ClimberSubsystem mClimberSubsystem;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        try {
            LoggableRobotFile logger = new LoggableRobotFile("test");
            logger.init();
            logger.writeToFile("testing", LoggableRobotFile.DebugLevel.DEBUG);
            mDriveSubsystem = DriveSubsystem.getInstance();
            mLimeLight = LimeLight.getInstance();
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
                    mLimeLight,
                    mClimberSubsystem);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            RobotState.getInstance().resetKalmanFilters();

            SmartDashboard.putNumber("Fourbar des voltage input", 0.0);

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

    private boolean mHasFlippedClimberSetting = false;

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();
            mDisabledLooper.start();
            mSubsystemManager.stop();
            mLimeLight.start();
            // mLimeLight.setDisableProcessing(false);

        } catch (Throwable t) {
            throw t;

        }

        if (mAutoModeExecutor != null) {
            mAutoModeExecutor.stop();
        }

        mAutoModeSelector.reset();
        mAutoModeSelector.updateModeCreator(false);
        mAutoModeExecutor = new AutoModeExecutor();

        mHasFlippedClimberSetting = false;
    }

    @Override
    public void disabledPeriodic() {
        try {
            if (AllianceChooser.getInstance().isAllianceRed()) {
                RobotState.getInstance().setRed();
                mLimeLight.setRedTagMap();
            } else {
                RobotState.getInstance().setBlue();
                mLimeLight.setBlueTagMap();
            }

            mAutoModeSelector.updateModeCreator(AllianceChooser.getInstance().isAllianceChanged());
            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent()) {
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            // Safety for Climber
            boolean wantClimberCoastMode = mControlBoard.wantClimberCoastMode();

            if (mHasFlippedClimberSetting) {
                mClimberSubsystem.setWantBrakeMode(!wantClimberCoastMode);

            } else if (!wantClimberCoastMode) {
                mHasFlippedClimberSetting = true;

            }

            mFourbarSubsystem.setWantBrakeMode(!mControlBoard.wantFourbarCoastMode());

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

                // TODO: Reset with vision
                mDriveSubsystem.resetOdometry(autoMode.get().getStartingPose());
            } else {
                mDriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0.0)));

            }

            mEnabledLooper.start();
            mAutoModeExecutor.start();

            // mLimeLight.setDisableProcessing(false);
            RobotState.getInstance().setHasBeenEnabled(true);

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
            mDisabledLooper.stop();
            mEnabledLooper.start();

            // mLimeLight.setDisableProcessing(false);

            RobotState.getInstance().setHasBeenEnabled(true);

        } catch (Throwable t) {
            throw t;
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        try {
            mControlBoard.update();

            if (mControlBoard.wantZeroHeading()) {
                mDriveSubsystem.resetHeading(AllianceChooser.getInstance().isAllianceRed() ? 180.0 : 0.0);
            }

            if (mControlBoard.wantXBrake()) {
                mDriveSubsystem.setX();
            } else {
                mDriveSubsystem.teleopDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        mControlBoard.getSwerveTranslation().x(),
                        mControlBoard.getSwerveTranslation().y(),
                        mControlBoard.getSwerveRotation(),
                        mDriveSubsystem.getHeading().toWPI(),
                        AllianceChooser.getInstance().isAllianceRed()));
            }

            if (mControlBoard.wantDemoLimits()) {
                mDriveSubsystem.setKinematicLimits(Constants.DriveConstants.kDemoLimits);
            } else {
                mDriveSubsystem.setKinematicLimits(Constants.DriveConstants.kUncappedLimits);
            }

            if (mControlBoard.wantAutoLock()) {

            }

            if (mControlBoard.wantDynamic()) {

            }

            if (mControlBoard.wantStow()) {

            }

            if (mControlBoard.wantPrep()) {

            }

            if (mControlBoard.wantIdle()) {
                mSuperstructure.setGoalState(Superstructure.GoalState.IDLE);

            } else if (mControlBoard.wantIntake()) {
                mSuperstructure.setGoalState(Superstructure.GoalState.INTAKE);

            } else if (mControlBoard.wantShoot()) {
                mSuperstructure.setGoalState(Superstructure.GoalState.SHOOT);

            } else if (mControlBoard.wantExhaust()) {
                mSuperstructure.setGoalState(Superstructure.GoalState.EXHAUST);

            } else {
                mSuperstructure.setGoalState(Superstructure.GoalState.STOW);

            }

            if (mControlBoard.wantClimbMode()) {
                if (mControlBoard.wantRaiseClimber()) {
                    mClimberSubsystem.setDesiredState(ClimberSubsystem.State.RAISE);

                } else if (mControlBoard.wantSlowLowerClimber()) {
                    mClimberSubsystem.setDesiredState(ClimberSubsystem.State.SLOW_LOWER);

                } else if (mControlBoard.wantLowerClimber()) {
                    mClimberSubsystem.setDesiredState(ClimberSubsystem.State.LOWER);

                } else {
                    mClimberSubsystem.setDesiredState(ClimberSubsystem.State.IDLE);

                }
            }

        } catch (Throwable t) {
            System.out.println(t.getMessage());
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
