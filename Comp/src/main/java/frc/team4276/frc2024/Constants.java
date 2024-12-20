// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.vision.PhotonDevice.PhotonDeviceConstants;
import frc.team4276.lib.characterizations.ElevatorFeedForward;
import frc.team4276.lib.characterizations.FourBarFeedForward;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.RevUtil;
import frc.team4276.lib.rev.VIKCANSparkMaxServo;
import frc.team4276.lib.rev.RevUtil.SparkRelativeEncoderConfig;
import frc.team4276.lib.swerve.MAXSwerveModule.MAXSwerveModuleConstants;

import frc.team1678.lib.swerve.SwerveDriveKinematics;

public final class Constants {
    public static final boolean isComp = false;

    public static final double kLooperDt = 0.02;

    // Enables extra SmartDashboard Debugs
    // Don't use during competition
    public static final boolean disableExtraTelemetry = false;

    public static final class DebugConstants {
        public static final boolean writeSwerveErrors = true;
        // set to "DriverStation" to log to driver station or set to "Standard Out" to
        // log to the standard output
        public static final String printOutput = "Standard Out";

        // Must end with a slash
        // Do not leave blank
        public static final String logDirectory = "/home/lvuser/logs/";
        // max file size of the log directory
        public static final double maxDirSize = 20;
        // size to reduce the directory by
        public static final double reductionSize = 5;

    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final MAXSwerveModuleConstants kFLConstants = new MAXSwerveModuleConstants();
        static {
            kFLConstants.kName = "Front Left";
            kFLConstants.kDriveId = Ports.FRONT_LEFT_DRIVE;
            kFLConstants.kTurnId = Ports.FRONT_LEFT_TURN;
            kFLConstants.kOffset = -Math.PI / 2;
        }

        public static final MAXSwerveModuleConstants kFRConstants = new MAXSwerveModuleConstants();
        static {
            kFRConstants.kName = "Front Right";
            kFRConstants.kDriveId = Ports.FRONT_RIGHT_DRIVE;
            kFRConstants.kTurnId = Ports.FRONT_RIGHT_TURN;
            kFRConstants.kOffset = 0;
        }

        public static final MAXSwerveModuleConstants kBRConstants = new MAXSwerveModuleConstants();
        static {
            kBRConstants.kName = "Back Right";
            kBRConstants.kDriveId = Ports.BACK_RIGHT_DRIVE;
            kBRConstants.kTurnId = Ports.BACK_RIGHT_TURN;
            kBRConstants.kOffset = Math.PI / 2;
        }

        public static final MAXSwerveModuleConstants kBLConstants = new MAXSwerveModuleConstants();
        static {
            kBLConstants.kName = "Back Left";
            kBLConstants.kDriveId = Ports.BACK_LEFT_DRIVE;
            kBLConstants.kTurnId = Ports.BACK_LEFT_TURN;
            kBLConstants.kOffset = Math.PI;
        }

        public static final boolean kGyroReversed = false;

        public static final double kMaxVel = MaxSwerveModuleConstants.kDriveWheelFreeSpeedRps; // meters per second
        public static final double kMaxAttainableVel = kMaxVel * 0.8;
        public static final double kMaxAttainableAccel = 8.8;

        public static final double kMaxAngularVel = kMaxVel * 2 / (kTrackWidth * Math.sqrt(2)); // radians per second
        public static final double kMaxAngularAccel = kMaxAttainableAccel * 2 / (kTrackWidth * Math.sqrt(2)); // radians per second

        public static final DriveSubsystem.KinematicLimits kUncappedLimits = new DriveSubsystem.KinematicLimits();
        static {
            kUncappedLimits.kMaxDriveVelocity = kMaxVel;
            kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
            kUncappedLimits.kMaxAngularVelocity = kMaxAngularVel;
            kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
            kUncappedLimits.kName = "Uncapped";
        }

        public static final DriveSubsystem.KinematicLimits kDemoLimits = new DriveSubsystem.KinematicLimits();
        static {
            kDemoLimits.kMaxDriveVelocity = 1.5;
            kDemoLimits.kMaxAccel = Double.MAX_VALUE;
            kDemoLimits.kMaxAngularVelocity = kMaxAngularVel; // Rad/Sec
            kDemoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
            kDemoLimits.kName = "Demo";
        }

        public static final double kAutoTranslationKp = 3.0;
        public static final double kAutoTranslationKd = 0.0;
        public static final double kAutoRotationKp = 4.0;
        public static final double kAutoRotationKd = 0.0;

        public static final PIDConstants kAutoTranslationPIDConstants = new PIDConstants(kAutoTranslationKp, 0,
                kAutoTranslationKd);
        public static final PIDConstants kAutoRotationPIDConstants = new PIDConstants(kAutoRotationKp, 0,
                kAutoRotationKd);

        public static final PPHolonomicDriveController kAutoDriveControllerConstants = 
            new PPHolonomicDriveController(kAutoTranslationPIDConstants, kAutoRotationPIDConstants);

        public static final ModuleConfig kPPModuleconfig = new ModuleConfig(
            Units.inchesToMeters(1.5), kMaxAttainableVel, 1.0, DCMotor.getNEO(1), 40, 1);

        public static final double kMOI = 0.5 * Units.lbsToKilograms(120.0) * Math.pow(Units.inchesToMeters(18), 2);

        public static final RobotConfig kPPRobotConfig = new RobotConfig(Units.lbsToKilograms(120.0), 
            kMOI, kPPModuleconfig, kTrackWidth, kWheelBase);

        //TODO: tune
        public static final double kAutoTransAccelFF = 0.0;
        public static final double kAutoRotAccelFF = 0.0;

        public static final double kAutoMaxError = 0.75; // Meters

        public static final double kSnapHeadingKp = 3.3;
        public static final double kSnapHeadingKi = 0.0;
        public static final double kSnapHeadingKd = 0.0;

        public static final double kSnapPositionTolerance = 2 * Math.PI / 180;
        public static final double kSnapAngularVelocityTolerance = 0.5;
        
        //TODO: tune
        public static final double kProfiledSnapHeadingKp = 3.3;
        public static final double kProfiledSnapHeadingKi = 0.0;
        public static final double kProfiledSnapHeadingKd = 0.0;

        public static final double kProfiledSnapPositionTolerance = 2 * Math.PI / 180;
        public static final double kProfiledSnapAngularVelocityTolerance = 0.5;

    }

    public static final class MaxSwerveModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI); // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final CANSparkMaxFactory.CANSparkMaxPIDFConfig kDrivingPIDFConfig = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
        static {
            kDrivingPIDFConfig.kP = 0.04;
            kDrivingPIDFConfig.kFF = 1 / kDriveWheelFreeSpeedRps;
            kDrivingPIDFConfig.kPIDOutputRange = 1.0;
        }

        public static final CANSparkMaxFactory.CANSparkMaxPIDFConfig kTurningPIDFConfig = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
        static {
            kTurningPIDFConfig.kP = 1.0;
            kTurningPIDFConfig.kPIDOutputRange = 1.0;
        }

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static final RevUtil.SparkAbsoluteEncoderConfig kTurningEncoderConfig = new RevUtil.SparkAbsoluteEncoderConfig();
        static {
            kTurningEncoderConfig.kIsInverted = kTurningEncoderInverted;
            kTurningEncoderConfig.kUnitsPerRotation = kTurningEncoderPositionFactor;
            kTurningEncoderConfig.kPeriodicFrameTime = 0.02;
        }
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOpControllerPort = 1;
        public static final double kJoystickDeadband = 0.1;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class RobotStateConstants {
        public static final boolean kVisionResetsHeading = false;

        public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1));
        public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.03, 1),
                Math.pow(0.03, 1));
    }

    public static final class FourbarConstants {
        public static final FourBarFeedForward.FourBarFeedForwardConstants kFeedForwardConstants = new FourBarFeedForward.FourBarFeedForwardConstants();
        static {
            kFeedForwardConstants.kS = 0.14;

            kFeedForwardConstants.kMotorFreeSpeedRpm = 5676.0;
            kFeedForwardConstants.kGearRatio = 185.712;
            kFeedForwardConstants.kStallTorque = 3.28;
            kFeedForwardConstants.kMotorAmnt = 2;
            kFeedForwardConstants.kEfficiency = 0.3;

            kFeedForwardConstants.kBottomLength = Units.inchesToMeters(8.001578);
            kFeedForwardConstants.kMotorLegLength = Units.inchesToMeters(11.000000);
            kFeedForwardConstants.kTopLength = Units.inchesToMeters(12.000808);
            kFeedForwardConstants.kSupportLegLength = Units.inchesToMeters(9.750);

            kFeedForwardConstants.kMotorLegMass = Units.lbsToKilograms(0.86);
            kFeedForwardConstants.kTopMass = Units.lbsToKilograms(28.0);
            kFeedForwardConstants.kSupportLegMass = Units.lbsToKilograms(0.494);

            kFeedForwardConstants.kMotorToCom = new frc.team254.lib.geometry.Translation2d(Units.inchesToMeters(5.5),
                    Units.inchesToMeters(0));
            kFeedForwardConstants.kMotorLegToTopCom = new frc.team254.lib.geometry.Translation2d(
                    Units.inchesToMeters(4.799278),
                    Units.inchesToMeters(1.121730));
            kFeedForwardConstants.kSupportToCom = new frc.team254.lib.geometry.Translation2d(
                    Units.inchesToMeters(5.076911),
                    Units.inchesToMeters(1.096583));
        }

        public static final ServoMotorSubsystem.ServoMotorSubsystemConstants kSubsystemConstants = new ServoMotorSubsystem.ServoMotorSubsystemConstants();
        static {
            kSubsystemConstants.kName = "Fourbar";

            kSubsystemConstants.kMasterConstants.id = Ports.FOURBAR_MASTER;
            kSubsystemConstants.kMasterConstants.isInverted = false;

            kSubsystemConstants.kFollowerConstants = new ServoMotorSubsystem.ServoMotorConfig[1];

            kSubsystemConstants.kFollowerConstants[0] = new ServoMotorSubsystem.ServoMotorConfig();
            kSubsystemConstants.kFollowerConstants[0].id = Ports.FOURBAR_FOLLOWER;
            kSubsystemConstants.kFollowerConstants[0].isInverted = true;

            kSubsystemConstants.kSmartCurrentLimit = 40;
            kSubsystemConstants.kIdleMode = IdleMode.kBrake;
            kSubsystemConstants.kIsCircular = false; // assume max position is 360
            kSubsystemConstants.kMinPosition = 50.0; // Input Bound
            kSubsystemConstants.kMaxPosition = 135.0; // Input Bound
            kSubsystemConstants.kMaxVel = 150.0;
            kSubsystemConstants.kMaxAccel = 175.0; //TODO: tune to be faster
            kSubsystemConstants.kS = 0.14; // Smart Motion Firmware
            kSubsystemConstants.kTol = 1.0;
            kSubsystemConstants.kForwardLimitPolarity = Type.kNormallyOpen;
            kSubsystemConstants.kReverseLimitPolarity = Type.kNormallyOpen;

            kSubsystemConstants.kSlotIdSmartMotionCruise = 0;
            kSubsystemConstants.kSlotIdSmartMotionMaintain = 1;
            kSubsystemConstants.kSlotIdFuseMotion = 2;

            kSubsystemConstants.kPidfConfigs = new CANSparkMaxFactory.CANSparkMaxPIDFConfig[3];

            kSubsystemConstants.kPidfConfigs[0] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kSubsystemConstants.kPidfConfigs[0].kSlotId = 0; // Smart Motion Cruise (Velocity Control)
            kSubsystemConstants.kPidfConfigs[0].kP = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kI = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kD = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kFF = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kDFilter = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kIZone = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kIMaxAccum = 0.0;
            kSubsystemConstants.kPidfConfigs[0].kPIDOutputRange = 1.0;

            kSubsystemConstants.kPidfConfigs[1] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kSubsystemConstants.kPidfConfigs[1].kSlotId = 1; // Smart Motion Maintain (Position Control)
            kSubsystemConstants.kPidfConfigs[1].kP = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kI = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kD = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kFF = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kDFilter = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kIZone = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kIMaxAccum = 0.0;
            kSubsystemConstants.kPidfConfigs[1].kPIDOutputRange = 1.0;

            kSubsystemConstants.kPidfConfigs[2] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kSubsystemConstants.kPidfConfigs[2].kSlotId = 2; // Fuse Motion
            kSubsystemConstants.kPidfConfigs[2].kP = 0.02;
            kSubsystemConstants.kPidfConfigs[2].kI = 0.0001;
            kSubsystemConstants.kPidfConfigs[2].kD = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kFF = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kDFilter = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kIZone = 1.0;
            kSubsystemConstants.kPidfConfigs[2].kIMaxAccum = 0.01;
            kSubsystemConstants.kPidfConfigs[2].kPIDOutputRange = 1.0;

            kSubsystemConstants.kFuseMotionConfig = new VIKCANSparkMaxServo.FuseMotionConfig();
            kSubsystemConstants.kFuseMotionConfig.kProfileSlot = kSubsystemConstants.kSlotIdFuseMotion;
            kSubsystemConstants.kFuseMotionConfig.kFeedForward = new FourBarFeedForward(kFeedForwardConstants);
            kSubsystemConstants.kFuseMotionConfig.kLooperDt = 0.005;
            kSubsystemConstants.kFuseMotionConfig.kMaxVel = kSubsystemConstants.kMaxVel;
            kSubsystemConstants.kFuseMotionConfig.kMaxAccel = kSubsystemConstants.kMaxAccel;
        }

        public static final RevUtil.SparkAbsoluteEncoderConfig kFourbarEncoderConfig = new RevUtil.SparkAbsoluteEncoderConfig();
        static {
            kFourbarEncoderConfig.kIsInverted = false;
            kFourbarEncoderConfig.kUnitsPerRotation = 360.0;
            kFourbarEncoderConfig.kOffset = 94.5;
            kFourbarEncoderConfig.kAvgSamplingDepth = 128;
            kFourbarEncoderConfig.kPeriodicFrameTime = 0.2;
        }
    }

    public static class FlywheelConstants {
        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kSmartCurrentLimit = 50;

        public static final int kAvgSamplingDepth = 8;
        public static final int kMeasurementPeriod = 10;
        public static final double kUnitsPerRotation = 1.0;

        public static final double kS_Top = 0.15;
        public static final double kS_Bottom = 0.15;
        // Math: (V * S / m) / 60 sec / 39.37 in/m * circumference of flywheel
        public static final double kV_Top = 0.002;
        public static final double kV_Bottom = 0.002;
        public static final double kA = 0;

        public static final double kPrep = 4.0; // Volts

        public static final double kFlywheelTolerance = 300.0; // RPM
    }

    public static class ClimberSubsystemConstants {
        public static final ServoMotorSubsystemConstants kClimberServoConstants = new ServoMotorSubsystemConstants();
        static {
            kClimberServoConstants.kName = "Climber";

            kClimberServoConstants.kMasterConstants.id = Ports.CLIMBER_RIGHT;
            kClimberServoConstants.kMasterConstants.isInverted = false;

            kClimberServoConstants.kFollowerConstants = new ServoMotorSubsystem.ServoMotorConfig[1];
            
            kClimberServoConstants.kFollowerConstants[0] = new ServoMotorSubsystem.ServoMotorConfig();
            kClimberServoConstants.kFollowerConstants[0].id = Ports.CLIMBER_LEFT;
            kClimberServoConstants.kFollowerConstants[0].isInverted = true;

            kClimberServoConstants.kSmartCurrentLimit = 60;
            kClimberServoConstants.kIdleMode = IdleMode.kBrake;
            kClimberServoConstants.kIsCircular = false; // assume max position is 360
            kClimberServoConstants.kMinPosition = 0.0; // Input Bound
            kClimberServoConstants.kMaxPosition = 0.0; // Input Bound
            kClimberServoConstants.kMaxVel = 0.0;
            kClimberServoConstants.kMaxAccel = 0.0;
            kClimberServoConstants.kS = 0.0; // Smart Motion Firmware
            kClimberServoConstants.kTol = 0.0;
            // kClimberServoConstants.kForwardLimitPolarity = Type.kNormallyOpen;
            // kClimberServoConstants.kReverseLimitPolarity = Type.kNormallyOpen;

            kClimberServoConstants.kSlotIdSmartMotionCruise = 0;
            kClimberServoConstants.kSlotIdSmartMotionMaintain = 1;
            kClimberServoConstants.kSlotIdFuseMotion = 2;

            kClimberServoConstants.kPidfConfigs = new CANSparkMaxFactory.CANSparkMaxPIDFConfig[3];

            kClimberServoConstants.kPidfConfigs[0] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kClimberServoConstants.kPidfConfigs[0].kSlotId = 0; // Smart Motion Cruise (Velocity Control)
            kClimberServoConstants.kPidfConfigs[0].kP = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kI = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kD = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kFF = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kDFilter = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kIZone = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kIMaxAccum = 0.0;
            kClimberServoConstants.kPidfConfigs[0].kPIDOutputRange = 1.0;

            kClimberServoConstants.kPidfConfigs[1] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kClimberServoConstants.kPidfConfigs[1].kSlotId = 1; // Smart Motion Maintain (Position Control)
            kClimberServoConstants.kPidfConfigs[1].kP = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kI = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kD = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kFF = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kDFilter = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kIZone = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kIMaxAccum = 0.0;
            kClimberServoConstants.kPidfConfigs[1].kPIDOutputRange = 1.0;

            kClimberServoConstants.kPidfConfigs[2] = new CANSparkMaxFactory.CANSparkMaxPIDFConfig();
            kClimberServoConstants.kPidfConfigs[2].kSlotId = 2; // Fuse Motion
            kClimberServoConstants.kPidfConfigs[2].kP = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kI = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kD = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kFF = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kDFilter = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kIZone = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kIMaxAccum = 0.0;
            kClimberServoConstants.kPidfConfigs[2].kPIDOutputRange = 1.0;

            kClimberServoConstants.kFuseMotionConfig = new VIKCANSparkMaxServo.FuseMotionConfig();
            kClimberServoConstants.kFuseMotionConfig.kProfileSlot = kClimberServoConstants.kSlotIdFuseMotion;
            kClimberServoConstants.kFuseMotionConfig.kFeedForward = new ElevatorFeedForward(
                kClimberServoConstants.kS, 0.0, 0.05);
            kClimberServoConstants.kFuseMotionConfig.kLooperDt = 0.02;
            kClimberServoConstants.kFuseMotionConfig.kMaxVel = kClimberServoConstants.kMaxVel;
            kClimberServoConstants.kFuseMotionConfig.kMaxAccel = kClimberServoConstants.kMaxAccel;
        }

        public static final SparkRelativeEncoderConfig kEncoderConfig = new SparkRelativeEncoderConfig(); // Use defaults bc why not
    }

    public static final class SuperstructureConstants {
        public static final double kFourbarStowState = 70.0;
        public static final double kFourbarPrepState = 90.0;
        public static final double kFourbarIntakeState = 135.0;
        public static final double kFourbarSubCloseState = 135.0;
        public static final double kFourbarFerryState = 135.0;
        public static final double kFourbarAmpState = 135;
        public static final double kFourbarSkimState = 120;
        public static final double kFourbarClimbState = 50.0;
        public static final double kFourbarPoopState = 50.0;

        public static final int kNormalShotRPM = 3500;
        public static final int kFerryRPM = 3500;
        public static final int kSpinUpRPM = 2500;
        public static final int kPoopTopRPM = 3500;
        public static final int kPoopBotRPM = 3200;
        public static final int kExhaustRPM = -1000;
        public static final int kAmpTopRPM = 150;
        public static final int kAmpBotRPM = 1450;

        public static final double kSpinUpDistance = 4.0;
        public static final double kDoableShotDistance = 4.0; //TODO: tune
    }

    public static final class VisionConstants {
        public static final PhotonDeviceConstants kFrontCameraConstants = new PhotonDeviceConstants();
        static {
            kFrontCameraConstants.kCameraName = "Front Camera";
            kFrontCameraConstants.kCameraNameId = "Arducam_OV9281_USB_Camera";
            kFrontCameraConstants.kRobotToCamera = new Transform3d(new Translation3d(
                Units.inchesToMeters(9.591351), Units.inchesToMeters(7.500000) * -1.0, Units.inchesToMeters(5.575688)), 
                new Rotation3d(0.0, Math.toRadians(20.0) * -1.0, 0.0));
        }

        public static final PhotonDeviceConstants kBackCameraConstants = new PhotonDeviceConstants();
        static {
            kFrontCameraConstants.kCameraName = "Back Camera";
            kBackCameraConstants.kCameraNameId = "Arducam_12MP";
            kBackCameraConstants.kRobotToCamera = new Transform3d(new Translation3d(
                Units.inchesToMeters(7.837035), Units.inchesToMeters(8.750000), Units.inchesToMeters(6.072381)),
                new Rotation3d(0.0, Math.toRadians(20.0) * -1.0, 0.0));
        }
    }
}
