// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.vision.PhotonDevice.PhotonDeviceConstants;
import frc.team4276.lib.characterizations.FourBarFeedForward;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.RevUtil;
import frc.team4276.lib.rev.VIKCANSparkMaxServo;
import frc.team4276.lib.swerve.MAXSwerveModuleV3.MAXSwerveModuleConstants;
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
    //max file size of the log directory
    public static final double maxDirSize = 20;
    //size to reduce the directory by
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

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final boolean kGyroReversed = false;

        public static final double kMaxVel = ModuleConstants.kDriveWheelFreeSpeedRps; // meters per second
        public static final double kMaxAttainableVel = kMaxVel * 0.8;
        public static final double kMaxAttainableAccel = 8.9;

        public static final double kMaxAngularVel = kMaxVel * 2 / (kTrackWidth * Math.sqrt(2)); // radians per second

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
            kDemoLimits.kMaxDriveVelocity = 2.0;
            kDemoLimits.kMaxAccel = Double.MAX_VALUE;
            kDemoLimits.kMaxAngularVelocity = kMaxAngularVel; // Rad/Sec
            kDemoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
            kDemoLimits.kName = "Demo";
        }

        //TODO: tune
        public static final double kAutoTranslationKp = 3.0;
        public static final double kAutoTranslationKd = 0.0;
        public static final double kAutoRotationKp = 4.0;
        public static final double kAutoRotationKd = 0.0;

        public static final PIDConstants kAutoTranslationPIDConstants = new PIDConstants(kAutoTranslationKp, 0,
                kAutoTranslationKd);
        public static final PIDConstants kAutoRotationPIDConstants = new PIDConstants(kAutoRotationKp, 0,
                kAutoRotationKd);

        public static final double kAutoAccelFF = 0.0;

        public static final double kAutoMaxError = 0.75; // Meters

        public static final double kSnapHeadingKp = 0.18;
        public static final double kSnapHeadingKi = 0.0;
        public static final double kSnapHeadingKd = 0.0;

        public static final double kSnapPositionTolerance = 0.1;
        public static final double kSnapAngularVelocityTolerance = 0.1;

    }

    public static final class ModuleConstants {
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

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOpControllerPort = 1;
        public static final double kJoystickDeadband = 0.1;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    //TODO: tune (though these are probably accurate)
    public static final class RobotStateConstants {
        public static final boolean kVisionResetsHeading = false;

        public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.07, 1), Math.pow(0.07, 1));
        public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.03, 1),
                Math.pow(0.03, 1));
    }

    public static final class FourbarConstants {
        public static final FourBarFeedForward.FourBarFeedForwardConstants kFeedForwardConstants = new FourBarFeedForward.FourBarFeedForwardConstants();
        static {
            kFeedForwardConstants.kS = 0.185;

            kFeedForwardConstants.kMotorFreeSpeedRpm = 5676.0;
            kFeedForwardConstants.kGearRatio = 232.14;
            kFeedForwardConstants.kStallTorque = 3.28;
            kFeedForwardConstants.kMotorAmnt = 2;
            kFeedForwardConstants.kEfficiency = 0.2155;

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
            kSubsystemConstants.kMinPosition = 42.0; // Input Bound
            kSubsystemConstants.kMaxPosition = 125.0; // Input Bound
            kSubsystemConstants.kMaxVel = 0.0;
            kSubsystemConstants.kMaxAccel = 0.0;
            kSubsystemConstants.kS = 0.185;
            kSubsystemConstants.kTol = 1.0;
            kSubsystemConstants.kForwardLimitPolarity = SparkLimitSwitch.Type.kNormallyOpen;
            kSubsystemConstants.kReverseLimitPolarity = SparkLimitSwitch.Type.kNormallyOpen;
            
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
            kSubsystemConstants.kPidfConfigs[2].kP = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kI = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kD = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kFF = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kDFilter = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kIZone = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kIMaxAccum = 0.0;
            kSubsystemConstants.kPidfConfigs[2].kPIDOutputRange = 1.0;

            kSubsystemConstants.kFuseMotionConfig = new VIKCANSparkMaxServo.FuseMotionConfig();
            kSubsystemConstants.kFuseMotionConfig.kProfileSlot = kSubsystemConstants.kSlotIdFuseMotion;
            kSubsystemConstants.kFuseMotionConfig.kFeedForward = new FourBarFeedForward(kFeedForwardConstants);
            kSubsystemConstants.kFuseMotionConfig.kLooperDt = Constants.kLooperDt;
            kSubsystemConstants.kFuseMotionConfig.kMaxVel = kSubsystemConstants.kMaxVel;
            kSubsystemConstants.kFuseMotionConfig.kMaxAccel = kSubsystemConstants.kMaxAccel;
        }

        public static final RevUtil.SparkAbsoluteEncoderConfig kAbsoluteEncoderConfig = new RevUtil.SparkAbsoluteEncoderConfig();
        static {
            kAbsoluteEncoderConfig.kIsInverted = true;
            kAbsoluteEncoderConfig.kUnitsPerRotation = 360.0;
            kAbsoluteEncoderConfig.kOffset = 84.5;
            kAbsoluteEncoderConfig.kAvgSamplingDepth = 32;
        }
    }

    public static class FlywheelConstants {
        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static int kSmartCurrentLimit = 50;
        
        public static int kAvgSamplingDepth = 8;
        public static int kMeasurementPeriod = 10;
        public static double kUnitsPerRotation = 1.0;

        public static double kS_Top = 0.188;
        public static double kS_Bottom = 0.188;
        // Math: (V * S / m) / 60 sec / 39.37 in/m * circumference of flywheel
        public static double kV_Top = 0.002;
        public static double kV_Bottom = 0.002;
        public static double kA = 0;

        public static double kPrep = 2.0; // Volts

        public static double kFlywheelTolerance = 300.0; // RPM
    }

    public static final class SuperstructureConstants {
        // Radians
        public static final double kFourbarStowState = 120.0;
        public static final double kFourbarPrepState = 106.7;
        public static final double kFourbarIntakeState = 49.0;
        public static final double kFourbarSubCloseState = 45.0;
        public static final double kFourbarFerryState = 45.0;

        public static final int kNormalShotRPM = 3500;

        public static final double kShotWaitTime = 0.5;
        public static final double kExhaustWaitTime = 0.5;
    }

    public static final class VisionConstants {
        public static final PhotonDeviceConstants kFrontCameraConstants = new PhotonDeviceConstants();
        static {
            kFrontCameraConstants.kCameraName = "Front Camera";
            kFrontCameraConstants.kCameraNameId = "Arducam_OV9281_USB_Camera";
            kFrontCameraConstants.kRobotToCamera = new Transform3d();
        }

        public static final PhotonDeviceConstants kBackCameraConstants = new PhotonDeviceConstants();
        static {
            kFrontCameraConstants.kCameraName = "Back Camera";
            kBackCameraConstants.kCameraNameId = "PI_CAM_3";
            kBackCameraConstants.kRobotToCamera = new Transform3d();
        }
    }
}
