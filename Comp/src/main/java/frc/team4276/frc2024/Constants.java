// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

import frc.team1678.lib.swerve.SwerveDriveKinematics;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team4276.frc2024.Limelight.LimelightConstantsFactory;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.statemachines.SuperstructureState;
import frc.team4276.frc2024.statemachines.SuperstructureState.FlywheelTolerance;
import frc.team4276.frc2024.statemachines.SuperstructureState.FourbarSpeed;
import frc.team4276.frc2024.statemachines.SuperstructureState.FourbarTolerance;
import frc.team4276.frc2024.subsystems.DriveSubsystem.KinematicLimits;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.lib.drivers.FourBarFeedForward.FourBarFeedForwardConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.team4276.lib.motion.ProfileFollower.ProfileFollowerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean isComp = false;

  public static final double kLooperDt = 0.02;

  // Enables extra SmartDashboard Debugs
  // Don't use during competition
  public static final boolean disableExtraTelemetry = false;

  public static final class DebugConstants {
    public static final boolean writeSwerveErrors = true;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxVel = 4.8; // meters per second
    public static final double kMaxAttainableVel = kMaxVel * 0.85;

    public static final double kMaxAngularVel = 2 * Math.PI; // radians per second

    public static final KinematicLimits kUncappedLimits = new KinematicLimits();
    static {
      kUncappedLimits.kMaxDriveVelocity = kMaxVel;
      kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
      kUncappedLimits.kMaxAngularVelocity = kMaxAngularVel;
      kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
      kUncappedLimits.kName = "Uncapped";
    }

    public static final KinematicLimits kAutoLimits = new KinematicLimits();
    static {
      kAutoLimits.kMaxDriveVelocity = kMaxAttainableVel;
      kAutoLimits.kMaxAccel = Double.MAX_VALUE;
      kAutoLimits.kMaxAngularVelocity = Double.MAX_VALUE; // Rad/Sec
      kAutoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI
      kAutoLimits.kName = "Auto";

    }

    public static final KinematicLimits kDemoLimits = new KinematicLimits();
    static {
      kDemoLimits.kMaxDriveVelocity = 1.0;
      kDemoLimits.kMaxAccel = Double.MAX_VALUE;
      kDemoLimits.kMaxAngularVelocity = kMaxAngularVel / 4; // Rad/Sec
      kDemoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
      kDemoLimits.kName = "Demo";
    }

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kRearRightTurningCanId = 6;

    public static final int[] kDriveTrainCanIds = {
        kFrontLeftDrivingCanId,
        kFrontRightDrivingCanId,
        kRearLeftDrivingCanId,
        kRearRightDrivingCanId,
        kFrontLeftTurningCanId,
        kFrontRightTurningCanId,
        kRearLeftTurningCanId,
        kRearRightTurningCanId
    };

    public static final boolean kGyroReversed = false;
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
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

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

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
    public static final double kJoystickDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1;
    public static final double kDXController = 0;
    public static final double kPYController = 1;
    public static final double kDYController = 0;
    public static final double kPThetaController = 1;
    public static final double kDThetaController = 1;

    public static final PIDConstants kTranslationPIDConstants = new PIDConstants(kPXController, 0, kDXController);
    public static final PIDConstants kRotationPIDConstants = new PIDConstants(kPThetaController, 0, kDThetaController);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SnapConstants {
    // Radians

    public static final double kP = 0.18;
    public static final double kI = 0.009;
    public static final double kD = 0.012;

    public static final double kPositionTolerance = 0.1;
    public static final double kAngularVelocityTolerance = 0.1;
  }

  public static final class LimelightConstants {
    public static final int kImageCaptureLatency = 11;
    public static final int kLimelightTransmissionTimeLatency = 0;

    public static final double kMaxAcceptibleTargetDist = 10;

    public static final boolean disableAfterTeleop = !isComp;

    public static final Pose2d kLimeLightRobotOffset = new Pose2d(0.25, 0, new Rotation2d(0));
    public static final Rotation2d kLimeLightTilt = new Rotation2d(20);

    public static final double kLensHeight = 0.2;

    public static final double kResolutionWidth = 1280;
    public static final double kResolutionHeight = 960;
    public static final frc.team254.lib.limelight.LimelightConstants kLimelightConstants = LimelightConstantsFactory
        .getConstantsForId("test");
  }

  public static final class RobotStateConstants {
    public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
    public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));
  }

  public static final class AutoAlignConstants {

    public static final ProfileFollowerConstants kTranslationConstants = new ProfileFollowerConstants();
    static {
      kTranslationConstants.kP = 0.0;
      kTranslationConstants.kI = 0.0;
      kTranslationConstants.kV = 0.0;
      kTranslationConstants.kFFV = 0.0;
      kTranslationConstants.kFFA = 0.0;
      kTranslationConstants.kFFS = 0.0;
      kTranslationConstants.kTol = 0.05;
      kTranslationConstants.kXTol = 0.05;
      kTranslationConstants.kDxTol = 0.05;
    }

    public static final double kMaxTransAccel = 0.0;
    public static final double kMaxTransVel = 0.0;

    public static final ProfileFollowerConstants kThetaConstants = new ProfileFollowerConstants();
    static {
      kThetaConstants.kP = 0.0;
      kThetaConstants.kI = 0.0;
      kThetaConstants.kV = 0.0;
      kThetaConstants.kFFV = 0.0;
      kThetaConstants.kFFA = 0.0;
      kThetaConstants.kFFS = 0.0;
      kThetaConstants.kTol = Math.PI / 50;
      kThetaConstants.kXTol = Math.PI / 50;
      kThetaConstants.kDxTol = Math.PI / 50;
    }

    public static final double kMaxThetaAccel = 0.0;
    public static final double kMaxThetaVel = 0.0;

    // Metres
    public static final double kTranslationTolerance = 0.05;
    public static final double kTranslationUpdateTolerance = 0.05;

    public static final double kThetaTolerance = Math.PI / 50;
    public static final double kThetaUpdateTolerance = Math.PI / 50;
  }

  public static final class SuperstructureConstants {
    public static final FourBarFeedForwardConstants kFourbarFFConstants = new FourBarFeedForwardConstants();
    static {
      kFourbarFFConstants.kS = 0.0;

      kFourbarFFConstants.kMotorFreeSpeedRpm = 5880.0;
      kFourbarFFConstants.kGearRatio = 330.0;
      kFourbarFFConstants.kStallTorque = 3.28;
      kFourbarFFConstants.kMotorAmnt = 1;
      kFourbarFFConstants.kEfficiency = 0.8;

      kFourbarFFConstants.kBottomLength = 0.0;
      kFourbarFFConstants.kMotorLegLength = 0.0;
      kFourbarFFConstants.kTopLength = 0.0;
      kFourbarFFConstants.kSupportLegLength = 0.0;

      kFourbarFFConstants.kMotorLegMass = 0.0;
      kFourbarFFConstants.kTopMass = 0.0;
      kFourbarFFConstants.kSupportLegMass = 0.0;

      kFourbarFFConstants.kMotorToCom = frc.team254.lib.geometry.Translation2d.identity();
      kFourbarFFConstants.kMotorLegToTopCom = frc.team254.lib.geometry.Translation2d.identity();
      kFourbarFFConstants.kSupportToCom = frc.team254.lib.geometry.Translation2d.identity();
    }

    public static final ServoMotorSubsystemConstants kFourBarConstants = new ServoMotorSubsystemConstants();
    static {
      kFourBarConstants.kMasterConstants.id = 9;
      kFourBarConstants.kMasterConstants.isInverted = false;

      kFourBarConstants.kIsCircular = true;
      kFourBarConstants.kUnitsPerRotation = 2 * Math.PI; // Overall Rotation to Radians
      kFourBarConstants.kGearRatio = 330.0; // Motor Rotations to Overall Rotations
      kFourBarConstants.kOffset = 0.0; // Set in hardware client
      kFourBarConstants.kHomePosition = 0.0;
      kFourBarConstants.kMinPosition = Double.NEGATIVE_INFINITY;
      kFourBarConstants.kMaxPosition = Double.POSITIVE_INFINITY;
      kFourBarConstants.kRelativeEncoderAvgSamplingDepth = 2;

      // TODO: add PID placeholders here

      kFourBarConstants.kFourBarFFConstants = kFourbarFFConstants;
      kFourBarConstants.kFourBarProfileContraints = new Constraints(Math.PI / 8, Math.PI / 8);

      kFourBarConstants.kIdleMode = IdleMode.kBrake;

      kFourBarConstants.kSmartCurrentLimit = 40;
      kFourBarConstants.kVoltageCompensation = 12.0;
    }

    public static final double kSlowFourbarSpeed = 0.0; // radians / second
    public static final double kMediumFourbarSpeed = 0.0;
    public static final double kSonicFourbarSpeed = 0.0;

    public static final double kLiberalFourbarTolerance = 0.0; // radians
    public static final double kLiberalFlywheelTolerance = 0.0; // RPM

    public static final double kModerateFourbarTolerance = 0.0;
    public static final double kModerateFlywheelTolerance = 0.0;

    public static final double kConservativeFourbarTolerance = 0.0;
    public static final double kConservativeFlywheelTolerance = 0.0;

    public static final SuperstructureState kSuperstructureStowState = new SuperstructureState(0.0, new FlywheelState(), IntakeState.IDLE, FourbarSpeed.SONIC, FourbarTolerance.CONSERVATIVE, FlywheelTolerance.LIBERAL);
    public static final SuperstructureState kSuperstructureIntakeState = new SuperstructureState(0.0, new FlywheelState(), IntakeState.IDLE, FourbarSpeed.SONIC, FourbarTolerance.CONSERVATIVE, FlywheelTolerance.LIBERAL);
    public static final SuperstructureState kSuperstructureSpeakerCloseFrontState = new SuperstructureState(0.0, new FlywheelState(), IntakeState.IDLE, FourbarSpeed.SONIC, FourbarTolerance.CONSERVATIVE, FlywheelTolerance.LIBERAL);
    public static final SuperstructureState kSuperstructureSpeakerCloseSideState = new SuperstructureState(0.0, new FlywheelState(), IntakeState.IDLE, FourbarSpeed.SONIC, FourbarTolerance.CONSERVATIVE, FlywheelTolerance.LIBERAL);
    public static final SuperstructureState kSuperstructureReadyMiddleState = new SuperstructureState(0.0, new FlywheelState(), IntakeState.IDLE, FourbarSpeed.SONIC, FourbarTolerance.CONSERVATIVE, FlywheelTolerance.LIBERAL);
  }

  public static class FlywheelConstants {
    public static double kS_Top = 0.188;
    public static double kS_Bottom = 0.166;
    // Math: (V * S / m) / 60 sec / 39.37 in/m * circumference of flywheel
    public static double kV_Top = 0.0020014125023555073743193198053;
    public static double kV_Bottom = 0.0020614125023555073743193198053;
    public static double kA = 0;

    public static double kFlywheelAllowableError = 50;
  }

}
