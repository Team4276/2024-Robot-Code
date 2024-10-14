package frc.team4276.frc2024.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2024.Ports;
import frc.team4276.lib.rev.RevUtil;
import frc.team4276.lib.rev.SparkMaxFactory;

public class DriveConstants {
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

    public static final double kFreeSpeedRpm = 5676;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
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

    public static final SparkMaxFactory.CANSparkMaxPIDFConfig kDrivingPIDFConfig = new SparkMaxFactory.CANSparkMaxPIDFConfig();

    static {
        kDrivingPIDFConfig.kP = 0.04;
        kDrivingPIDFConfig.kFF = 1 / kDriveWheelFreeSpeedRps;
        kDrivingPIDFConfig.kPIDOutputRange = 1.0;
    }

    public static final SparkMaxFactory.CANSparkMaxPIDFConfig kTurningPIDFConfig = new SparkMaxFactory.CANSparkMaxPIDFConfig();

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

    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kFrontLeftDriveId = Ports.FRONT_LEFT_DRIVE;
    public static final int kFrontLeftTurnId = Ports.FRONT_LEFT_TURN;
    public static final double kFrontLeftOffset = -Math.PI / 2;

    public static final int kFrontRightDriveId = Ports.FRONT_RIGHT_DRIVE;
    public static final int kFrontRightTurnId = Ports.FRONT_RIGHT_TURN;
    public static final double kFrontRightOffset = 0;

    public static final int kBackLeftDriveId = Ports.BACK_LEFT_DRIVE;
    public static final int kBackLeftTurnId = Ports.BACK_LEFT_TURN;
    public static final double kBackLeftOffset = Math.PI;

    public static final int kBackRightDriveId = Ports.BACK_RIGHT_DRIVE;
    public static final int kBackRightTurnId = Ports.BACK_RIGHT_TURN;
    public static final double kBackRightOffset = Math.PI / 2;

    public static final double kMaxVel = kDriveWheelFreeSpeedRps; // meters per second
    public static final double kMaxAttainableVel = kMaxVel * 0.8;
    public static final double kMaxAttainableAccel = 8.9;

    public static final double kMaxAngularVel = kMaxVel * 2 / (kTrackWidth * Math.sqrt(2)); // radians per second

    public static final double kAutoTranslationKp = 3.0;
    public static final double kAutoTranslationKd = 0.0;
    public static final double kAutoRotationKp = 4.0;
    public static final double kAutoRotationKd = 0.0;

    // public static final PIDConstants kAutoTranslationPIDConstants =
    // new PIDConstants(kAutoTranslationKp, 0, kAutoTranslationKd);
    // public static final PIDConstants kAutoRotationPIDConstants =
    // new PIDConstants(kAutoRotationKp, 0, kAutoRotationKd);

    public static final double kAutoAccelFF = 0.0;

    public static final double kAutoMaxError = 0.75; // Meters

    public static final double kSnapHeadingKp = 3.0;
    public static final double kSnapHeadingKi = 0.0;
    public static final double kSnapHeadingKd = 0.0;

    public static final double kSnapPositionTolerance = 0.1;
    public static final double kSnapAngularVelocityTolerance = 0.1;
}
