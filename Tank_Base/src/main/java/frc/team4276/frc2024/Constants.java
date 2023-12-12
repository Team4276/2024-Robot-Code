// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

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
  public static final double kLooperDt = 0.02;

  // Enables extra SmartDashboard Debugs
  // Don't use during competition
  public static final boolean disableExtraTelemetry = true;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxVel = 2.5; // meters per second

    public static final double kMaxAngularVel = 2 * Math.PI; // radians per second

    public static class KinematicLimits {
      public double kMaxDriveVelocity = kMaxVel; // m/s
      public double kMaxAccel = Double.MAX_VALUE; // m/s^2
      public double kMaxAngularVelocity = kMaxAngularVel; // rad/s
      public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
  } 

    public static final KinematicLimits kUncappedLimits = new KinematicLimits();
    static {
            kUncappedLimits.kMaxDriveVelocity = kMaxVel;
            kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
            kUncappedLimits.kMaxAngularVelocity = kMaxAngularVel;
            kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
    }

    public static final KinematicLimits kAutoLimits = new KinematicLimits();
    static {
            kAutoLimits.kMaxDriveVelocity = kMaxVel;
            kAutoLimits.kMaxAccel = Double.MAX_VALUE;
            kAutoLimits.kMaxAngularVelocity =  Double.MAX_VALUE; // Rad/Sec
            kAutoLimits.kMaxAngularAccel = Double.MAX_VALUE; // 2 * Math.PI;
    }

    //TODO: fill 
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25);

    //TODO: fill IDs
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 12;

    public static final boolean kGyroReversed = false;
  }

  public static final class SkidConstants {
    //TODO: fill chassis constants
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = 1 * Math.PI;
    public static final double kDrivingMotorReduction = 1;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kPositionFactor = kWheelCircumferenceMeters
        / kDrivingMotorReduction; // meters

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;

    public static final IdleMode kIdleMode = IdleMode.kBrake;

    public static final int kCurrentLimit = 40; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
    public static final double kJoystickDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
