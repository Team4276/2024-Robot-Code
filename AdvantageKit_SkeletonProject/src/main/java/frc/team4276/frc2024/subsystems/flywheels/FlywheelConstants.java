package frc.team4276.frc2024.subsystems.flywheels;

import com.revrobotics.CANSparkBase.IdleMode;

public class FlywheelConstants {
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
  public static double kA = 0.0;

  public static double kPrepRpm = 2000.0;
  public static double kAmpTopRpm = 0.0;
  public static double kAmpBottomRpm = 2000.0;
  public static double kBlindSubRpm = 4000.0;
  public static double kBlindPodiumRpm = 4500.0;
  public static double kBlindFerryRpm = 4500.0;
  public static double kBlindPoopRpm = 4500.0;

  public static double kFlywheelTolerance = 300.0; // RPM
}
