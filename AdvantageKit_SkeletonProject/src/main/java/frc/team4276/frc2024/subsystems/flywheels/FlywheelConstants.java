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

    public static final int kNormalShotRPM = 3500;
    public static final int kFerryRPM = 3500;
    public static final int kSpinUpRPM = 2500;
    public static final int kPoopTopRPM = 3500;
    public static final int kPoopBotRPM = 3200;
    public static final int kExhaustRPM = -1000;
    public static final int kAmpTopRPM = 150;
    public static final int kAmpBotRPM = 1450;

    public static double kFlywheelTolerance = 300.0; // RPM
}
