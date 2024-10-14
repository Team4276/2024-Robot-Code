package frc.team4276.frc2024.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;
// shamelessly pasted from 6328

public interface FlywheelIO {
  @AutoLog
  class FlywheelsIOInputs {
    public boolean topMotorConnected = true;
    public boolean bottomMotorConnected = true;

    public double topPositionRads = 0.0;
    public double topVelocityRpm = 0.0;
    public double topAppliedVolts = 0.0;
    public double topSupplyCurrentAmps = 0.0;
    public double topTorqueCurrentAmps = 0.0;
    public double topTempCelsius = 0.0;

    public double bottomPositionRads = 0.0;
    public double bottomVelocityRpm = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomSupplyCurrentAmps = 0.0;
    public double bottomTorqueCurrentAmps = 0.0;
    public double bottomtTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double topVolts, double bottomVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  default void runVelocity(double topFeedforward, double bottomFeedforward) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationTop(double input) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationBottom(double input) {}
}
