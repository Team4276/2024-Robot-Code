package frc.team4276.frc2024.subsystems.drive.flywheel;

import org.littletonrobotics.junction.AutoLog;
// shamelessly pasted from 6328

public interface FlywheelIO {
  @AutoLog
  class FlywheelsIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  default void runVelocity(double topFeedforward, double bottomFeedforward) {}

  /** Run left flywheels at voltage */
  // TODO: impl
  default void runCharacterizationLeft(double input) {}

  /** Run right flywheels at voltage */
  // TODO: impl
  default void runCharacterizationRight(double input) {}
}
