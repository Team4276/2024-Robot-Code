package frc.team4276.frc2024.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public double positionRads = 0.0;
        public double absoluteEncoderPositionRads = 0.0;
        public double relativeEncoderPositionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] currentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
        public boolean absoluteEncoderConnected = true;
    }

    default void updateInputs(ArmIOInputs inputs) {
    }

    /** Run to setpoint angle in radians */
    default void runSetpoint(double setpointRads, double ff) {
    }

    /** Run to setpoint angle in radians */
    default void runSetpoint(double setpointRads) {
    }

    /** Run motors at volts */
    default void runVolts(double volts) {
    }

    /** Run motors at current */
    default void runCurrent(double amps) {
    }

    /** Set brake mode enabled */
    default void setBrakeMode(boolean enabled) {
    }

    /** Set PID values */
    default void setPID(double p, double i, double d) {
    }

    /** Stops motors */
    default void stop() {
    }
}
