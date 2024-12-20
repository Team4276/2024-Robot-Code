package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionMetres = 0.0;
        public double driveVelocityMetresPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;

        public double turnPositionRads = 0.0;
        public double turnVelocityRadsPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnTorqueCurrentAmps = 0.0;

        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run drive motor at volts */
    default void runDriveVolts(double volts) {
    }

    /** Run turn motor at volts */
    default void runTurnVolts(double volts) {
    }

    /** Run characterization input (amps or volts) into drive motor */
    default void runCharacterization(double input) {
    }

    /** Run to drive velocity setpoint with feedforward */
    default void runDriveVelocitySetpoint(double velocityMetresPerSec, double feedForward) {
    }

    /** Run to turn position setpoint */
    default void runTurnPositionSetpoint(double angleRads) {
    }

    /** Configure drive PID */
    default void setDrivePID(double kP, double kI, double kD) {
    }

    /** Configure turn PID */
    default void setTurnPID(double kP, double kI, double kD) {
    }

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveBrakeMode(boolean enable) {
    }

    /** Enable or disable brake mode on the turn motor. */
    default void setTurnBrakeMode(boolean enable) {
    }

    /** Disable output to all motors */
    default void stop() {
    }
}
