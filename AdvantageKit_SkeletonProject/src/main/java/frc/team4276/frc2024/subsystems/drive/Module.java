package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class Module {
    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModuleState setpointState = new SwerveModuleState();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    /** Called while blocking odometry thread */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    /** Runs to {@link SwerveModuleState} */
    public void runSetpoint(SwerveModuleState setpoint) {
        setpointState = SwerveModuleState.optimize(setpoint, getAngle());

        io.runDriveVelocitySetpoint(
                setpointState.speedMetersPerSecond / DriveConstants.kWheelRadiusMeters, 0.0);
        io.runTurnPositionSetpoint(setpointState.angle.getRadians());
    }

    public SwerveModuleState getSetpointState() {
        return setpointState;
    }

    /**
     * Runs characterization volts or amps depending on using voltage or current
     * control.
     */
    public void runCharacterization(double turnSetpointRads, double input) {
        io.runTurnPositionSetpoint(turnSetpointRads);
        io.runCharacterization(input);
    }

    /** Sets brake mode to {@code enabled}. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Stops motors. */
    public void stop() {
        io.stop();
    }

    /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
    public SwerveModulePosition[] getModulePositions() {
        int minOdometryPositions = Math.min(inputs.odometryDrivePositionsMeters.length,
                inputs.odometryTurnPositions.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
        for (int i = 0; i < minOdometryPositions; i++) {
            positions[i] = new SwerveModulePosition(
                    inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
        }
        return positions;
    }

    /** Get turn angle of module as {@link Rotation2d}. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Get position of wheel rotations in radians */
    public double getPositionRads() {
        return inputs.drivePositionRads;
    }

    /** Get position of wheel in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRads * DriveConstants.kWheelRadiusMeters;
    }

    /** Get velocity of wheel in m/s. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadsPerSec * DriveConstants.kWheelRadiusMeters;
    }

    /** Get current {@link SwerveModulePosition} of module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Get current {@link SwerveModuleState} of module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadsPerSec;
    }
}
