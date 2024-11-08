package frc.team4276.frc2024.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.lib.rev.SparkMaxFactory;
import frc.team4276.lib.rev.VIKSparkMax;

public class ModuleIOSparkMax implements ModuleIO {
    public static class ModuleConfig {
        public String kName = "ERROR_NO_NAME";
        public int kDriveId = -1;
        public int kTurnId = -1;
        public double kOffset = 0;
    }

    private VIKSparkMax driveMotor;
    private VIKSparkMax turnMotor;

    private AbsoluteEncoder turnAbsoluteEncoder;
    private double absoluteEncoderOffset;

    private RelativeEncoder driveEncoder;

    private SparkPIDController drivePid;
    private SparkPIDController turnPid;

    public ModuleIOSparkMax(ModuleConfig config) {
        // Init motor & encoder objects
        driveMotor = SparkMaxFactory.createDefault(config.kDriveId);
        turnMotor = SparkMaxFactory.createDefault(config.kTurnId);
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
        absoluteEncoderOffset = config.kOffset;
        driveEncoder = driveMotor.getEncoder();

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();
        driveMotor.setCANTimeout(250);
        turnMotor.setCANTimeout(250);

        for (int i = 0; i < 30; i++) {
            driveMotor.setSmartCurrentLimit(50);
            driveMotor.enableVoltageCompensation(12.0);
            driveMotor.setWantBrakeMode(true);
            
            turnMotor.setSmartCurrentLimit(20);
            turnMotor.enableVoltageCompensation(12.0);
            turnMotor.setWantBrakeMode(true);
            turnMotor.setInverted(true);

            driveEncoder.setPositionConversionFactor(DriveConstants.kDrivingEncoderPositionFactor);
            driveEncoder.setVelocityConversionFactor(DriveConstants.kDrivingEncoderVelocityFactor);
            driveEncoder.setPosition(0.0);

            turnAbsoluteEncoder.setInverted(true);
            turnAbsoluteEncoder.setPositionConversionFactor(DriveConstants.kTurningEncoderPositionFactor);
            turnAbsoluteEncoder.setVelocityConversionFactor(DriveConstants.kTurningEncoderVelocityFactor);

            drivePid = driveMotor.getPIDController();
            drivePid.setFeedbackDevice(driveEncoder);
            drivePid.setP(DriveConstants.kDrivingPIDFConfig.kP);
            drivePid.setFF(DriveConstants.kDrivingPIDFConfig.kFF);
            drivePid.setPositionPIDWrappingEnabled(true);
            drivePid.setPositionPIDWrappingMaxInput(DriveConstants.kTurningEncoderPositionPIDMaxInput);
            drivePid.setPositionPIDWrappingMinInput(DriveConstants.kTurningEncoderPositionPIDMinInput);

            turnPid = turnMotor.getPIDController();
            turnPid.setFeedbackDevice(turnAbsoluteEncoder);
            turnPid.setP(DriveConstants.kTurningPIDFConfig.kP);

        }

        driveMotor.burnFlash();
        turnMotor.burnFlash();

        driveMotor.setCANTimeout(0);
        turnMotor.setCANTimeout(0);
    }

        /** Updates the set of loggable inputs. */
        public void updateInputs(ModuleIOInputs inputs) {
            inputs.drivePositionMetres = driveEncoder.getPosition();
            inputs.driveVelocityMetresPerSec = driveEncoder.getVelocity();
            inputs.driveAppliedVolts = driveMotor.getAppliedVoltage();
            inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();

            inputs.turnPositionRads = turnAbsoluteEncoder.getPosition();
            inputs.turnVelocityRadsPerSec = turnAbsoluteEncoder.getVelocity();
            inputs.turnAppliedVolts = turnMotor.getAppliedVoltage();
            inputs.turnSupplyCurrentAmps = turnMotor.getOutputCurrent();

            inputs.odometryDrivePositionsMeters = new double[] {};
            inputs.odometryTurnPositions = new Rotation2d[] {};
        }
    
        /** Run drive motor at volts */
        public void runDriveVolts(double volts) {
            driveMotor.setVoltage(volts);
        }
    
        /** Run turn motor at volts */
        public void runTurnVolts(double volts) {
            turnMotor.setVoltage(volts);
        }
    
        /** Run characterization input (amps or volts) into drive motor */
        public void runCharacterization(double input) {
        }
    
        /** Run to drive velocity setpoint with feedforward */
        public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
            driveMotor.setReference(velocityRadsPerSec, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
        }
    
        /** Run to turn position setpoint */
        public void runTurnPositionSetpoint(double angleRads) {
            turnMotor.setReference(angleRads, ControlType.kPosition, 0, 0.0, ArbFFUnits.kVoltage);
        }
    
        /** Configure drive PID */
        public void setDrivePID(double kP, double kI, double kD) {
        }
    
        /** Configure turn PID */
        public void setTurnPID(double kP, double kI, double kD) {
        }
    
        /** Enable or disable brake mode on the drive motor. */
        public void setDriveBrakeMode(boolean enable) {
        }
    
        /** Enable or disable brake mode on the turn motor. */
        public void setTurnBrakeMode(boolean enable) {
        }
    
        /** Disable output to all motors */
        public void stop() {
        }
}
