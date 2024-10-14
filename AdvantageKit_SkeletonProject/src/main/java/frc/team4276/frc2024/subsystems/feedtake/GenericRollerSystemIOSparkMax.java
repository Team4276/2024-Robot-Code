package frc.team4276.frc2024.subsystems.feedtake;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.team4276.lib.rev.SparkMaxFactory;
import frc.team4276.lib.rev.VIKSparkMax;

public abstract class GenericRollerSystemIOSparkMax implements GenericRollerSystemIO {
    private final VIKSparkMax motor;
    private final RelativeEncoder encoder;

    private final double reduction;

    public GenericRollerSystemIOSparkMax(
            int id, int currentLimit, boolean invert, boolean brake, double reduction) {
        this.reduction = reduction;
        motor = SparkMaxFactory.createDefault(id);

        motor.setSmartCurrentLimit(currentLimit);
        motor.setInverted(invert);
        motor.setWantBrakeMode(brake);

        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(GenericRollerSystemIOInputs inputs) {
        inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
        inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyCurrentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
