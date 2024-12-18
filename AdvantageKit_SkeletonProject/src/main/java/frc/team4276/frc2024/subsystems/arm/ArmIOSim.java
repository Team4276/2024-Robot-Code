package frc.team4276.frc2024.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.team4276.frc2024.Constants;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ArmConstants.kFeedForwardConstants.kGearRatio, 
            1.0, 
            ArmConstants.kFeedForwardConstants.kMotorLegLength, 
            Math.toRadians(50.0), 
            Math.toRadians(135.0), 
            false, 
            Math.toRadians(90.0));

    private final PIDController controller;
    private double appliedVoltage = 0.0;

    public ArmIOSim() {
        controller = new PIDController(0.02, 0.0001, 0.0);
        controller.setIZone(1.0);
        controller.setIntegratorRange(-0.01, 0.01);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        sim.update(Constants.kLooperDt);

        inputs.positionRads = sim.getAngleRads();
        inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = new double[] {appliedVoltage};
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.tempCelcius = new double[] {0.0};
    }

    /** Run to setpoint angle in radians */
    @Override
    public void runSetpoint(double setpointRads, double ff) {
        runVolts(controller.calculate(sim.getAngleRads(), setpointRads) + ff);
    }

    /** Run to setpoint angle in radians */
    @Override
    public void runSetpoint(double setpointRads) {
        runSetpoint(setpointRads, 0.0);
    }

    /** Run motors at volts */
    @Override
    public void runVolts(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    /** Run motors at current */
    @Override
    public void runCurrent(double amps) {
    }

    /** Set brake mode enabled */
    @Override
    public void setBrakeMode(boolean enabled) {
    }

    /** Set PID values */
    @Override
    public void setPID(double p, double i, double d) {
    }

    /** Stops motors */
    @Override
    public void stop() {
        appliedVoltage = 0.0;
        sim.setInputVoltage(appliedVoltage);
    }
}
