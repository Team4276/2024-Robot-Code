package frc.team4276.frc2024.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;

import frc.team4276.lib.rev.VIKSparkMax;

public class ArmIOSparkMax implements ArmIO {
    private final VIKSparkMax leaderSparkMax;
    private final VIKSparkMax followerSparkMax;
    private final AbsoluteEncoder absoluteEncoder;

    public ArmIOSparkMax() {
        leaderSparkMax = new VIKSparkMax(ArmConstants.kMasterId);
        followerSparkMax = new VIKSparkMax(ArmConstants.kFollowerId);
        absoluteEncoder = leaderSparkMax.getAbsoluteEncoder();

        leaderSparkMax.restoreFactoryDefaults();
        followerSparkMax.restoreFactoryDefaults();
        leaderSparkMax.setCANTimeout(250);
        followerSparkMax.setCANTimeout(250);

        for (int i = 0; i < 4; i++) {
            leaderSparkMax.enableVoltageCompensation(12.0);
            leaderSparkMax.setInverted(false);
            leaderSparkMax.setSmartCurrentLimit(40);
            leaderSparkMax.setWantBrakeMode(true);
            leaderSparkMax.setPeriodicFramePeriodSec(PeriodicFrame.kStatus0, 0.005);
            leaderSparkMax.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
            leaderSparkMax.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);

            followerSparkMax.follow(leaderSparkMax, true);
            followerSparkMax.enableVoltageCompensation(12.0);
            followerSparkMax.setInverted(true);
            followerSparkMax.setSmartCurrentLimit(40);
            followerSparkMax.setWantBrakeMode(true);

            absoluteEncoder.setInverted(false);
            absoluteEncoder.setPositionConversionFactor(360.0);
            absoluteEncoder.setVelocityConversionFactor(360.0);
            absoluteEncoder.setZeroOffset(94.5); //TODO: FIND IT

            SparkPIDController pidController = leaderSparkMax.getPIDController();
            pidController.setFeedbackDevice(absoluteEncoder);
            pidController.setP(0.02);
            pidController.setI(0.0001);
            pidController.setD(0.0);
            pidController.setIZone(1.0);
            pidController.setIMaxAccum(0.01, 0);

        }

        leaderSparkMax.burnFlash();
        followerSparkMax.burnFlash();

        leaderSparkMax.setCANTimeout(0);
        followerSparkMax.setCANTimeout(0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.absoluteEncoderPositionRads = Math.toRadians(absoluteEncoder.getPosition()); //TODO: figure out where we want to use units n stuff
        inputs.velocityRadsPerSec = Math.toRadians(absoluteEncoder.getVelocity());
        
        inputs.appliedVolts[0] = leaderSparkMax.getAppliedVoltage();
        inputs.currentAmps[0] = leaderSparkMax.getOutputCurrent();
        inputs.tempCelcius[0] = leaderSparkMax.getMotorTemperature();

        inputs.appliedVolts[1] = followerSparkMax.getAppliedVoltage();
        inputs.currentAmps[1] = followerSparkMax.getOutputCurrent();
        inputs.tempCelcius[1] = followerSparkMax.getMotorTemperature();
    }

    @Override
    public void runSetpoint(double setpointRads, double ff) {
        double setpointDeg = Units.radiansToDegrees(setpointRads);

        leaderSparkMax.setReference(setpointDeg, ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
    }
    
    @Override
    public void runSetpoint(double setpointRads) {
        runSetpoint(setpointRads, 0.0);
    }

    @Override
    public void runVolts(double volts) {
        leaderSparkMax.setVoltage(volts);
    }

    @Override
    public void runCurrent(double amps) {
        //TODO: impl
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderSparkMax.setWantBrakeMode(enabled);
        followerSparkMax.setWantBrakeMode(enabled);
    }

    @Override
    public void setPID(double p, double i, double d) {
        //TODO: impl
    }

    @Override
    public void stop() {
        leaderSparkMax.stopMotor();
        followerSparkMax.stopMotor();
    }

}
