package frc.team4276.frc2024.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2024.Constants;
import frc.team4276.lib.feedforwards.IFeedForward;
import frc.team4276.lib.rev.VIKSparkMax;

public class ArmIOSparkMax implements ArmIO {
    private final VIKSparkMax leaderSparkMax;
    private final VIKSparkMax followerSparkMax;
    private final AbsoluteEncoder absoluteEncoder;

    private TrapezoidProfile.Constraints profileConstraints = new TrapezoidProfile.Constraints(80.0, 60.0);
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
    private IFeedForward ff;

    public ArmIOSparkMax(IFeedForward ff) {
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

            followerSparkMax.follow(leaderSparkMax, true);
            followerSparkMax.enableVoltageCompensation(12.0);
            followerSparkMax.setInverted(true);
            followerSparkMax.setSmartCurrentLimit(40);
            followerSparkMax.setWantBrakeMode(true);

            absoluteEncoder.setInverted(false);
            absoluteEncoder.setPositionConversionFactor(360.0);
            absoluteEncoder.setVelocityConversionFactor(360.0);
            absoluteEncoder.setZeroOffset(214.5);

            SparkPIDController pidController = leaderSparkMax.getPIDController();
            pidController.setFeedbackDevice(absoluteEncoder);
            pidController.setP(0.05);

        }

        leaderSparkMax.burnFlash();
        followerSparkMax.burnFlash();

        leaderSparkMax.setCANTimeout(0);
        followerSparkMax.setCANTimeout(0);

        profile = new TrapezoidProfile(profileConstraints);
        this.ff = ff;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.absoluteEncoderPositionRads = absoluteEncoder.getPosition();
        inputs.velocityRadsPerSec = absoluteEncoder.getVelocity();
        
        inputs.appliedVolts[0] = leaderSparkMax.getAppliedVoltage();
        inputs.currentAmps[0] = leaderSparkMax.getOutputCurrent();
        inputs.tempCelcius[0] = leaderSparkMax.getMotorTemperature();

        inputs.appliedVolts[1] = followerSparkMax.getAppliedVoltage();
        inputs.currentAmps[1] = followerSparkMax.getOutputCurrent();
        inputs.tempCelcius[1] = followerSparkMax.getMotorTemperature();
    }

    @Override
    public void runSetpoint(double setpointRads) {
        double setpointDeg = Units.radiansToDegrees(setpointRads);

        setpointState = profile.calculate(Constants.kLooperDt, setpointState,
                new TrapezoidProfile.State(setpointDeg, 0.0));

        leaderSparkMax.setReference(setpointState.position, ControlType.kPosition, 0,
                ff.calculate(setpointState.position, setpointState.velocity, 0.0), ArbFFUnits.kVoltage);
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

        setpointState = new TrapezoidProfile.State(absoluteEncoder.getPosition(), 0);
    }

}
